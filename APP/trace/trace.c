#include "trace.h"

// ============================================================================
// 差速底盘运动层（Phase 1 + 2 + 3）
// ----------------------------------------------------------------------------
// 背景说明：
//   本届是「四平地轮差速底盘」，无横移能力。左轮 = 201(左前)+204(左后)，
//   右轮 = 202(右前)+203(右后)。电机板自带闭环，写速度寄存器即可稳速。
//
// 本文件提供：
//   Diff_Drive  —— 差速基础接口
//   Line_Follow —— 循迹（向前用前灰度、向后用后灰度，位置低通 + PD 纠偏 + 丢线记忆回找）
//   Spin        —— 原地旋（开环，调试用）
//   Spin_Angle  —— 原地旋（编码器测角闭环，速度自动平滑）
//   Stop        —— 停车
// ============================================================================

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// 无串口调试变量（Cortex-Debug：在 Watch 窗口直接添加变量名即可实时观察）
//   DBG_pos  本次灰度位置（0~70；-1 全白；-2 全黑）
//   DBG_e    误差；DBG_u P 差速量
//   DBG_vl / DBG_vr 本次下发给左/右轮的速度
//   DBG_lost 0=有位置 1=无有效位置
//   DBG_dir  0=向前循迹 1=向后循迹
// ----------------------------------------------------------------------------
volatile int16_t DBG_pos  = 0;
volatile int16_t DBG_e    = 0;
volatile int16_t DBG_u    = 0;
volatile int16_t DBG_vl   = 0;
volatile int16_t DBG_vr   = 0;
volatile uint8_t DBG_lost = 0;
volatile uint8_t DBG_dir  = 0;
volatile uint8_t DBG_raw  = 0; // 原始灰度字节（bit7~bit0 = 探头1~8，1=该探头读到黑）

// 按 KEY0（Trace_Forward 首次调用）那一刻、车静止时锁存的灰度字节：
//   跑起来后随时暂停，看这个值就知道「起步瞬间」灰度读到什么
volatile uint8_t DBG_boot_raw = 0;

// 灰度滚动记录：最近 32 次的 DBG_raw（DBG_hist_i 指向下一个待写位置）
//   跑完车暂停，在 WATCH 里展开 DBG_hist 就能看出灰度到底变没变
volatile uint8_t DBG_hist[32] = {0};
volatile uint8_t DBG_hist_i   = 0;

// ----------------------------------------------------------------------------
// 循线滤波 / PD / 丢线记忆状态（Line_Follow 内部使用）
//   trace_pos_f / trace_bpos_f  前/后灰度位置的一阶低通滤波值
//   trace_e_last                上一次滤波后误差（KD 微分项用）
//   trace_last_side             丢线前线的方向：-1=线在左，+1=线在右
// ----------------------------------------------------------------------------
static float  trace_pos_f      = 35.0f;
static float  trace_bpos_f     = 35.0f;
static float  trace_e_last     = 0.0f;
static int16_t trace_last_side = 0;
static uint8_t trace_lost_ticks = 0;   // 丢线后累计拍数（扫线换向用）
static int16_t trace_search_dir = 1;   // 丢线且方向未知时的扫线方向：+1/-1

// ----------------------------------------------------------------------------
// 差速底盘基础接口
//   左轮 = Motor_201(左前) + Motor_204(左后)
//   右轮 = Motor_202(右前) + Motor_203(右后)
// 参数：V_L / V_R 为左右两侧目标速度，正=前进、负=后退（符号待实车标定）
// ----------------------------------------------------------------------------
uint8_t Diff_Drive(int16_t V_L, int16_t V_R)
{
#if TRACE_MOTOR_DISABLE
    (void)V_L;
    (void)V_R;
    return 0;
#else
    Motor_201.Target_Speed = V_L; // 左前
    Motor_204.Target_Speed = V_L; // 左后
    Motor_202.Target_Speed = V_R; // 右前
    Motor_203.Target_Speed = V_R; // 右后

    Motor_201.PID_Output = Motor_201.Target_Speed;
    Motor_202.PID_Output = Motor_202.Target_Speed;
    Motor_203.PID_Output = Motor_203.Target_Speed;
    Motor_204.PID_Output = Motor_204.Target_Speed;

    // 发送顺序为板子通道约定（203,202,204,201），保持不动
    return MOTOR_CONTROL_DATA_Predit(&Motor_203, &Motor_202, &Motor_204, &Motor_201);
#endif
}

// ----------------------------------------------------------------------------
// 原地旋（开环）：左右轮反向，绕车中心旋转，用于调试/简单转弯
// 参数：v 为旋转速度，符号待实车标定
// ----------------------------------------------------------------------------
void Spin(int16_t v)
{
    Diff_Drive(v, -v);
}

// ----------------------------------------------------------------------------
// 停车
// ----------------------------------------------------------------------------
void Stop(void)
{
    (void)Diff_Drive(0, 0);
}

// ----------------------------------------------------------------------------
// 可靠停车：反复下发零速直到某一帧被驱动板接收（返回 1=确认写进板子）
//   用途：驱动板一旦收不到帧就会保持上一次速度（车不停），单次 Stop() 不够可靠。
// ----------------------------------------------------------------------------
uint8_t Stop_Retry(uint8_t tries)
{
    uint8_t i;

    for (i = 0; i < tries; i++) {
        if (Diff_Drive(0, 0) == 0) return 1;
        delay_ms(5);
    }
    return 0;
}

// ----------------------------------------------------------------------------
// 循迹封装（无参）：向前用前灰度、向后用后灰度
//   KEY0 按下后立即开始循线，不再做任何「点头/扭动」自检（那是之前转圈的来源）。
//   首次调用只把静止时的灰度字节锁存到 DBG_boot_raw，并复位滤波/丢线状态。
// ----------------------------------------------------------------------------
void Trace_Forward(void)
{
    static uint8_t first = 1;

    if (first) {
        first = 0;

        DBG_boot_raw = F_GW_digital_Read(); // 起步瞬间灰度，保留给调试器看，不动车
        trace_pos_f      = 35.0f;
        trace_bpos_f     = 35.0f;
        trace_e_last     = 0.0f;
        trace_last_side  = 0;
        trace_lost_ticks = 0;
        trace_search_dir = 1;
    }

    Line_Follow(LINE_SPEED);
}

// ----------------------------------------------------------------------------
// 向后循迹（后灰度 PD）
//   2026-10-25：首次接入导航（A 点取料后倒退回圆心），之前是死代码、从未实车跑过。
//   补上首次复位块：否则切到反向时滤波值还是初值 35、trace_e_last 还留着
//   前进段的最后一个误差 → 反向第一拍 D 项突跳，起步会甩一下。
// ----------------------------------------------------------------------------
void Trace_Backward(void)
{
    static uint8_t first = 1;

    if (first) {
        first = 0;

        trace_bpos_f     = 35.0f;  // 后灰度位置低通复位
        trace_e_last     = 0.0f;   // 清掉前进段残留误差，避免第一拍 D 突跳
        trace_last_side  = 0;
        trace_lost_ticks = 0;
        trace_search_dir = 1;
    }

    Line_Follow(-TRACE_BACK_SPEED);
}

// ----------------------------------------------------------------------------
// 读 4 路电机编码器累计脉冲（**int32**，与官方例程一致）
// 编码器顺序与速度写入顺序一致：203(右后) 202(右前) 204(左后) 201(左前)
//
// 【2026-10-25 第十五轮：按官方例程改成 32 位】
// 官方两种例程都是把 16 字节**直接当 int32_t 数组用**：
//     int32_t EncodeTotal[4];                     // Arduino: int32_t，PDF: long int
//     WireReadDataArray(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)EncodeTotal,16);
//     printf("%ld", EncodeTotal[0..3]);
// 即：小端、每个电机 4 字节、有符号 32 位累计值。
// 我们之前只取低 2 字节（int16）→ 16 位在 8 圈左右就回绕，长距离标定会算错。
//
// 返回：0=读成功  1=读失败（全 0xFF = 总线悬空，enc 已清零，调用方跳过）
// ----------------------------------------------------------------------------
uint8_t Encoder_ReadRaw(int32_t enc[4])
{
    uint8_t buf[16];
    uint8_t i;
    uint8_t all_ff = 1;
    uint8_t ret    = I2C_Read_Len(MOTOR_ENCODER_TOTAL_ADDR, buf, 16);

    /* 这块驱动板的从机几乎不应答 ACK，但字节内容正常，所以 I2C_Read_Len
       不再因 ACK 失败返回错误。这里改用数据本身做有效性判断：
       16 字节全是 0xFF = 总线悬空（没有器件驱动，被上拉读成 0xFF），判为失败。
       全 0x00 是合法值（编码器正好在 0 位），不能当失败。 */
    for (i = 0; i < 16; i++) {
        if (buf[i] != 0xFF) {
            all_ff = 0;
            break;
        }
    }

    if (ret != 0 || all_ff) {
        for (i = 0; i < 4; i++) enc[i] = 0;
        return 1;
    }

    /* 每电机 4 字节小端拼成 int32（与官方 EncodeTotal[4] 完全对应） */
    for (i = 0; i < 4; i++) {
        uint8_t *p = &buf[i * 4];

        enc[i] = (int32_t)((uint32_t)p[0] |
                           ((uint32_t)p[1] << 8) |
                           ((uint32_t)p[2] << 16) |
                           ((uint32_t)p[3] << 24));
    }
    return 0; // 0=成功  1=读失败
}

// ----------------------------------------------------------------------------
// 原地旋的转角度量：左轮脉冲 - 右轮脉冲
// 原地旋时左右轮反向，左-右 的脉冲差正比于车体转角（符号待标定）
// ----------------------------------------------------------------------------
int32_t Spin_Encoder(void)
{
    int32_t enc[4];
    Encoder_ReadRaw(enc);
    int32_t left  = (int32_t)enc[2] + enc[3]; // 204 + 201
    int32_t right = (int32_t)enc[0] + enc[1]; // 203 + 202
    return left - right;
}

// ----------------------------------------------------------------------------
// 走行里程：左右轮脉冲和。直线走 ≈ 2×里程，原地旋 ≈ 0（左右抵消）
// 用于圆心全黑测距（距离与车速无关，比「采样次数」更稳健）
// ----------------------------------------------------------------------------
int32_t Encoder_Odometer(void)
{
    int32_t enc[4];
    Encoder_ReadRaw(enc);
    int32_t left  = (int32_t)enc[2] + enc[3]; // 204 + 201
    int32_t right = (int32_t)enc[0] + enc[1]; // 203 + 202
    return left + right;
}

// ----------------------------------------------------------------------------
// 循线（位置低通滤波 + PD 纠偏 + 丢线记忆回找）
//   pos: 0~70 = 线位置（0 最左、70 最右）；-1 = 全白；-2 = 全黑
//   V > 0 向前（前灰度）；V < 0 向后（后灰度，差速方向镜像）
//   有效位置：先一阶低通滤波压掉 8 路数字量的离散跳变，再对滤波后误差做 PD。
//   全黑：交点/宽线，保持直行通过（不停车、不打转）。
//   全白：丢线，向前时朝「丢线前线的方向」小角度画弧回找，不再原地快旋。
// ----------------------------------------------------------------------------

// 对滤波后误差 e 做 PD，并把差速写入左右轮
//   e 符号：正=线偏右（左轮快、右轮慢 → 车右转），负=线偏左（车左转）
//   V>=0 向前：vl=V+u, vr=V-u；V<0 向后：差速方向镜像
static void Trace_Output(float e, int16_t V)
{
    float   u = KP_LINE * e + KD_LINE * (e - trace_e_last);
    int16_t u_lim, ui;
    int16_t vl, vr;

    trace_e_last = e;

    // 差速量既不能超过 U_LIMIT，也不能让任一侧轮子反向（否则变成原地打转）
    u_lim = (V >= 0) ? (V - 1) : (-V - 1);
    if (u_lim > U_LIMIT) u_lim = U_LIMIT;
    if (u_lim < 1) u_lim = 1;

    if (u > (float)u_lim) u = (float)u_lim;
    if (u < -(float)u_lim) u = -(float)u_lim;

    ui = (int16_t)(u + (u >= 0.0f ? 0.5f : -0.5f)); // 四舍五入，避免小偏差被截断成 0
    if (ui > u_lim) ui = u_lim;
    if (ui < -u_lim) ui = -u_lim;

    DBG_e = (int16_t)e;
    DBG_u = ui;

    if (V >= 0) {
        vl = (int16_t)(V + ui);
        vr = (int16_t)(V - ui);
    } else {
        vl = (int16_t)(V - ui);
        vr = (int16_t)(V + ui);
    }

    DBG_vl = vl;
    DBG_vr = vr;
    Diff_Drive(vl, vr);
}

void Line_Follow(int16_t V)
{
    uint8_t fwd = (V >= 0) ? 1u : 0u;
    int16_t pos;
    float   e = 0.0f;
    float  *pf;

    pos = fwd ? F_GW_position() : B_GW_position();
    pf  = fwd ? &trace_pos_f : &trace_bpos_f;

    /* ---- 状态灯（低电平点亮，每周期实时刷新）----
       红灯亮 = 线偏左    绿灯亮 = 线偏右
       双灯灭 = 线居中    双灯亮 = 无有效位置（全白/全黑/读失败） */
#if LINE_LED_DEBUG
    if (pos < 0) {
        LED_RED   = 0;
        LED_GREEN = 0;
    } else if (pos < 35) {
        LED_RED   = 0;
        LED_GREEN = 1;
    } else if (pos > 35) {
        LED_RED   = 1;
        LED_GREEN = 0;
    } else {
        LED_RED   = 1;
        LED_GREEN = 1;
    }
#endif

    /* 原始灰度字节：WATCH 里看它就知道哪几路探头在读「黑」
       例: 0x01=只有最右一路黑；0x80=只有最左一路黑；0x00=全白；0xFF=全黑 */
    DBG_raw = fwd ? F_GW_digital_Read() : B_GW_digital_Read();

    /* 滚动记录最近 32 次（跑完暂停，展开 DBG_hist 看灰度是否在变化） */
    DBG_hist[DBG_hist_i] = DBG_raw;
    DBG_hist_i           = (uint8_t)((DBG_hist_i + 1) & 31);

    DBG_pos = pos;
    DBG_dir = fwd ? 0u : 1u;

    if (pos >= 0) {
        /* 有位置：低通滤波后做 PD 纠偏 */
        *pf += POS_FILTER_ALPHA * ((float)pos - *pf);
        e = fwd ? (*pf - 35.0f) : (35.0f - *pf);
        e = LINE_SIGN * e;

        if (e < -0.5f) trace_last_side = -1;
        else if (e > 0.5f) trace_last_side = 1;

        trace_lost_ticks = 0;
        trace_search_dir = 1;
        DBG_lost = 0;
        Trace_Output(e, V);
        return;
    }

    DBG_lost = 1;

    if (pos == -2) {
        /* 全黑：交点/宽线/圆心，直行通过 */
        DBG_vl = V;
        DBG_vr = V;
        Diff_Drive(V, V);
        trace_e_last = 0.0f;
        trace_lost_ticks = 0;
    } else {
        /* 全白：丢线。向前朝丢线前方向慢速回找；
           方向未知时左右慢速扫线。
           2026-10-25：向后不再「保持直行」——长距离倒退丢线后直行会一路漂，
           改成和前进同源：朝丢线前方向画弧回找（trace_last_side 的语义
           在 pos>=0 分支里已按反向镜像，可直接复用）。 */
        if (trace_last_side != 0) {
            e = (float)(trace_last_side * LOST_TURN);
        } else {
            trace_lost_ticks++;
            if (trace_lost_ticks >= LOST_SWEEP_TICKS) {
                trace_lost_ticks = 0;
                trace_search_dir = -trace_search_dir;
            }
            e = (float)(trace_search_dir * LOST_TURN);
        }
        trace_e_last = e; // 丢线阶段不用微分，避免找回瞬间的方向突变
        Trace_Output(e, V);
    }
}

// ----------------------------------------------------------------------------
// 原地旋（编码器测角闭环，平滑转弯）
// 参数：deg 为目标转角（带符号，正=逆时针，符号待实车标定）
// 原理：比例控制 + 速度限幅 + 最低速 + 死区
//   err = 目标脉冲 - 已转脉冲，err 大时转速快，接近目标自动减速（丝滑）
// ----------------------------------------------------------------------------
void Spin_Angle(int16_t deg)
{
    int32_t target = (int32_t)deg * ENC_PER_DEG;
    int32_t start  = Spin_Encoder();
    int32_t err;
    int16_t v;

    while (1) {
        err = target - (Spin_Encoder() - start);

        // 到位（进入死区）
        if (err > -TURN_DEAD && err < TURN_DEAD)
            break;

        v = (int16_t)(TURN_KP * err);

        if (v > TURN_MAX) v = TURN_MAX;
        if (v < -TURN_MAX) v = -TURN_MAX;
        if (v >= 0 && v < TURN_MIN) v = TURN_MIN;
        if (v < 0 && v > -TURN_MIN) v = -TURN_MIN;

        Diff_Drive(v, -v);
    }

    Stop();
}
