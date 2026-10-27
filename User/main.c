#include "main.h"

// 电机映射测试速度：正值方向待实车标定，40 为中等转速（速度寄存器 int8_t 范围 -127~127）
#define MOTOR_TEST_SPEED 40

/* ============================================================================
 * 以下为历史调试函数，暂时不用（需要恢复时，去掉本段开头的块注释起始标记
 * 与结尾的块注释结束标记即可）
 * ============================================================================

// ----------------------------------------------------------------------------
// 调试辅助：单独驱动某一个电机（其余三个停转）
// 用途：确认电机映射是否正确（201=左前 202=右前 203=右后 204=左后）
// ----------------------------------------------------------------------------
static void Single_Motor(int motor_id, int8_t speed)
{
    Motor_201.Target_Speed = 0;
    Motor_202.Target_Speed = 0;
    Motor_203.Target_Speed = 0;
    Motor_204.Target_Speed = 0;

    switch (motor_id) {
        case 201:
            Motor_201.Target_Speed = speed;
            break;
        case 202:
            Motor_202.Target_Speed = speed;
            break;
        case 203:
            Motor_203.Target_Speed = speed;
            break;
        case 204:
            Motor_204.Target_Speed = speed;
            break;
        default:
            break;
    }

    Motor_201.PID_Output = Motor_201.Target_Speed;
    Motor_202.PID_Output = Motor_202.Target_Speed;
    Motor_203.PID_Output = Motor_203.Target_Speed;
    Motor_204.PID_Output = Motor_204.Target_Speed;

    MOTOR_CONTROL_DATA_Predit(&Motor_203, &Motor_202, &Motor_204, &Motor_201);
}

// ----------------------------------------------------------------------------
// KEY0 调试：四轮顺时针依次运行 5 秒，肉眼确认电机映射
// ----------------------------------------------------------------------------
static void Motor_Map_Test(void)
{
    printf("\r\n==== Motor Map Test (KEY0) ====\r\n");

    printf("1/4 left-front  201\r\n");
    Single_Motor(201, MOTOR_TEST_SPEED);
    delay_ms(5000);

    printf("2/4 right-front 202\r\n");
    Single_Motor(202, MOTOR_TEST_SPEED);
    delay_ms(5000);

    printf("3/4 right-back  203\r\n");
    Single_Motor(203, MOTOR_TEST_SPEED);
    delay_ms(5000);

    printf("4/4 left-back   204\r\n");
    Single_Motor(204, MOTOR_TEST_SPEED);
    delay_ms(5000);

    Stop();
    printf("done\r\n");
}

// ----------------------------------------------------------------------------
// KEY1 调试：循环读取灰度，打印 raw + 8 探头位 + 加权位置
// ----------------------------------------------------------------------------
static void Gray_Debug(void)
{
    uint8_t raw;
    int16_t e;
    uint8_t i;

    printf("\r\n==== Gray Debug (KEY1) ====\r\n");

    while (1) {
        raw = F_GW_digital_Read();
        e   = F_GW_digital_eight();

        printf("raw=0x%02X bits=", (unsigned)raw);
        for (i = 0; i < 8; i++) {
            printf("%d", (raw >> (7 - i)) & 0x01);
        }
        printf(" eight=%d\r\n", e);

        delay_ms(200);
    }
}

// ----------------------------------------------------------------------------
// KEY1 调试：舵机夹爪测试（先 SET 张开 → 再 RESET 收紧）
// ----------------------------------------------------------------------------
static void Steering_Test(void)
{
    printf("\r\n==== Steering Test (KEY1) ====\r\n");
    printf("SET(1.98ms open) -> RESET(1.32ms tighten), 2s each\r\n");

    while (1) {
        Steering_SET();
        printf("Steering_SET   -> 1.98ms (open)\r\n");
        delay_ms(2000);
        Steering_RESET();
        printf("Steering_RESET -> 1.32ms (tighten)\r\n");
        delay_ms(2000);
    }
}

// ----------------------------------------------------------------------------
// KEY0 调试：一直向前循迹（前灰度），纯控制循环
// ----------------------------------------------------------------------------
static void Trace_Forward_Test(void)
{
    while (1) {
        Line_Follow(LINE_SPEED);
        delay_ms(10);
    }
}

// ----------------------------------------------------------------------------
// KEY1 调试：灰度 LED 诊断（红=丢线/绿=全黑/双亮=偏左/双灭=偏右）
// ----------------------------------------------------------------------------
static void Gray_LED_Test(void)
{
    while (1) {
        int16_t pos = F_GW_position();

        if (pos == -1)      { LED_RED = 0; LED_GREEN = 1; }
        else if (pos == -2) { LED_RED = 1; LED_GREEN = 0; }
        else if (pos < 35)  { LED_RED = 0; LED_GREEN = 0; }
        else                { LED_RED = 1; LED_GREEN = 1; }

        delay_ms(50);
    }
}

// ----------------------------------------------------------------------------
// 转弯测试：原地旋 90°，打印转弯前后脉冲差（标定 ENC_PER_DEG）
// ----------------------------------------------------------------------------
static void Trace_Turn_Test(void)
{
    int32_t before, after;
    before = Spin_Encoder();
    printf("\r\n==== Turn Test: Spin_Angle(90) ====\r\n");
    printf("enc before = %ld\r\n", (long)before);
    delay_ms(500);
    Spin_Angle(90);
    after = Spin_Encoder();
    printf("enc after = %ld, delta = %ld\r\n", (long)after, (long)(after - before));
    printf("=> ENC_PER_DEG = delta / 90 = %ld\r\n", (long)((after - before) / 90));
}

 * ============================================================================ */

// ----------------------------------------------------------------------------
// KEY1 短按：前进 → 停 → 后退 → 停（每个阶段都【循环重发】速度命令）
//   ⚠ 关键：驱动板的速度命令必须周期性下发（循迹时也是每 10ms 一次）。
//   只在本段开头发一次的话，车只会动一下 —— 之前「看起来两次都前进」就是这个原因。
//   无串口硬件：用 Cortex-Debug 在 4 个阶段各打一个断点即可确认进到了哪段；
//   电机实际下发值看全局变量 MOTOR_Data[4]（Watch 窗口）：
//     前进应为 {-30,+30,-30,+30}，后退应为 {+30,-30,+30,-30}
// ----------------------------------------------------------------------------
static void Drive_Dir_Test(void)
{
    uint16_t i;

    // 1) 前进 2s
    for (i = 0; i < 200; i++) {
        Diff_Drive(30, 30);
        delay_ms(10);
    }

    // 2) 停 1s
    for (i = 0; i < 100; i++) {
        Stop();
        delay_ms(10);
    }

    // 3) 后退 2s
    for (i = 0; i < 200; i++) {
        Diff_Drive(-30, -30);
        delay_ms(10);
    }

    // 4) 停 1s
    for (i = 0; i < 100; i++) {
        Stop();
        delay_ms(10);
    }
}

// ----------------------------------------------------------------------------
// 灰度调试（复用旧版 Gray_Debug 风格）：循环读取前/后灰度
// 打印 raw + 8 探头位 + 加权位置；无串口时用 Cortex-Debug 断点观察 raw/e
// ----------------------------------------------------------------------------
static void Gray_Debug(void)
{
    volatile uint8_t raw_f, raw_b; // volatile：防止调试时被优化掉(<optimized out>)
    volatile int16_t e_f, e_b;
    uint8_t i;

    printf("\r\n==== Gray Debug (KEY1) ====\r\n");

    while (1) {
        raw_f = F_GW_digital_Read();  // 前灰度（取反后：黑=1、白=0）
        raw_b = B_GW_digital_Read();  // 后灰度（取反后：黑=1、白=0）
        e_f   = F_GW_digital_eight(); // 前灰度加权位置
        e_b   = B_GW_digital_eight(); // 后灰度加权位置

        printf("F_raw=0x%02X bits=", (unsigned)raw_f);
        for (i = 0; i < 8; i++) printf("%d", (raw_f >> (7 - i)) & 0x01);
        printf(" eight=%d | B_raw=0x%02X bits=", e_f, (unsigned)raw_b);
        for (i = 0; i < 8; i++) printf("%d", (raw_b >> (7 - i)) & 0x01);
        printf(" eight=%d\r\n", e_b);

        delay_ms(200);
    }
}

// ----------------------------------------------------------------------------
// 编码器极性测试（临时）：验证驱动板 0x15 编码器极性能否实现反转
// 关键：驱动板负速度不反转（负值被当正速度），测「极性反转 + 正速度」是否后退
// ⚠ 极性反转可能导致闭环正反馈失控，请把车架起来测
// ----------------------------------------------------------------------------
static void Polarity_Test(void)
{
    int8_t pol;

    // 1) 极性=1（当前默认），正速度 → 期望前进
    pol = 1;
    I2C_Write_Len(MOTOR_ENCODER_POLARITY_ADDR, &pol, 1);
    Diff_Drive(20, 20);
    delay_ms(1500);
    Stop();
    delay_ms(800);

    // 2) 极性=0（反转），正速度 → 期望后退（关键验证）
    pol = 0;
    I2C_Write_Len(MOTOR_ENCODER_POLARITY_ADDR, &pol, 1);
    Diff_Drive(20, 20);
    delay_ms(1500);
    Stop();
    delay_ms(800);

    // 恢复极性=1
    pol = 1;
    I2C_Write_Len(MOTOR_ENCODER_POLARITY_ADDR, &pol, 1);
}

// ----------------------------------------------------------------------------
// 固定 PWM 反转测试（临时）：验证驱动板 0x1F 开环 PWM 负值能否反转
// 开环 PWM 直接设占空比，不经编码器闭环；负值若表示方向则可反转
// ----------------------------------------------------------------------------
static void PWM_Test(void)
{
    int8_t pwm[4];
    // 1) 正 PWM：4 电机正转
    pwm[0] = 50;
    pwm[1] = 50;
    pwm[2] = 50;
    pwm[3] = 50;
    I2C_Write_Len(MOTOR_FIXED_PWM_ADDR, pwm, 4);
    delay_ms(1500);
    // 停
    pwm[0] = 0;
    pwm[1] = 0;
    pwm[2] = 0;
    pwm[3] = 0;
    I2C_Write_Len(MOTOR_FIXED_PWM_ADDR, pwm, 4);
    delay_ms(800);
    // 2) 负 PWM：看电机是否反转（关键）
    pwm[0] = -50;
    pwm[1] = -50;
    pwm[2] = -50;
    pwm[3] = -50;
    I2C_Write_Len(MOTOR_FIXED_PWM_ADDR, pwm, 4);
    delay_ms(1500);
    // 停
    pwm[0] = 0;
    pwm[1] = 0;
    pwm[2] = 0;
    pwm[3] = 0;
    I2C_Write_Len(MOTOR_FIXED_PWM_ADDR, pwm, 4);
}

// ----------------------------------------------------------------------------
// 第一题：出发区 → 圆心 → 左转90° → A点（路径状态机见 APP/nav/nav.c）
//   上电不动；KEY0 = 启动（单次）；KEY1 = 急停并保持停车（需断电复位）
//   Watch：DBG_NAV_PHASE（0待机 1去圆心 2转弯 3去A 4完成 5异常）
//          DBG_NAV_CENTER_REASON / DBG_NAV_A_REASON / DBG_NAV_LINES
//          DBG_NAV_GAP / DBG_NAV_PREVGAP / DBG_NAV_SEG_TICKS / DBG_NAV_TURN_POS
// ----------------------------------------------------------------------------
static uint8_t Enc_Key_Debounce(uint8_t raw, uint8_t *cnt, uint8_t *stable); // 定义在后面

static void Nav_Task2_Test(void)
{
    uint8_t k0_cnt = 0, k0_stable = 0;
    uint8_t k1_cnt = 0, k1_stable = 0;
    uint8_t state  = 0; // 0=待机 1=运行 2=急停

    Nav_Init();
    Stop();

    while (1) {
        if (state == 0) {
            if (Enc_Key_Debounce(KEY0_DOWN(), &k0_cnt, &k0_stable)) {
                Nav_Init();        // 重置全部状态
                Nav_Task2_Start(); // 启动第一题
                state = 1;
            }
            Stop(); // 待机保持零速
        } else if (state == 1) {
            if (Enc_Key_Debounce(KEY1_DOWN(), &k1_cnt, &k1_stable)) {
                Stop();
                state = 2; // 急停：保持停车，需断电复位
            } else {
                Nav_Task2_Run();
            }
        } else {
            Stop();
        }

        delay_ms(10);
    }
}

/* ============================================================================
 * 编码器标定（无串口，只用 Cortex-Debug Watch 读数）
 * ----------------------------------------------------------------------------
 * 流程：
 *   1) 直线距离：车摆正 → 按 KEY0 立刻前进 → 走到目标距离前按 KEY1 停车
 *      → Watch 读 ENC_CAL_SUM（四轮脉冲和）、ENC_CAL_MS（耗时）
 *      → ENC_PULSES_PER_MM = |ENC_CAL_SUM| / 实测距离(mm)
 *   2) 原地转角：车放原地 → 按 KEY1 切到旋转模式（看 ENC_CAL_MODE_NOW=1）
 *      → 按 KEY0 开始原地左转 → 目测约 90° 时按 KEY1 停车
 *      → Watch 读 ENC_CAL_LR（左轮-右轮脉冲差）
 *      → ENC_PER_DEG = |ENC_CAL_LR| / 实测角度
 *   3) 按 KEY0 可随时重新置零再跑一次，不必复位/重烧
 *
 * Watch 变量：
 *   ENC_CAL_D[0..3]  四个电机各自变化量（顺序 203右后/202右前/204左后/201左前）
 *   ENC_CAL_LEFT / ENC_CAL_RIGHT / ENC_CAL_SUM(四轮和) / ENC_CAL_LR(左-右)
 *   ENC_CAL_MS       从 KEY0 到 KEY1 的毫秒数
 *   ENC_CAL_PHASE    0=等待启动 1=运行中 2=已锁存 3=超时自动停
 *   ENC_CAL_RUN      1=正在跑；ENC_CAL_DONE 1=已锁存
 *   ENC_CAL_MODE_NOW 当前模式（0=直线 1=原地旋转）
 *   DBG_ENC_RAW[4]   编码器实时原始值
 *   DBG_KEY0/DBG_KEY1        按键实时状态（1=按下）—— 按了没反应先看这个
 *   DBG_KEY0_HITS / DBG_KEY1_HITS  按键被识别到的次数（按一下应该 +1）
 *   DBG_ENC_FAIL     编码器 I2C 读失败次数（非 0 = 总线有干扰，数据不可信）
 *   DBG_MOTOR_NACK   速度写入 I2C 失败次数（非 0 = 停止命令可能没写进去）
 *   DBG_FW_TAG       固件标记，应为 20261025（不是这个值 = 烧的还是旧固件）
 * ============================================================================ */
#define ENC_CALIB_MODE  0      // 上电默认模式：0=直线距离  1=原地转角
#define ENC_CAL_SPEED   12     // 直线前进速度（与 LINE_SPEED 一致）
#define ENC_CAL_SPIN_V  30     // 原地旋转速度（与 NAV_TURN_SPIN_V 一致）
#define ENC_CAL_TICK_MAX   460   // 【直线标定】KEY0 后跑约 5 秒自动停车（按拍计数，不依赖 DWT）
#define ENC_CAL_START_WAIT 1000  // 【固件版本指示】按 KEY0 后先原地等 1 秒再起步
#define ENC_CAL_STOP_TRIES 30    // 停车帧最多重发次数（驱动板收不到就会保持上次速度）

volatile int32_t  ENC_CAL_START[4] = {0};
volatile int32_t  ENC_CAL_END[4]   = {0};
volatile int32_t  ENC_CAL_D[4]     = {0};
volatile int32_t  ENC_CAL_LEFT     = 0;
volatile int32_t  ENC_CAL_RIGHT    = 0;
volatile int32_t  ENC_CAL_SUM      = 0;
volatile int32_t  ENC_CAL_LR       = 0;
volatile uint32_t ENC_CAL_MS       = 0;
volatile uint8_t  ENC_CAL_RUN      = 0;
volatile uint8_t  ENC_CAL_DONE     = 0;
volatile uint8_t  ENC_CAL_PHASE    = 0; // 0=等待 1=运行 2=已锁存 3=超时
volatile uint8_t  ENC_CAL_MODE_NOW = ENC_CALIB_MODE;
volatile int32_t  DBG_ENC_RAW[4]   = {0};

volatile uint8_t  DBG_KEY0       = 0; // 按键实时状态（1=按下）
volatile uint8_t  DBG_KEY1       = 0;
volatile uint16_t DBG_KEY0_HITS  = 0; // KEY0 识别到的按下次数
volatile uint16_t DBG_KEY1_HITS  = 0; // KEY1 识别到的按下次数
volatile uint16_t DBG_ENC_FAIL   = 0; // 编码器读取失败次数
volatile uint8_t  DBG_STOP_OK    = 0; // 上一次停车帧是否被驱动板确认（1=确认）
volatile uint8_t  DBG_START_OK   = 0; // 起步前基准值是否读到（0=读失败，本次数据不可信）
volatile uint8_t  DBG_END_OK     = 0; // 停车后终点值是否读到（0=读失败，本次数据不可信）

/* 空闲编码器探针：车不动时每 ~260ms 读一次编码器，滚动存最近 8 次。
   用途：不用开车、不用按键，手转车轮就能验证「编码器到底读不读得到、计不计数」。
     DBG_PROBE_TRY / DBG_PROBE_OK  → 探针总次数 / 成功次数，两者差很多就是 I2C 读不通
     DBG_PROBE[0..7][0..3]         → 最近 8 次原始值，手转车轮时这几个数应该跟着变 */
#define DBG_PROBE_N 8
volatile int32_t  DBG_PROBE[DBG_PROBE_N][4] = {{0}};
volatile uint8_t  DBG_PROBE_IDX = 0; // 下一个写入槽位
volatile uint16_t DBG_PROBE_TRY = 0; // 探针总次数
volatile uint16_t DBG_PROBE_OK  = 0; // 探针成功次数
volatile uint16_t DBG_BAT_MV    = 0; // 对照量：电池电压寄存器 0x00 的读数（mV）
                                     // ≈7000~12000 = 读通路正常；=0 = 读通路有问题
volatile uint16_t ENC_CAL_WRITE_FAIL = 0; // 运行中速度写失败累计（非 0 = 帧没进驱动板）
volatile uint32_t DBG_FW_TAG     = 20261025u; // 固件标记（确认烧的是新固件）
volatile uint32_t DBG_LOOP_CNT   = 0;         // 主循环心跳（不涨 = CPU 卡死）
volatile uint16_t DBG_TICK       = 0;         // 运行拍数（涨到 460 就该停车）
volatile uint32_t DBG_HARDFAULT  = 0;         // 进过硬故障则 = 0xDEADBEEF
// 注意：HardFault_Handler 定义在 User/stm32f4xx_it.c（不要在本文件重复定义，
// 否则链接报 L6200E 重复符号）。那边会在死循环前写入本变量。

static uint32_t enc_cal_t0 = 0; // DWT 周期计数起点

// DWT 周期计数器：只做耗时统计，不占用 SysTick（SysTick 被 delay_ms 当一次性定时器用）
static void Enc_Cal_Dwt_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Enc_Cal_Apply(int32_t *now)
{
    uint8_t i;
    for (i = 0; i < 4; i++) {
        DBG_ENC_RAW[i] = now[i];
        /* 逐电机差分：累计量现在是 32 位（与官方 int32_t EncodeTotal 一致），
           不再需要 int16 环绕校正，直接相减即可。 */
        ENC_CAL_D[i] = now[i] - ENC_CAL_START[i];
    }
    ENC_CAL_RIGHT = ENC_CAL_D[0] + ENC_CAL_D[1]; // 203 右后 + 202 右前
    ENC_CAL_LEFT  = ENC_CAL_D[2] + ENC_CAL_D[3]; // 204 左后 + 201 左前
    ENC_CAL_SUM   = ENC_CAL_LEFT + ENC_CAL_RIGHT;
    ENC_CAL_LR    = ENC_CAL_LEFT - ENC_CAL_RIGHT;
}

// 读编码器（带重试）：失败时做总线恢复再来一次
static uint8_t Enc_Cal_Read_Retry(int32_t *now, uint8_t tries)
{
    uint8_t i;

    for (i = 0; i < tries; i++) {
        if (Encoder_ReadRaw(now) == 0) return 1;
        DBG_ENC_FAIL++;
        IIC_Bus_Recover(); // 从机把 SDA 拉死时先把总线顶出来
        delay_ms(5);
    }
    return 0;
}

// 停车并锁存：先停、等电机真正停稳（减速滑行的位移也要算进本次距离），再读最终值
static void Enc_Cal_Stop_Latch(uint32_t elapsed_ms)
{
    int32_t now[4];
    uint8_t i;

    /* 驱动板收不到帧就会一直保持上一次速度（「命令停车但车不停」）。
       现在 ACK 不再作为失败依据（板子本来就不应答），Stop_Retry 会一次成功，
       所以这里改成**显式连发 5 帧零速**，间隔 5ms，防止单帧被丢掉。 */
    for (i = 0; i < 5; i++) {
        Stop();
        delay_ms(5);
    }
    DBG_STOP_OK = 1;
    delay_ms(200); // 等减速/惯性走完

    /* 读终点值并【算出差值】。之前漏了 Enc_Cal_Apply 这一句，
       终点读到了却没做差，ENC_CAL_SUM 恒为 0（表现为「停车正常但读不到编码器」）。 */
    if (Enc_Cal_Read_Retry(now, 5)) {
        for (i = 0; i < 4; i++) ENC_CAL_END[i] = now[i];
        Enc_Cal_Apply(now);
        DBG_END_OK = 1;
    } else {
        DBG_END_OK = 0; // 5 次全失败：看 DBG_ENC_FAIL
    }

    ENC_CAL_MS    = elapsed_ms;
    ENC_CAL_DONE  = 1;
    ENC_CAL_RUN   = 0;
    ENC_CAL_PHASE = 2;
}

// 两拍确认消抖：连续 2 拍读到「按下」才认一次（约 26ms），短按不会漏
// 返回 1 = 本次确认到一次按下事件
static uint8_t Enc_Key_Debounce(uint8_t raw, uint8_t *cnt, uint8_t *stable)
{
    if (raw) {
        if (*cnt < 2) (*cnt)++;
    } else {
        *cnt = 0;
    }

    if (*cnt >= 2 && !*stable) {
        *stable = 1;
        return 1;
    }
    if (*cnt == 0) *stable = 0; // 松开：重新武装，可以再触发
    return 0;
}

static void Encoder_Calib_Test(void)
{
    int32_t  now[4] = {0};
    uint8_t  k0_cnt = 0, k0_stable = 0;
    uint8_t  k1_cnt = 0, k1_stable = 0;
    uint8_t  i;
    uint16_t tick = 0;       // 计时用拍数（DWT 不可靠时兜底）
    uint16_t probe_tick = 0; // 空闲探针分频计数
    uint16_t idle_tick = 0;  // 空闲补发停车帧分频计数
    uint32_t ms;

    Enc_Cal_Dwt_Init();
    Stop();
    ENC_CAL_PHASE = 0;

    while (1) {
        /* ---------------- 1. 按键（两拍确认消抖，按下沿触发） ---------------- */
        DBG_KEY0 = KEY0_DOWN();
        DBG_KEY1 = KEY1_DOWN();

        if (Enc_Key_Debounce(DBG_KEY0, &k0_cnt, &k0_stable)) { // KEY0：置零并开始
            DBG_KEY0_HITS++;
            Stop_Retry(5); // 先确认停住，再取基准

            /* 【固件版本指示】按下 KEY0 后先原地等 1 秒再起步：
               车立刻就走 = 烧的还是旧固件（旧版 KEY0 就是一直向前循迹）。 */
            delay_ms(ENC_CAL_START_WAIT);
            Stop_Retry(5);

            /* 基准值必须真读到，否则算出来的差值毫无意义。
               读失败时把 DBG_START_OK 置 0，本次数据直接作废重测。 */
            if (Enc_Cal_Read_Retry(now, 5)) {
                for (i = 0; i < 4; i++) {
                    ENC_CAL_START[i] = now[i];
                    DBG_ENC_RAW[i]   = now[i];
                    ENC_CAL_D[i]     = 0;
                }
                DBG_START_OK = 1;
            } else {
                DBG_START_OK = 0;
            }
            ENC_CAL_LEFT = ENC_CAL_RIGHT = ENC_CAL_SUM = ENC_CAL_LR = 0;
            ENC_CAL_MS    = 0;
            ENC_CAL_DONE  = 0;
            ENC_CAL_RUN   = 1;
            ENC_CAL_PHASE = 1;
            enc_cal_t0    = DWT->CYCCNT;
            tick          = 0;
        }

        if (Enc_Key_Debounce(DBG_KEY1, &k1_cnt, &k1_stable)) { // KEY1：停车锁存 / 切模式
            DBG_KEY1_HITS++;
            if (ENC_CAL_RUN) { // 手动提前停车（KEY1 可用时）
                Enc_Cal_Stop_Latch((DWT->CYCCNT - enc_cal_t0) / (SystemCoreClock / 1000u));
            } else {
                ENC_CAL_MODE_NOW = (uint8_t)(ENC_CAL_MODE_NOW ? 0 : 1);
                ENC_CAL_PHASE    = 0;
            }
        }

        /* ---------------- 2. 运行控制 ---------------- */
        if (ENC_CAL_RUN) {
            tick++; // 拍数计时：不依赖 DWT（DWT 不走时 ms 恒为 0，5 秒判据永远不成立）
            DBG_TICK = tick;
            ms = (DWT->CYCCNT - enc_cal_t0) / (SystemCoreClock / 1000u);
            ENC_CAL_MS = ms;          // 仅供参考：跑完还是 0 说明 DWT 没在计数

            if (tick >= ENC_CAL_TICK_MAX) { // 跑满约定拍数：自动停车并锁存
                Enc_Cal_Stop_Latch(ms);
            } else {
                /* 运行中【只写速度、不读编码器】：
                   读编码器是 16 字节软件 I2C 传输，会把电机板的速度写入打断
                   （10-24 左偏的同一个根因）。标定只需要起点和终点的值，
                   中途读没有任何必要，反而让停车命令写不进去 → 车不停。 */
                if (ENC_CAL_MODE_NOW) {
                    Spin(-ENC_CAL_SPIN_V); // 原地左转（与 Nav_SpinLeft90 同向）
                } else {
                    Diff_Drive(ENC_CAL_SPEED, ENC_CAL_SPEED); // 直线前进
                }
                // 速度写失败次数实时可见：非 0 就说明这一帧没进驱动板
                ENC_CAL_WRITE_FAIL = DBG_MOTOR_NACK;
            }
        } else {
            /* 空闲停车帧改成每 ~2 秒补发一次（原来是每 10ms 一帧）。
               原因：每帧失败还会再重试 5 次，等于每秒往软件 I2C 上砸 500 次
               写事务，把总线占满，反而更容易把从机搞失步。
               板子丢帧会保持上次速度，2 秒补一帧足够兜底。 */
            if (++idle_tick >= 200) {
                idle_tick = 0;
                Stop();
            }

            /* ---------------- 空闲探针：验证编码器读取通路 ----------------
               车静止时每 ~260ms 读一次编码器并滚动保存最近 8 次。
               手转车轮 → DBG_PROBE 里的数跟着变 = 读取和计数都正常；
               数值不变 = 读得到但编码器不计数（类型/极性配置）；
               TRY 涨、OK 不涨 = I2C 读本身失败。 */
            if (++probe_tick >= 20) {
                probe_tick = 0;
                DBG_PROBE_TRY++;

                /* 对照读：电池电压寄存器 0x00（2 字节，单位 mV）。
                   官方例程就是读这个。它是**已知非零**的量，用来区分两种情况：
                     DBG_BAT_MV ≈ 7000~12000 → 读通路是好的，那 0x3C 全 0 就是编码器侧问题
                     DBG_BAT_MV = 0            → 读通路本身坏了（或板子根本没返回数据） */
                {
                    uint8_t vb[2] = {0, 0};
                    (void)I2C_Read_Len(ADC_BAT_ADDR, vb, 2);
                    DBG_BAT_MV = (uint16_t)(vb[0] | (vb[1] << 8));
                }

                if (Encoder_ReadRaw(now) == 0) {
                    for (i = 0; i < 4; i++) DBG_PROBE[DBG_PROBE_IDX][i] = now[i];
                    DBG_PROBE_IDX = (uint8_t)((DBG_PROBE_IDX + 1) % DBG_PROBE_N);
                    if (DBG_PROBE_OK < 60000) DBG_PROBE_OK++;
                }
            }
        }

        DBG_LOOP_CNT++; // 心跳：车还在动但这个值不涨 = CPU 卡死/进硬故障
        delay_ms(10);
    }
}

/* ============================================================================
 * 循迹测试入口（最简单的一条链路：前灰度 → Line_Follow → 差速 → 驱动板）
 * ----------------------------------------------------------------------------
 * 用这个入口的目的：先把「差速能不能立起来」这件事单独验证掉。
 *   完全不碰编码器（不读、不算里程）
 *   完全不碰 nav.c（不数线、不判坐标）
 * 操作：
 *   KEY0 按下 → 开始向前循迹
 *   KEY1 按下 → 立即停车，回到待机
 * 上电不会动，必须按键才启动。
 *
 * Watch 观察：
 *   DBG_pos / DBG_e / DBG_u / DBG_vl / DBG_vr  循迹内部量
 *   DBG_raw / DBG_hist                         灰度原始字节（看探头是否在变）
 *   DBG_MOTOR_NACK                             速度帧未被确认的次数
 * 判读：DBG_vl 与 DBG_vr 不相等 → 差速已经下发；
 *       车却直冲不拐 → 差速没被驱动板执行（极性/类型/驱动板），不是循迹算法问题。
 * ============================================================================ */
static void Trace_Follow_Test(void)
{
    uint8_t mode    = 0; // 0=待机 1=向前循迹
    uint8_t k0_cnt = 0, k0_stable = 0;
    uint8_t k1_cnt = 0, k1_stable = 0;

    Stop();

    while (1) {
        if (Enc_Key_Debounce(KEY0_DOWN(), &k0_cnt, &k0_stable)) {
            DBG_KEY0_HITS++;
            mode = 1;
        }
        if (Enc_Key_Debounce(KEY1_DOWN(), &k1_cnt, &k1_stable)) {
            DBG_KEY1_HITS++;
            mode = 0;
            Stop();
        }

        if (mode) {
            Trace_Forward();
        } else {
            Stop();
        }

        DBG_LOOP_CNT++;
        delay_ms(10);
    }
}

int main()
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_Init(168);
    LED_Init();

    /* 本车没有 LED 硬件：不做 LED 自检（原来的 2 秒等待会让开机期间的按键全被吃掉，
       表现成「第一次按 KEY0 没反应」）。LED_Init/LED_EX_Init 保留，引脚初始化无害。 */
    LED_EX_Init();
    /* 本车没有串口硬件：不做 USART1 初始化（原初始化会开 RXNE 中断，
       PA10 悬空时被噪声持续触发，ISR 里还有无超时死等 → 主循环被饿死）。 */
    // USART1_Init(19200);
    MOTOR_Init();
    GREY_Init();
    START_Init();
    KEY1_Init();
    Steering_Init();
    Steering_SET();  // 上电即张开并保持（否则舵机无脉冲会松掉，可能在待机时自己闭合）
    TCS3200_Init();  // 颜色模块（TIM3 计数 + PC0~PC3）；USE_FIXED_WHITE_REF=1，不用现场白平衡
    Nav_Init();
    delay_ms(200);

    Stop();

    /* ==================== 模式入口：同时只放开一行 ==================== */
    // 当前：第一题（KEY0 启动 → 出发区→圆心→左转90°→A点；KEY1 急停）
    Nav_Task2_Test();

    // Trace_Follow_Test(); // 对照：纯向前循迹（KEY0 向前循迹 / KEY1 停车），不碰 nav

    // Encoder_Calib_Test(); // 编码器测试/标定（上电静止自动探针 → KEY0 前进 5 秒 → 自动锁存）

    // 手动循迹（正常模式）：KEY0 向前循迹、KEY1 向后循迹
    // while (1) {
    //     if (START_FLAG == 0) {         // KEY0（PE4）：向前循迹
    //         delay_ms(20);
    //         while (1) { Trace_Forward(); delay_ms(10); }
    //     }
    //     if (KEY1_FLAG == 0) {          // KEY1（PE3）：向后循迹
    //         delay_ms(20);
    //         while (1) { Trace_Backward(); delay_ms(10); }
    //     }
    // }
}
