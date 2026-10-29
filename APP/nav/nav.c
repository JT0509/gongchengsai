#include "nav.h"

// ============================================================================
// 导航层实现 —— 第一题：出发区 → 圆心 → 左转 90° → A 点
// ----------------------------------------------------------------------------
// 本版按「完整执行方案-逐步.md」第十节、开发记录 16.1 的定稿重写：
//   • 侧灰度边沿状态机：黑连续 K 次确认记 1 条，白连续 M 次才重新武装
//   • 圆心 = 侧灰度跨过 0° 辐射线（第 5 条线）+ 间隔签名（50mm 紧跟 336mm）
//   • A 点 = 转弯后跨过内圆线（跳过脚下/315° 线，用"长间隔"识别）
//   • 运动用 trace.c 的 Trace_Forward()：前灰度 PD + 丢线回找，实车已验证
//   • 完全不用编码器；不用前/后灰度判圆心/A
// ============================================================================

NavState Nav;

// ==================== 无串口调试变量（Cortex-Debug Watch） ====================
volatile int16_t  DBG_NAV_POS          = 0;
volatile uint8_t  DBG_NAV_SGW          = 0;
volatile uint8_t  DBG_NAV_STATE        = 0;
volatile uint16_t DBG_NAV_LINES        = 0;
volatile uint16_t DBG_NAV_GAP          = 0;
volatile uint16_t DBG_NAV_PREVGAP      = 0;
volatile uint16_t DBG_NAV_SEG_TICKS    = 0;
volatile uint16_t DBG_NAV_CAP          = 0;
volatile uint8_t  DBG_NAV_CENTER_REASON = 0;
volatile uint8_t  DBG_NAV_A_REASON      = 0;
volatile uint8_t  DBG_NAV_TURN_DONE     = 0;
volatile int16_t  DBG_NAV_TURN_POS      = 0;
volatile uint8_t  DBG_NAV_SERVO         = 0; // 夹爪状态：0=张开 1=夹住
volatile uint8_t  DBG_NAV_COLOR         = 0;
volatile uint8_t  DBG_NAV_BACK_REASON   = 0;
volatile uint16_t DBG_NAV_BACK_SIG_GAP   = 0;
// 转向留档：每次 Nav_Turn_To 记一次，用来区分「指令角度就不对」和「执行不到位」
volatile uint8_t  DBG_NAV_HEAD_BEFORE   = 0; // 转向前的 sg_heading
volatile uint8_t  DBG_NAV_TURN_TGT      = 0; // 本次目标角
volatile int16_t  DBG_NAV_TURN_DEG      = 0; // 本次实际下发的带符号转角
volatile uint16_t DBG_NAV_TURN_MS       = 0; // 本次旋转延时 ms
// 靶位对位微调（ms @速度12，正=前进 负=后退）：Watch 里可直接改，按 KEY0 重新开局即生效，
// 不用重编。每次开局 Nav_Task2_Start 用它覆盖 sg_place_fine_ms。
volatile int16_t  DBG_NAV_FINE_MS       = NAV_TARGET_FINE_MS;
// 实际用在靶位对位上的微调值（每投一次更新一次）：若它≠你设的值，说明设置没生效
volatile int16_t  DBG_NAV_FINE_APPLIED  = 0;
// 放料后退后时长（ms @NAV_INCH_V）：Watch 里直接改，下一次放料就用新值
volatile int16_t  DBG_NAV_BACK_MS       = NAV_BACKOFF_MS;
// 靶位到点反向刹车脉宽（ms @NAV_BRAKE_V）：Watch 里直接改，下一次到靶位就用新值
volatile int16_t  DBG_NAV_BRAKE_MS      = NAV_BRAKE_MS;

// 整局所有转向的流水（单变量只看得到最后一次，整局会被覆盖）
//   d = TGT − HEAD（按最短路径归一化）；MS 是下发的旋转延时
volatile uint8_t  NAV_TURN_LOG_NUM = 0;
volatile uint8_t  NAV_TURN_LOG_HEAD[NAV_TURN_LOG_N] = {0};
volatile uint8_t  NAV_TURN_LOG_TGT[NAV_TURN_LOG_N]  = {0};
volatile uint16_t NAV_TURN_LOG_MS[NAV_TURN_LOG_N]   = {0};
volatile uint8_t  DBG_NAV_PLACE_REASON  = 0;

// 回家段数线黑匣子（2026-10-29）：每条确认线的间隔，查「到底数了哪几条」
volatile uint8_t  NAV_HOME_N = 0;
volatile uint16_t NAV_HOME_GAP[6] = {0};

// ==================== 黑匣子（无串口/免跟车调试） ====================
// 车跑完停住后插 ST-Link，用 attach（不复位）读 RAM 回放现场。
volatile uint8_t  NAV_LOG_I = 0;
volatile int16_t  NAV_LOG_POS[NAV_LOG_N]     = {0};
volatile uint16_t NAV_LOG_LINES[NAV_LOG_N]   = {0};
volatile uint16_t NAV_LOG_GAP[NAV_LOG_N]     = {0};
volatile uint16_t NAV_LOG_PREVGAP[NAV_LOG_N] = {0};
volatile uint16_t NAV_LOG_TICKS[NAV_LOG_N]   = {0};
volatile uint8_t  NAV_LOG_PHASE[NAV_LOG_N]   = {0};

volatile uint8_t  NAV_STOP_SNAPPED  = 0;
volatile int16_t  NAV_STOP_POS      = 0;
volatile uint16_t NAV_STOP_LINES    = 0;
volatile uint16_t NAV_STOP_GAP      = 0;
volatile uint16_t NAV_STOP_PREVGAP  = 0;
volatile uint8_t  NAV_STOP_PHASE    = 0;

// ============================================================================
// 侧灰度边沿状态机
//   跳变才计数：黑连续 K 拍确认一条 → 转 REARM → 白连续 M 拍才允许下一条。
//   这样一条 20mm 宽的线只会被记 1 次，也不会被线内毛刺重复触发。
// ============================================================================
typedef enum
{
    SG_WAIT_EDGE = 0, // 等白→黑上升沿
    SG_DEBOUNCE  = 1, // 消抖：累计黑拍数
    SG_REARM     = 2  // 重武装：累计白拍数
} SgState;

static SgState  sg_state;
static uint8_t  sg_cnt;         // 消抖/重武装计数
static uint16_t sg_ticks;       // 距上一条确认线已过拍数（当前间隔）
static uint16_t sg_gap;         // 最近一次确认的间隔（拍）
static uint16_t sg_prev_gap;    // 上一次确认的间隔（拍）
static uint16_t sg_line_count;  // 本段累计确认线数

// ============================================================================
// 路径状态机
// ============================================================================
typedef enum
{
    PATH_IDLE = 0,
    PATH_TO_CENTER = 1, // 沿 270° 线向北到圆心
    PATH_TURN_NAV  = 2, // 通用转向：转到 sg_pick_angle（45° 格点，最短方向）
    PATH_TO_POINT  = 3, // 通用去内圆点：A/C/E 取料、B/D 暂存或取回
    PATH_DONE      = 4, // 正常完成
    PATH_ABORT     = 5, // 超限异常停车
    PATH_CLAMP_READ    = 6,  // 内圆点：夹住 → 读色 → 记账 → 定下一步
    PATH_BACK_TO_CENTER= 7,  // 内圆 → 圆心（倒退）
    PATH_TURN_TO_TARGET= 8,  // 圆心转到目标 φ
    PATH_TO_TARGET     = 9,  // 沿 φ 射线前进到大圆（靶位）
    PATH_PLACE         = 10, // 对位微调 → 张开放料 → 退后收尾
    PATH_DECIDE        = 11, // 纯决策：手上有料判投/存，没料选下一块
    PATH_BACK_FROM_TARGET = 12, // 大圆 → 圆心（倒退 780mm）
    PATH_TURN_TO_HOME  = 13, // 圆心转向 270°
    PATH_TO_HOME       = 14  // 沿 270° 出去，数第 4 条线停在大圆
} NavPathPhase;

static NavPathPhase nav_phase = PATH_IDLE;

// ============================================================================
// 自适应拍数上限
//   固定拍数上限不可靠：实车实测车速只有 ≈62mm/s（电池强弱还会变），
//   拍距 ≈0.72mm/拍，圆心 780mm 需要 ≈1040 拍，固定 800 必在 570mm 截停。
//   做法：每数到一条「已知里程」的线，就用这条线的实测间隔换算剩余拍数。
// ============================================================================
static uint32_t sg_cap_ticks = NAV_CENTER_MAX_TICKS; // 本段拍数上限
static uint16_t sg_max_gap   = 0;                    // 本段最长间隔（≈336mm 那一段，用来估拍距）
static uint16_t sg_a_min_gap = NAV_A_LINE_MIN_GAP;   // 去 A 段"长间隔"门限（按实拍距换算）

// ----------------------------------------------------------------------------
// 取料后送靶位段的状态
//   sg_target_angle 目标靶位角（0/45/90/135/180），0xFF=判色失败
//   sg_tgt_mode     0=数任意线（绿靶：沿途只有大圆一条）
//                   1=只数长间隔线（其它靶：斜线短、内圆/大圆长）
//   sg_tgt_need     需要数到第几条（绿=1，其它=2）
//   sg_tgt_long     已数到的条数
//   sg_tgt_last     上一拍的 sg_line_count，用来识别"又确认了一条新线"
// ----------------------------------------------------------------------------
static uint8_t  sg_target_angle = 0xFFu;
static uint8_t  sg_tgt_mode     = 0u;
static uint8_t  sg_tgt_need     = 1u;
static uint8_t  sg_tgt_long     = 0u;
static uint16_t sg_tgt_last     = 0u;
static uint16_t sg_tgt_min_gap  = NAV_TARGET_MIN_GAP; // 长间隔线门槛（靶位/回家各一套）

// 前灰度判靶位状态机：arm(先看到线) → 数"靶位特征"脉冲
static uint8_t  sg_fr_armed  = 0u;  // 1=已武装，可以开始数特征
static uint8_t  sg_fr_clear  = 0u;  // 连续看到线（pos>=0）的拍数
static uint8_t  sg_fr_cnt    = 0u;  // 特征连续拍数（消抖）
static uint8_t  sg_target_color = 0u; // 本次夹到的颜色（黑料靶特征不同）

// 彩盘前缘提前触发（2026-10-29）
//   中间区（探头3~6）沿射线一直压着黑线 → "见过黑"计数；进彩盘后中间区全白 → 计数。
static uint8_t  sg_fr_mid_black = 0u; // 中间区见过黑的记忆窗（每拍递减）
static uint8_t  sg_fr_edge_cnt  = 0u; // 中间区连续全白拍数
static uint16_t sg_tgt_inner_ticks = 0u; // 非绿靶：内圆确认时的 seg_ticks（前缘距离门用）

// ----------------------------------------------------------------------------
// 三块料账本 + 车头朝向
//   sg_heading      车头朝向，只可能是 45° 整数倍；每次转向后在格点上重新锚定
//   sg_taken[0..2]  A(0°) / C(90°) / E(180°) 的料是否已取走
//   sg_stage[0..1]  B(45°) / D(135°) 暂存点放的是什么颜色（0=空）
//   sg_claw         夹爪里料的颜色（0=空）
//   sg_picked       已取几块（0..3）
//   sg_retrieve     1 = 这趟去 B/D 是【取回暂存块】，不是取料
//   sg_stage_slot   PLACE 时往哪个暂存点放：0=B 1=D 0xFF=投靶位
//   sg_pick_angle   本趟要去的内圆点角（0/45/90/135/180）
//   sg_place_fine_ms 放料微调（靶位用 NAV_TARGET_FINE_MS，暂存用 NAV_STORE_FINE_MS）
// ----------------------------------------------------------------------------
// 角度一律用 uint16_t：回家角 270° 装不进 uint8_t
static uint16_t sg_heading        = 90u;  // 出发朝北
static uint8_t  sg_taken[3]       = {0u, 0u, 0u};
static uint8_t  sg_stage[2]       = {0u, 0u};
static uint8_t  sg_claw           = 0u;
static uint8_t  sg_picked         = 0u;
static uint8_t  sg_retrieve       = 0u;
static uint8_t  sg_stage_slot     = 0xFFu;
static uint16_t sg_pick_angle     = 90u;
static int16_t  sg_place_fine_ms  = NAV_TARGET_FINE_MS;

// 线序号 → 累计里程（mm）：1=大圆(起步) 2=出发圆北缘 3=内圆 4=315°线 5=0°线(圆心)
static const uint16_t kCenterMm[6] = { 0u, 0u, 152u, 394u, 730u, 780u };

// ============================================================================
// 黑匣子：每拍记录 + 停车瞬间锁存
// ============================================================================
static void Nav_LogTick(void)
{
    uint8_t i = NAV_LOG_I;

    NAV_LOG_POS[i]     = DBG_NAV_POS;
    NAV_LOG_LINES[i]   = (uint16_t)sg_line_count;
    NAV_LOG_GAP[i]     = sg_gap;
    NAV_LOG_PREVGAP[i] = sg_prev_gap;
    NAV_LOG_TICKS[i]   = Nav.seg_ticks;
    NAV_LOG_PHASE[i]   = (uint8_t)nav_phase;

    NAV_LOG_I = (uint8_t)((i + 1u) % NAV_LOG_N);
}

// ============================================================================
// 相位迁移黑匣子（2026-10-27 加）
//   Nav_LogTick 只留最近 128 拍（≈1.3s），整局查不了；这里每次**进入新相位**
//   记一条，一局最多几十条，跑完 attach 就能看完整流程：
//   哪一步跳去了哪里、当时的朝向估计 sg_heading 是多少、目标角是多少。
//   典型用途：查「明明该去 A，为什么直接回家了」——如果是 PATH_TURN_TO_HOME，
//   说明 Nav_Plan_Pick 返回了 0（账本认为三块都取完了），账本错了。
//   纯记录，不参与任何控制。
// ============================================================================
volatile uint8_t NAV_TRC_NUM = 0;
volatile uint8_t NAV_TRC_PHASE[NAV_TRC_N] = {0};
volatile uint8_t NAV_TRC_HEAD[NAV_TRC_N]  = {0};
volatile uint8_t NAV_TRC_AIM[NAV_TRC_N]   = {0};
volatile uint8_t NAV_TRC_LED[NAV_TRC_N]   = {0};

static uint8_t sg_trc_last = 0xFFu;   // 上一次记录的相位（Nav_Task2_Start 复位）

static void Nav_TrcPhase(void)
{
    uint8_t p = (uint8_t)nav_phase;

    if (p == sg_trc_last) return;
    sg_trc_last = p;

    if (NAV_TRC_NUM < NAV_TRC_N)
    {
        uint8_t i = NAV_TRC_NUM;
        NAV_TRC_PHASE[i] = p;
        NAV_TRC_HEAD[i]  = (uint8_t)sg_heading;
        // 本相位的目标角：去点取料/去靶位用 pick_angle，转靶位用 target_angle
        NAV_TRC_AIM[i]   = (p == PATH_TURN_TO_TARGET)
                           ? (uint8_t)sg_target_angle
                           : (uint8_t)sg_pick_angle;
        NAV_TRC_LED[i]   = (uint8_t)((sg_picked & 0x03u)
                           | (sg_taken[0] ? 0x04u : 0u)
                           | (sg_taken[1] ? 0x08u : 0u)
                           | (sg_taken[2] ? 0x10u : 0u)
                           | ((sg_stage[0] != 0u) ? 0x20u : 0u)
                           | ((sg_stage[1] != 0u) ? 0x40u : 0u)
                           | ((sg_claw     != 0u) ? 0x80u : 0u));
        NAV_TRC_NUM++;
    }
}

// ★2026-10-29 靶位专用刹车：判到点后先反向给一小段速度，把滑行抵掉再停。
//   只用于「投料到靶位」这一处停车。圆心/内圆点/转向的停车距离都是开环标定的，
//   换刹车方式会改变这些点的实际停车位置，破坏已标好的里程门限。
static void Nav_Stop_Brake(void)
{
    int16_t bm = DBG_NAV_BRAKE_MS;
    if (bm < 0)    { bm = 0; }
    if (bm > 400)  { bm = 400; }
    Diff_Drive(-NAV_BRAKE_V, -NAV_BRAKE_V);
    delay_ms((uint32_t)bm);
    Stop_Retry(3);
}

static void Nav_StopSnap(void)
{
    NAV_STOP_SNAPPED = 1;
    NAV_STOP_POS     = DBG_NAV_POS;
    NAV_STOP_LINES   = (uint16_t)sg_line_count;
    NAV_STOP_GAP     = sg_gap;
    NAV_STOP_PREVGAP = sg_prev_gap;
    NAV_STOP_PHASE   = (uint8_t)nav_phase;

#if NAV_STOP_BREAK
    __breakpoint(0);
#endif
}

// ============================================================================
// 侧灰度计数器
//   count_foot = 1：起步时若已压在黑线上，这条线算第 1 条（出发区大圆线）
//   count_foot = 0：刚转完弯，脚下这条线（270° 辐射线）不算，等白了再武装
// ============================================================================
static void Nav_Counter_Init(uint8_t count_foot)
{
    uint8_t black_now = S_GW_Read();

    sg_cnt        = 0;
    sg_ticks      = 0;
    sg_gap        = 0;
    sg_prev_gap   = 0;
    sg_max_gap    = 0;
    sg_line_count = 0;

    if (black_now && count_foot)
    {
        // 起步就压在出发区大圆线上：它必须算第 1 条，否则整串计数少一条
        sg_line_count = 1;
        sg_state      = SG_REARM;
    }
    else if (black_now)
    {
        // 转弯后脚下是 270° 辐射线：不算，必须等白 M 拍后才认下一条
        sg_state = SG_REARM;
    }
    else
    {
        sg_state = SG_WAIT_EDGE;
    }

    Nav.line_count = (int32_t)sg_line_count;
    Nav.gap        = 0;
    Nav.prev_gap   = 0;
    Nav.seg_ticks  = 0;
}

// ============================================================================
// 数到一条线后重算本段拍数上限（用刚测到的间隔换剩余里程）
//   出发→圆心：第2条线剩628mm、第3条线剩386mm（×1.5）
//             第4条线（315°线）之后到 0°线的实际距离 = 侧灰度横向偏移 d，
//             实车测得 d≈100mm（不是方案假设的50mm），所以按 250mm 给余量
//   圆心→内圆点：上限在转向时用上一段最长间隔（315°线≈336mm）一次算好，见 PATH_TURN_NAV
// ============================================================================
static void Nav_Update_Cap(uint16_t line_no)
{
    uint32_t seg_mm, rest_mm, rest, ratio;

    if (sg_gap == 0u) return;

    if (nav_phase == PATH_TO_CENTER && line_no >= 2u && line_no <= 4u)
    {
        seg_mm  = (uint32_t)kCenterMm[line_no] - (uint32_t)kCenterMm[line_no - 1u];
        rest_mm = 780u - (uint32_t)kCenterMm[line_no];
        ratio   = 3u; // 1.5×
        if (line_no == 4u)
        {
            rest_mm = 250u; // 第4条线后实测还有≈100mm，按 250mm 留足余量
            ratio   = 2u;   // 不再额外乘
        }
        rest = (uint32_t)sg_gap * rest_mm / seg_mm;
        sg_cap_ticks = (uint32_t)Nav.seg_ticks + rest * ratio / 2u;
    }

    if (sg_cap_ticks > 60000u) sg_cap_ticks = 60000u; // 防溢出到 uint16 比较
    DBG_NAV_CAP = (uint16_t)sg_cap_ticks;
}

// 每控制周期调用（10ms/拍 + I2C 开销 ≈11.5ms/拍，实车 ≈0.72mm/拍）
void Nav_Update(void)
{
    uint8_t raw = S_GW_Read();
    DBG_NAV_SGW = raw;

    if (sg_ticks < 0xFFFFu) sg_ticks++;
    if (Nav.seg_ticks < 0xFFFFu) Nav.seg_ticks++;

    switch (sg_state)
    {
        case SG_WAIT_EDGE:
            if (raw)
            {
                sg_state = SG_DEBOUNCE;
                sg_cnt   = 1;
            }
            break;

        case SG_DEBOUNCE:
            if (raw)
            {
                sg_cnt++;
                if (sg_cnt >= SG_DEBOUNCE_K)
                {
                    // 确认一条线：记录间隔并重新武装
                    sg_line_count++;
                    sg_prev_gap = sg_gap;
                    sg_gap      = sg_ticks;
                    sg_ticks    = 0;
                    sg_cnt      = 0;
                    sg_state    = SG_REARM;

                    Nav.line_count = (int32_t)sg_line_count;
                    Nav.gap        = sg_gap;
                    Nav.prev_gap   = sg_prev_gap;

                    // 记录本段最长间隔（≈336mm 那一段，转弯后用来估 A 段拍距）
                    if (sg_gap > sg_max_gap) sg_max_gap = sg_gap;

                    // 用这条线的实测间隔重算本段拍数上限（电池掉电也不会提前截停）
                    Nav_Update_Cap(sg_line_count);
                }
            }
            else
            {
                // 毛刺：回到等沿
                sg_state = SG_WAIT_EDGE;
                sg_cnt   = 0;
            }
            break;

        case SG_REARM:
            if (!raw)
            {
                sg_cnt++;
                if (sg_cnt >= SG_REARM_M)
                {
                    sg_state = SG_WAIT_EDGE;
                    sg_cnt   = 0;
                }
            }
            else
            {
                sg_cnt = 0; // 还黑着，继续等白
            }
            break;

        default:
            sg_state = SG_WAIT_EDGE;
            sg_cnt   = 0;
            break;
    }

    DBG_NAV_STATE     = (uint8_t)sg_state;
    DBG_NAV_LINES     = (uint16_t)sg_line_count;
    DBG_NAV_GAP       = sg_gap;
    DBG_NAV_PREVGAP   = sg_prev_gap;
    DBG_NAV_SEG_TICKS = Nav.seg_ticks;
    DBG_NAV_CAP       = (uint16_t)sg_cap_ticks;
}

// ============================================================================
// 圆心判定（出发→圆心段）
//   签名：本次间隔 ×4 < 上次间隔，且本次间隔不超过 NAV_CENTER_GAP_MAX 拍
//         （50mm 紧跟 336mm；排除内圆 242 → 315° 336 这类正常长间隔）
//   返回：0=未到圆心 1=第5条+签名 2=签名兜底(少一条) 3=第6条兜底
// ============================================================================
static uint8_t Nav_Center_Check(void)
{
    if (sg_line_count >= 2 && sg_prev_gap >= NAV_CENTER_PREV_MIN &&
        sg_gap <= NAV_CENTER_GAP_MAX &&
        (uint32_t)sg_gap * NAV_CENTER_GAP_RATIO < (uint32_t)sg_prev_gap)
    {
        return (sg_line_count >= NAV_CENTER_LINE) ? 1u : 2u;
    }

    if (sg_line_count >= NAV_CENTER_LINE_FB) return 3u;

    return 0;
}

// ============================================================================
// 任意角带符号转向（开环）：角度 = k × (t − T0)，线性模型不变
//   deg > 0 右转（顺时针，φ 增大）；deg < 0 左转（逆时针）
//
// ★2026-10-27 用三次导航实跑数据重新拟合参数（逻辑没动，只改 T0 / RATE）：
//     546ms → 45°
//     741ms → 72.5°   (RATE=23)
//     1409ms → 137.5° (RATE=17，差40多度)
//   最小二乘 → k ≈ 0.1045°/ms、起步死区 t0 ≈ 65ms
//   ⚠ 关键：原来的 T0=350 是拿「标定程序」数据标的，标定时车没停稳，
//     把一部分纯旋转算成了死区。导航里 SETTLE=600ms 完全静止后，
//     真正的死区只有 ~65ms。所以 T0 必须一起降，只调 RATE 永远对不上。
//   新参数 T0=50 / RATE=10 的预测：45°→45°、90°→92°、135°→139°、180°→186°
// ============================================================================
// 转向时长：ms = t0 + |deg| × 100 / rate
static uint32_t Nav_Spin_Ms(uint16_t a, uint16_t t0, uint16_t rate)
{
    if (rate == 0u) rate = 1u;
    return (uint32_t)t0 + (uint32_t)a * 100u / (uint32_t)rate;
}

static void Nav_SpinDeg(int16_t deg)
{
    uint32_t ms;
    uint16_t a;

    if (deg == 0) return;

    a  = (uint16_t)((deg > 0) ? deg : -deg);
    ms = Nav_Spin_Ms(a, NAV_TURN_T0_MS, NAV_TURN_RATE_CENTI);

    Spin((deg > 0) ? NAV_TURN_SPIN_V : -NAV_TURN_SPIN_V);
    delay_ms((uint16_t)ms);
    Stop();
}

// ============================================================================
// 开环转向标定（不跑导航，main.c 的 Turn_Calib_Test 调用）
//   DBG_TURN_DEG  : 目标角度，Watch 里直接改（正=右转、负=左转）
//   DBG_TURN_T0   : 起步滞后 ms（默认 NAV_TURN_T0_MS=350）
//   DBG_TURN_RATE : 角速度，单位 0.01°/ms（14 = 0.14°/ms）
//   标定流程见开发记录 §22。收敛后把 T0/RATE 写回 nav.h 的两个宏。
//   注意：这两个值只在标定程序里生效，正式导航仍用宏，避免被误改。
// ============================================================================
volatile int16_t  DBG_TURN_DEG     = 180;
volatile uint16_t DBG_TURN_T0      = NAV_TURN_T0_MS;
volatile uint16_t DBG_TURN_RATE    = NAV_TURN_RATE_CENTI;
volatile uint16_t DBG_TURN_LAST_MS = 0;
volatile uint16_t DBG_TURN_COUNT   = 0;
// ★非 0 时直接用这个 ms 转，忽略 T0/RATE —— 用来直接试「N ms 到底转多少度」
//   标定查表时最方便：设 DBG_TURN_DEG=180、DBG_TURN_MS_OVERRIDE=1900，看转多少。
volatile uint16_t DBG_TURN_MS_OVERRIDE = 0;

// 线性式：只给标定程序用（导航已改查表，不走这里）
static uint32_t Nav_Spin_Ms_Linear(uint16_t a, uint16_t t0, uint16_t rate)
{
    if (rate == 0u) rate = 1u;
    return (uint32_t)t0 + (uint32_t)a * 100u / (uint32_t)rate;
}

void Nav_Calib_SpinDeg(void)
{
    int16_t  deg = DBG_TURN_DEG;
    uint32_t ms;
    uint16_t a;

    if (deg == 0) return;

    a  = (uint16_t)((deg > 0) ? deg : -deg);
    ms = (DBG_TURN_MS_OVERRIDE != 0u)
       ? (uint32_t)DBG_TURN_MS_OVERRIDE
       : Nav_Spin_Ms_Linear(a, DBG_TURN_T0, DBG_TURN_RATE);

    DBG_TURN_LAST_MS = (uint16_t)ms;
    DBG_TURN_COUNT++;

    Spin((deg > 0) ? NAV_TURN_SPIN_V : -NAV_TURN_SPIN_V);
    delay_ms((uint16_t)ms);
    Stop();
}

// 目标角 t 与当前朝向 h 的最短带符号差（+ = 右转）
static int16_t Nav_Delta(uint16_t t, uint16_t h)
{
    int16_t d = (int16_t)((t + 360u - h) % 360u);
    if (d > 180) d -= 360;
    return d;
}

// 转向并更新朝向
//   ★转向后一律把巡航速度压到 NAV_TURN_EXIT_SPEED 再爬坡：
//     转向刚结束、航向误差最大，此时若以 20 全速修正，PD 差速会把外侧轮推到 28，
//     一次修正就是一记猛回头（车头画蛇、夹着的料被甩出去）。详见 trace.h 的说明。
static void Nav_Turn_To(uint16_t target)
{
    int16_t d = Nav_Delta(target, sg_heading);

    // 留档：head_before + d 应等于 target；实转不足时看这三个数就能分清是
    //   "d 本身算错（账本/朝向漂了）" 还是 "d 对但车没转够（标定/机械）"
    DBG_NAV_HEAD_BEFORE = (uint8_t)sg_heading;
    DBG_NAV_TURN_TGT    = (uint8_t)target;
    DBG_NAV_TURN_DEG    = d;
    DBG_NAV_TURN_MS     = (uint16_t)Nav_Spin_Ms((uint16_t)((d > 0) ? d : -d),
                                                NAV_TURN_T0_MS, NAV_TURN_RATE_CENTI);

    if (NAV_TURN_LOG_NUM < NAV_TURN_LOG_N)
    {
        uint8_t k = NAV_TURN_LOG_NUM;
        NAV_TURN_LOG_HEAD[k] = (uint8_t)sg_heading;
        NAV_TURN_LOG_TGT[k]  = (uint8_t)target;
        NAV_TURN_LOG_MS[k]   = DBG_NAV_TURN_MS;
        NAV_TURN_LOG_NUM++;
    }

    Nav_SpinDeg(d);
    sg_heading = (uint16_t)(((int16_t)sg_heading + d + 360) % 360);
    Trace_SpeedKick(NAV_TURN_EXIT_SPEED);
}

// 去内圆点的"长间隔线"门限：用上一段实测的最长间隔换算 190mm
//   （第一段斜线距起点只有 d≈100mm，内圆还要再走 ≈277mm，门限取两者中间）
static void Nav_Compute_Pick_Gate(void)
{
    if (sg_max_gap >= 200u)
    {
        sg_a_min_gap = (uint16_t)((uint32_t)sg_max_gap * 190u / 336u);
        if (sg_a_min_gap < 80u) sg_a_min_gap = 80u;
    }
    else
    {
        sg_a_min_gap = NAV_A_LINE_MIN_GAP;
    }
}

// 某靶位角现在是否被挡（"可投/需暂存"表）
//   绿0 被 A 挡、红90 被 C 挡、蓝180 被 E 挡（这三条射线在 R≈390 有料）
//   白45 被 B 暂存挡、黑135 被 D 暂存挡
static uint8_t Nav_Blocked_For(uint8_t t, uint8_t ignore_slot);

static uint8_t Nav_Blocked(uint8_t t)
{
    return Nav_Blocked_For(t, 0xFFu);
}

// 同上，但忽略某个暂存槽（用于判断"把这块取回后，它的目标线是否空"）
static uint8_t Nav_Blocked_For(uint8_t t, uint8_t ignore_slot)
{
    switch (t)
    {
        case 0u:   return (sg_taken[0] == 0u) ? 1u : 0u;
        case 90u:  return (sg_taken[1] == 0u) ? 1u : 0u;
        case 180u: return (sg_taken[2] == 0u) ? 1u : 0u;
        case 45u:  return (ignore_slot == 0u) ? 0u : ((sg_stage[0] != 0u) ? 1u : 0u);
        case 135u: return (ignore_slot == 1u) ? 0u : ((sg_stage[1] != 0u) ? 1u : 0u);
        default:   return 1u;
    }
}

// ============================================================================
// 读色（带重试）→ 目标角
//   夹具闭合后颜色模块正对物料，所以必须在 Steering_RESET() 之后读。
// ============================================================================
//   ★2026-10-27 修：原来「读到一个有效值就用」，夹爪刚到位时的一次过渡读数
//     （实测把红色读成黑色）会被直接采信 → 送错靶位。
//     现在要求**连续两次读到同一个颜色**才采信；读不到有效值就继续读。
static uint8_t Nav_ReadColor(void)
{
    uint8_t c     = (uint8_t)TCS3200_COLOR_NONE;
    uint8_t valid = (uint8_t)TCS3200_COLOR_NONE; // 上一次读到、且有效的颜色
    uint8_t i;

    for (i = 0u; i <= NAV_COLOR_RETRY + 1u; i++)
    {
        c = (uint8_t)TCS3200_ReadColorStable(NAV_COLOR_SAMPLES);
        if (c == (uint8_t)TCS3200_COLOR_NONE ||
            c == (uint8_t)TCS3200_COLOR_UNKNOWN)
        {
            continue;                 // 无效读数不参与比对
        }
        if (c == valid)
        {
            return c;                 // 连续两次一致 → 采信
        }
        valid = c;                    // 第一次有效读数先记下，等下一次确认
    }

    // 只读到一次有效值（没等到确认）：宁可用它也别整局报废，
    // 但调用方会用 DBG_NAV_COLOR 看到它，便于事后核对
    if (valid == (uint8_t)TCS3200_COLOR_NONE) return (uint8_t)TCS3200_COLOR_NONE;
    return valid;
}

// 颜色 → 靶位角：绿0 白45 红90 黑135 蓝180（靶心都在大圆上）
static uint8_t Nav_ColorToAngle(uint8_t c)
{
    switch (c)
    {
        case (uint8_t)TCS3200_COLOR_GREEN: return 0u;
        case (uint8_t)TCS3200_COLOR_WHITE: return 45u;
        case (uint8_t)TCS3200_COLOR_RED:   return 90u;
        case (uint8_t)TCS3200_COLOR_BLACK: return 135u;
        case (uint8_t)TCS3200_COLOR_BLUE:  return 180u;
        default:                           return 0xFFu;
    }
}

// ============================================================================
// 选下一步（手上没料时调用）
//   ① 内联清场：暂存块的目标线已经空了 → 立刻去取回投掉（保证 B/D 最多一块）
//   ② 暂存块的目标被 A/C/E 挡着 → 先去把挡路的那个点取走
//   ③ 默认顺序 C(90) → E(180) → A(0)
//   返回 0 = 三块都取完且没有暂存块 → 该回家了
// ============================================================================
static uint8_t Nav_Plan_Pick(void)
{
    uint8_t i, t;

    // ① 内联清场
    for (i = 0u; i < 2u; i++)
    {
        if (sg_stage[i] == 0u) continue;
        t = Nav_ColorToAngle(sg_stage[i]);
        if (t == 0xFFu) { sg_stage[i] = 0u; continue; } // 脏数据，清掉
        // 判断"取回后能不能投"，必须忽略它自己占的那个槽（白/黑的目标线就是 B/D 本身）
        if (!Nav_Blocked_For(t, i))
        {
            sg_retrieve   = 1u;
            sg_stage_slot = i;
            sg_pick_angle = (i == 0u) ? 45u : 135u;
            return 1u;
        }
    }

    // ② 去取挡路的料
    for (i = 0u; i < 2u; i++)
    {
        if (sg_stage[i] == 0u) continue;
        t = Nav_ColorToAngle(sg_stage[i]);
        if ((t == 0u || t == 90u || t == 180u) && sg_taken[t / 90u] == 0u)
        {
            sg_retrieve   = 0u;
            sg_pick_angle = t;
            return 1u;
        }
    }

    // ③ 默认顺序
    if (sg_picked < 3u)
    {
        if      (sg_taken[1] == 0u) sg_pick_angle = 90u;
        else if (sg_taken[2] == 0u) sg_pick_angle = 180u;
        else if (sg_taken[0] == 0u) sg_pick_angle = 0u;
        else return 0u; // 账本不一致，保守回家
        sg_retrieve = 0u;
        return 1u;
    }

    return 0u;
}

// ============================================================================
// 回圆心判定（A 倒退 → 圆心）
//   倒退沿途：内圆(起步脚下，门控跳过) → 斜线(≈290mm) → 主辐射线(≈100mm)
//   签名 = 本次间隔骤减（复用圆心判据的比值/门限）
// ============================================================================
static uint8_t Nav_BackCenter_Check(void)
{
    if (sg_line_count >= 2u &&
        sg_prev_gap >= NAV_CENTER_PREV_MIN &&
        sg_gap <= NAV_CENTER_GAP_MAX &&
        (uint32_t)sg_gap * NAV_CENTER_GAP_RATIO < (uint32_t)sg_prev_gap)
    {
        return 1u;
    }
    return 0u;
}

// ============================================================================
// 靶位段数线判定
//   每拍调用；只在"又确认了一条新线"时计数，避免同一条线被重复算。
//   绿（mode=0）：任意线都算，第 1 条 = 大圆
//   其它（mode=1）：只有长间隔线才算，第 1 条 = 内圆、第 2 条 = 大圆
// ============================================================================
// 靶位主判据：前灰度判"彩盘打断黑线"的特征
//   前灰度在车头、离料近；侧灰度在车中心，它跨大圆时料已经冲出靶心 ≈12cm，
//   所以到点必须用前灰度。
//   两道门（防内圆 / 防空车误触发）：
//     门 1（硬性）：DBG_NAV_SERVO == 1，即夹着料。放料只可能在夹着料时发生，
//                   放完料张开、空车回来经过小圆时这道门直接关死。
//     门 2        ：侧灰度已跨过内圆（sg_tgt_long ≥ 1），绿靶不需要（A 已在内圆外）。
//   门都通过后，前灰度看到 1 次靶位特征（连续 NAV_TARGET_BLACK_K 拍）即判到点。
// ============================================================================
static void Nav_Front_Reset(void)
{
    sg_fr_armed     = 0u;
    sg_fr_clear     = 0u;
    sg_fr_cnt       = 0u;
    sg_fr_mid_black = 0u;
    sg_fr_edge_cnt  = 0u;
}

// 靶位特征：彩盘把黑线在中间"断开"
//   非黑料：中间区域白 + 两侧黑
//           中间区域 = 探头3~6（bit5~bit2），只要其中 ≥2 个是白就算数，
//           不再要求"正中间两个探头一定白"——车横向偏了也能命中。
//   黑料  ：圆盘和黑线连成一片 → 中间区域整段黑
static uint8_t Nav_Front_Zone_Sig(uint8_t d)
{
    uint8_t mid;
    uint8_t n;

    if (sg_target_color == (uint8_t)TCS3200_COLOR_BLACK)
    {
        if ((d & NAV_FR_MID_WHITE) != NAV_FR_MID_WHITE) return 0u;
        if ((d & NAV_FR_LEFT_BLACK) == 0u) return 0u;
        if ((d & NAV_FR_RIGHT_BLACK) == 0u) return 0u;
        return 1u;
    }

    // 中间区域数白的个数（黑=1，取反后为 1 的位就是白）
    mid = (uint8_t)(~d & NAV_FR_MID_WHITE);
    n = 0u;
    if (mid & 0x20u) n++;
    if (mid & 0x10u) n++;
    if (mid & 0x08u) n++;
    if (mid & 0x04u) n++;
    if (n < NAV_FR_MID_MIN_WHITE) return 0u;

    if ((d & NAV_FR_LEFT_BLACK) == 0u) return 0u;  // 左边还得有黑线
    if ((d & NAV_FR_RIGHT_BLACK) == 0u) return 0u; // 右边还得有黑线
    return 1u;
}

// ★2026-10-29 前缘触发的「距离门」
//   中间区全白本身也可能是丢线（横向偏出线外），所以必须车已经走到靶位附近才认。
//   绿靶：A 出发已在内圆外，按段内绝对拍数（≈315mm，彩盘前缘 344mm）。
//   非绿：必须已确认内圆，且内圆之后又走满一个内圆间隔（≈290mm）。
static uint8_t Nav_Target_Edge_Armed(void)
{
    if (sg_tgt_mode == 0u)
    {
        return (Nav.seg_ticks >= NAV_TARGET_EDGE_ARM_TICKS) ? 1u : 0u;
    }

    if (sg_tgt_long < 1u || sg_gap == 0u) return 0u;
    if (Nav.seg_ticks < sg_tgt_inner_ticks) return 0u;
    return ((uint32_t)(Nav.seg_ticks - sg_tgt_inner_ticks) >= (uint32_t)sg_gap) ? 1u : 0u;
}

static uint8_t Nav_Target_Front_Check(void)
{
    int16_t pos = F_GW_position();
    uint8_t d   = F_GW_digital_Read();

    // ★门 1（硬性）：夹着料才允许认靶位。
    //   DBG_NAV_SERVO 就是舵机状态、也就是"有没有料"的状态：
    //     1 = 夹住（A 点闭合后一路运到靶位）  0 = 张开（出发前 / 放料后）
    //   放料只可能发生在夹着料的时候，所以这道门是硬性的；
    //   放完料张开、空车回来经过小圆时这里直接不成立，标志位不会被污染。
    if (DBG_NAV_SERVO == 0u)
    {
        Nav_Front_Reset();
        return 0u;
    }

    // ★门 2：靶位特征和内圆十字路口（A~E 的白圆）完全一样，必须先把内圆排除掉。
    //   用【侧灰度已经跨过内圆】来开门（第 1 条长间隔线，已实车验证）。
    //   非绿（sg_tgt_mode=1）：侧灰度没过内圆之前，前灰度看到什么都不算。
    //   绿  （sg_tgt_mode=0）：从 A 出发已经在内圆外面，不需要这道门。
    if (sg_tgt_mode && sg_tgt_long < 1u)
    {
        Nav_Front_Reset();
        return 0u;
    }

    if (!sg_fr_armed)
    {
        // 先连续看到线才算"离开圆心黑区"；还在黑区里就一直重置
        if (pos >= 0)
        {
            if (sg_fr_clear < 0xFFu) sg_fr_clear++;
            if (sg_fr_clear >= NAV_TARGET_ARM_TICKS) sg_fr_armed = 1u;
        }
        else
        {
            sg_fr_clear = 0u;
        }
        return 0u;
    }

    // ★彩盘前缘提前触发（2026-10-29，治「送料过冲 31mm」）
    //   沿射线走时中间区（探头3~6）一直压着辐射线 → 有黑；
    //   进 Ø92 彩盘后中间区全白 → 在「压到大圆黑线」之前 ≈43mm 就判到点。
    //   黑料靶不走这条：黑盘和黑线分不开，仍用下面的老判据。
    if (sg_target_color != (uint8_t)TCS3200_COLOR_BLACK)
    {
        if ((d & NAV_FR_MID_WHITE) != 0u)
        {
            sg_fr_mid_black = NAV_TARGET_EDGE_BLACK_MEM; // 中间区还压着线
            sg_fr_edge_cnt  = 0u;
        }
        else if (sg_fr_mid_black > 0u)
        {
            sg_fr_mid_black--;
            if (sg_fr_edge_cnt < 0xFFu) sg_fr_edge_cnt++;

            if (sg_fr_edge_cnt >= NAV_TARGET_EDGE_K && Nav_Target_Edge_Armed())
            {
                sg_fr_edge_cnt  = 0u;
                sg_fr_mid_black = 0u;
                return 2u; // 2=彩盘前缘（提前≈43mm），1=老判据（压线）
            }
        }
        else
        {
            // 记忆窗过期：已经很久没压到线（多半是丢线），中间全白不算靶位
            sg_fr_edge_cnt = 0u;
        }
    }

    if (Nav_Front_Zone_Sig(d))
    {
        if (sg_fr_cnt < 0xFFu) sg_fr_cnt++;
        if (sg_fr_cnt >= NAV_TARGET_BLACK_K)
        {
            sg_fr_cnt = 0u;
            // 内圆已经被门 2 排除掉了，所以特征只要出现 1 次就是靶位
            return 1u;
        }
    }
    else
    {
        sg_fr_cnt = 0u;
    }

    return 0u;
}

static uint8_t Nav_Target_Check(void)
{
    uint8_t home = (nav_phase == PATH_TO_HOME) ? 1u : 0u;

    if (sg_line_count == 0u) return 0u;
    if (sg_line_count == sg_tgt_last) return 0u; // 没有新线
    sg_tgt_last = sg_line_count;

    // 回家段黑匣子：每条确认线都记间隔（含被门限滤掉的短线）
    if (home && NAV_HOME_N < 6u)
    {
        NAV_HOME_GAP[NAV_HOME_N] = sg_gap;
        NAV_HOME_N++;
    }

    if (sg_tgt_mode && sg_gap < sg_tgt_min_gap) return 0u; // 短间隔线不算

    // ★回家段第 1 条长线组合门（2026-10-29）
    //   起点若停在圆心前，第一条斜线的间隔会被拉大，可能超过 117 的门限被误认成长线，
    //   于是「内圆=2、出发圆=3」在出发圆就停（早 150mm）。
    //   斜线和真内圆的区别：斜线 gap 和 seg_ticks **同时小**；内圆 gap ≈258 拍。
    if (home && sg_tgt_long == 0u &&
        sg_gap < NAV_HOME_FIRST_MIN_GAP &&
        Nav.seg_ticks < NAV_HOME_FIRST_MIN_TICKS)
    {
        return 0u;
    }

    sg_tgt_long++;

    // 内圆（第 1 条长线）之后重算上限：剩 396mm，用刚测的 277mm 间隔换算×2
    //   ⚠ 只用于靶位段。回家段的兜底必须保持 NAV_HOME_MAX_TICKS，
    //     否则漏线时上限会被放大到 ≈970~1100mm，冲出出发区。
    if (sg_tgt_mode && sg_tgt_long == 1u && sg_gap > 0u)
    {
        sg_tgt_inner_ticks = Nav.seg_ticks; // 前缘触发的距离门基准点
        if (nav_phase == PATH_TO_TARGET)
        {
            sg_cap_ticks = (uint32_t)Nav.seg_ticks + (uint32_t)sg_gap * 2u;
            if (sg_cap_ticks > 60000u) sg_cap_ticks = 60000u;
            DBG_NAV_CAP = (uint16_t)sg_cap_ticks;
        }
    }

    // ★回家段总数门：第 3 条线（大圆）必须走够 ≈697mm，否则肯定是多数了一条
    if (home && sg_tgt_long >= sg_tgt_need && Nav.seg_ticks < NAV_HOME_MIN_TOTAL_TICKS)
    {
        return 0u;
    }

    return (sg_tgt_long >= sg_tgt_need) ? 1u : 0u;
}

// ============================================================================
// 初始化 / 启动
// ============================================================================
void Nav_Init(void)
{
    uint8_t i;

    sg_state      = SG_WAIT_EDGE;
    sg_cnt        = 0;
    sg_ticks      = 0;
    sg_gap        = 0;
    sg_prev_gap   = 0;
    sg_max_gap    = 0;
    sg_line_count = 0;

    Nav.line_count = 0;
    Nav.gap        = 0;
    Nav.prev_gap   = 0;
    Nav.seg_ticks  = 0;

    DBG_NAV_POS           = 0;
    DBG_NAV_SGW           = 0;
    DBG_NAV_STATE         = 0;
    DBG_NAV_LINES         = 0;
    DBG_NAV_GAP           = 0;
    DBG_NAV_PREVGAP       = 0;
    DBG_NAV_SEG_TICKS     = 0;
    DBG_NAV_CAP           = 0;
    DBG_NAV_CENTER_REASON = 0;
    DBG_NAV_A_REASON      = 0;
    DBG_NAV_TURN_DONE     = 0;
    DBG_NAV_TURN_POS      = 0;
    DBG_NAV_SERVO         = 0;
    DBG_NAV_COLOR         = 0;
    DBG_NAV_BACK_REASON   = 0;
    DBG_NAV_PLACE_REASON  = 0;
    DBG_NAV_HEAD_BEFORE   = 0;
    DBG_NAV_TURN_TGT      = 0;
    DBG_NAV_TURN_DEG      = 0;
    DBG_NAV_TURN_MS       = 0;
    NAV_TURN_LOG_NUM      = 0;

    sg_target_angle = 0xFFu;
    sg_tgt_mode     = 0u;
    sg_tgt_need     = 1u;
    sg_tgt_min_gap  = NAV_TARGET_MIN_GAP;
    sg_tgt_long     = 0u;
    sg_tgt_last     = 0u;
    sg_tgt_inner_ticks = 0u;
    NAV_HOME_N         = 0u;

    NAV_STOP_SNAPPED = 0;
    NAV_STOP_POS     = 0;
    NAV_STOP_LINES   = 0;
    NAV_STOP_GAP     = 0;
    NAV_STOP_PREVGAP = 0;
    NAV_STOP_PHASE   = 0;

    NAV_LOG_I = 0;
    for (i = 0; i < NAV_LOG_N; i++)
    {
        NAV_LOG_POS[i]     = 0;
        NAV_LOG_LINES[i]   = 0;
        NAV_LOG_GAP[i]     = 0;
        NAV_LOG_PREVGAP[i] = 0;
        NAV_LOG_TICKS[i]   = 0;
        NAV_LOG_PHASE[i]   = 0;
    }

    nav_phase = PATH_IDLE;
}

void Nav_Task2_Start(void)
{
    DBG_NAV_CENTER_REASON = 0;
    DBG_NAV_A_REASON      = 0;
    DBG_NAV_TURN_DONE     = 0;
    DBG_NAV_TURN_POS      = 0;
    DBG_NAV_COLOR         = 0;
    DBG_NAV_BACK_REASON   = 0;
    DBG_NAV_PLACE_REASON  = 0;
    NAV_STOP_SNAPPED      = 0;

    // 相位黑匣子清零（每局重新记）
    NAV_TRC_NUM = 0;
    sg_trc_last = 0xFFu;

    sg_target_angle = 0xFFu;
    sg_tgt_mode     = 0u;
    sg_tgt_need     = 1u;
    sg_tgt_min_gap  = NAV_TARGET_MIN_GAP;
    sg_tgt_long     = 0u;
    sg_tgt_last     = 0u;
    sg_tgt_inner_ticks = 0u;
    NAV_HOME_N         = 0u;

    // 账本清零
    sg_heading       = 90u;            // 出发朝北
    sg_taken[0]      = 0u;
    sg_taken[1]      = 0u;
    sg_taken[2]      = 0u;
    sg_stage[0]      = 0u;
    sg_stage[1]      = 0u;
    sg_claw          = 0u;
    sg_picked        = 0u;
    sg_retrieve      = 0u;
    sg_stage_slot    = 0xFFu;
    sg_pick_angle    = 90u;            // 第一块先取 C（正前方，不用转）
    sg_place_fine_ms = DBG_NAV_FINE_MS;   // Watch 可改，重开局生效

    sg_cap_ticks = NAV_CENTER_MAX_TICKS; // 兜底上限，数到第 2 条线后自动重算

    // 出发前张开夹爪：物料在 A 点，是车把它送进夹爪口的，
    // 夹爪闭着过去就夹不住，所以必须先张开并等它摆到位再起车。
    Steering_SET();
    DBG_NAV_SERVO = 0;
    delay_ms(NAV_SERVO_OPEN_MS);

    Nav_Counter_Init(1); // 出发段：脚下若压着大圆线，算第 1 条
    nav_phase = PATH_TO_CENTER;
}

// ============================================================================
// 第一题主循环：出发区 → 圆心 → 左转 90° → A 点
// ============================================================================
void Nav_Task2_Run(void)
{
    uint8_t reason;

    switch (nav_phase)
    {
        case PATH_TO_CENTER:
            Nav_Update();
            Trace_Forward();          // 前灰度 PD + 丢线回找（trace.c，实车已验证）
            DBG_NAV_POS = DBG_pos;    // 镜像到 nav 调试变量

            reason = Nav_Center_Check();
            if (reason)
            {
                // 圆心：侧灰度跨过 0° 辐射线，停车 → 查账本选第一块
                DBG_NAV_CENTER_REASON = reason;
                Stop();
                Nav_StopSnap();
                sg_heading = 90u;        // 出发段朝北，在格点上重新锚定
                nav_phase  = PATH_DECIDE;
            }
            else if ((uint32_t)Nav.seg_ticks >= sg_cap_ticks)
            {
                // 兜底：自适应拍数上限，防止冲过圆心撞 C 点物料
                DBG_NAV_CENTER_REASON = 9;
                Stop();
                Nav_StopSnap();
                nav_phase = PATH_ABORT;
            }
            break;

        // --------------------------------------------------------------------
        // 通用转向：转到 sg_pick_angle（45° 格点，最短方向）
        // --------------------------------------------------------------------
        case PATH_TURN_NAV:
            Stop();
            delay_ms(NAV_TURN_SETTLE_MS);   // 停稳再转（速度 18 需要更久，见 nav.h）
            Nav_Turn_To(sg_pick_angle); // 开环转向 + 更新 sg_heading
            DBG_NAV_TURN_POS  = F_GW_position();
            DBG_NAV_TURN_DONE = 1;

            // 去内圆点段的上限与门限：用上一段最长间隔换算 390mm
            Nav_Compute_Pick_Gate();
            if (sg_max_gap >= 200u)
            {
                sg_cap_ticks = (uint32_t)sg_max_gap * 390u / 336u * 3u / 2u;
            }
            else
            {
                sg_cap_ticks = NAV_A_MAX_TICKS;
            }

            // 转弯后：脚下可能是辐射线或圆心黑区，先等白了再数
            Nav_Counter_Init(0);
            nav_phase = PATH_TO_POINT;
            break;

        // --------------------------------------------------------------------
        // 通用去内圆点：A/C/E 取料、B/D 暂存/取回
        //   停车判据 = 距上一条线足够远的长间隔线（内圆）
        // --------------------------------------------------------------------
        case PATH_TO_POINT:
            Nav_Update();
            Trace_Forward();
            DBG_NAV_POS = DBG_pos;

            if (sg_line_count >= 1 && sg_gap >= sg_a_min_gap &&
                Nav.seg_ticks >= NAV_A_MIN_TICKS)
            {
                DBG_NAV_A_REASON = 1;
                Stop();
                Nav_StopSnap();
                // 侧灰度在车中间、夹具在车头前方：侧方到点停车时
                // 物料已完整进夹口，不需要微进，停车直接夹取+读色。
                // 手上已经有料 = 这趟是去 B/D 暂存，直接放，不再夹取。
                nav_phase = (sg_claw != 0u) ? PATH_PLACE : PATH_CLAMP_READ;
            }
            else if ((uint32_t)Nav.seg_ticks >= sg_cap_ticks)
            {
                // ★2026-10-27 二次评估：**撤回**上一版「撞上限也照常夹取」的兜底。
                //   实车证明它有害：车走错射线时会一路开到上限，然后在**错误位置**
                //   闭合夹爪（抓空）、读到错误颜色、再送进错误靶位 ——
                //   现场看到「回家后夹爪动了、抓空、又送去黑色区」就是这么来的。
                //   停在原地至少是确定的失败，还能保留 DBG_NAV_A_REASON=9 的依据。
                //   真正该修的是「为什么没走到点」（转向/抓线，见 NAV_TRC_*）。
                DBG_NAV_A_REASON = 9;
                Stop();
                Nav_StopSnap();
                nav_phase = PATH_ABORT;
            }
            break;

        // --------------------------------------------------------------------
        // 内圆点：夹住 → 读色 → 记账 → 定下一步（一次性阻塞序列）
        //   读色必须在闭合之后：夹具闭合并把颜色模块送到物料正前方。
        // --------------------------------------------------------------------
        case PATH_CLAMP_READ:
        {
            uint8_t c, t;

            Stop();
            delay_ms(NAV_CLAMP_DELAY_MS); // 停稳再夹
            Steering_RESET();
            DBG_NAV_SERVO = 1;
            delay_ms(NAV_CLAMP_SETTLE_MS); // 等夹爪力建立再读

            c = Nav_ReadColor();
            t = Nav_ColorToAngle(c);

            DBG_NAV_COLOR   = c;  // 唯一的判色留档：1白 2黑 3红 4绿 5蓝 0=读不出来
            sg_target_color = c;  // 靶位特征要用：黑料靶是全黑，其它是"中间白两侧黑"

            if (t == 0xFFu)
            {
                // 读不到色（夹空/没对正）：不带空夹爪乱投，直接异常停车
                Stop();
                Nav_StopSnap();
                nav_phase = PATH_ABORT;
                break;
            }

            sg_claw         = c;
            sg_target_angle = t;
            sg_tgt_long     = 0u;
            sg_tgt_last     = 0u;

            // ---- 记账 ----
            if (sg_retrieve)
            {
                // 从 B/D 取回：清暂存账（取回后必然能投，见 Nav_Plan_Pick 的内联清场）
                if (sg_stage_slot < 2u) sg_stage[sg_stage_slot] = 0u;
                sg_stage_slot = 0xFFu; // 关键：别让 PLACE 又把它当成"往暂存点放"
                sg_retrieve   = 0u;
            }
            else
            {
                // 从 A/C/E 取料：记"该点已空"
                if (sg_pick_angle == 0u)        sg_taken[0] = 1u;
                else if (sg_pick_angle == 90u)  sg_taken[1] = 1u;
                else if (sg_pick_angle == 180u) sg_taken[2] = 1u;
                if (sg_picked < 3u) sg_picked++;
            }

            if (t == sg_pick_angle)
            {
                // 靶位角正好是脚下这条射线 → 直接从内圆前进到大圆，
                // 不回圆心、不转向，省 780mm 和一次转向误差。沿途只有大圆一条线。
                sg_tgt_mode  = 0u;
                sg_tgt_need  = 1u;
                sg_tgt_long  = 0u;              // 必须清：残留值会让第一条线被 sg_tgt_last 吞掉
                sg_tgt_last  = 0u;
                sg_tgt_inner_ticks = 0u;
                sg_cap_ticks = NAV_TARGET_GREEN_MAX_TICKS;
                Nav_Front_Reset();
                Nav_Counter_Init(0); // 脚下是内圆线，跳过
                nav_phase = PATH_TO_TARGET;
            }
            else
            {
                // 其它靶位：必须先退回圆心，再转向沿目标射线出去
                // （直走只会走弦，落不到靶心）
                sg_tgt_mode  = 1u;
                sg_tgt_need  = NAV_TARGET_LONG_NEED;
                sg_cap_ticks = NAV_BACK_MAX_TICKS;
                Nav_Counter_Init(0); // 脚下是内圆线，跳过
                nav_phase = PATH_BACK_TO_CENTER;
            }
            break;
        }

        // --------------------------------------------------------------------
        // 非绿：倒退回圆心（不掉头——掉头会扫到夹在车头的料）
        // 停因：侧灰度间隔签名（斜线≈290mm → 主辐射线≈100mm）
        // --------------------------------------------------------------------
        case PATH_BACK_TO_CENTER:
            Nav_Update();
            Trace_Backward();
            DBG_NAV_POS = DBG_pos;

            // 一步停车：签名成立的那一刻，车心就在圆心。
            //   ★2026-10-27 修（这是「回圆心多退 ≈100mm」的根因）：
            //   原注释以为签名在「45° 斜线」处成立，再退一条线才到圆心 —— 前提是错的。
            //   实际数线序列（A 点沿 0° 射线倒退，侧灰度偏移 d≈100mm）：
            //     内圆(起步附近, gap≈16拍, prev_gap=0)     → 签名不成立
            //     45°斜线(距圆心 d, gap≈246拍, prev_gap≈16)→ prev_gap<100，签名不成立
            //     主辐射线(过圆心, gap≈89拍, prev_gap≈246) → 签名成立，而此刻车心已在圆心
            //   跨 45° 线时 prev_gap 拿到的是「内圆→45°」那个短间隔，所以签名只会
            //   在跨主轴线时成立。原来再等下一根线 = 多退 d ≈100mm，
            //   实车表现为「侧灰度退到米字线右下」→ 转 45° 后车不在射线上、前灰度全白。
            if (Nav_BackCenter_Check())
            {
                DBG_NAV_BACK_SIG_GAP = sg_gap; // 签名段的间隔拍数，≈89 表示正常跨到圆心
                DBG_NAV_BACK_REASON  = 2u;
                Stop();
                delay_ms(200); // 惯性停稳再决策
                Nav_StopSnap();
                nav_phase = PATH_DECIDE;
                break;
            }

            if (nav_phase == PATH_BACK_TO_CENTER &&
                (uint32_t)Nav.seg_ticks >= sg_cap_ticks)
            {
                DBG_NAV_BACK_REASON = 9;
                Stop();
                Nav_StopSnap();
                nav_phase = PATH_ABORT;
            }
            break;

        // --------------------------------------------------------------------
        // 圆心：转到目标 φ（0°=西 → φ 增大都是顺时针）
        // --------------------------------------------------------------------
        case PATH_TURN_TO_TARGET:
            Stop();
            delay_ms(NAV_TURN_SETTLE_MS);   // 停稳再转（与标定条件一致）
            Nav_Turn_To(sg_target_angle);
            DBG_NAV_TURN_POS  = F_GW_position();
            DBG_NAV_TURN_DONE = 1;

            sg_tgt_mode    = 1u;
            sg_tgt_need    = NAV_TARGET_LONG_NEED;
            sg_tgt_min_gap = NAV_TARGET_MIN_GAP;
            sg_tgt_long    = 0u;
            sg_tgt_last    = 0u;
            sg_tgt_inner_ticks = 0u;
            sg_cap_ticks   = NAV_TARGET_MAX_TICKS;
            Nav_Front_Reset();
            Nav_Counter_Init(0); // 圆心黑芯/射线上，等白了再数
            nav_phase = PATH_TO_TARGET;
            break;

        // --------------------------------------------------------------------
        // 沿 φ 射线前进到大圆 = 靶位
        // --------------------------------------------------------------------
        case PATH_TO_TARGET:
        {
            uint8_t hit = 0u;
            uint8_t fr;

            Nav_Update();
            Trace_Forward();
            DBG_NAV_POS = DBG_pos;

            // 主判据：前灰度靶位（离料近，不会把料送出靶心）
            //   返回 2=彩盘前缘（新，提前≈43mm）  1=压到大圆黑线（老判据，黑料靶走这条）
            // 兜底：侧灰度长间隔线（前灰度万一没识别到，退回老判据，宁可偏外也别冲过）
            fr = Nav_Target_Front_Check();
            if (fr)
            {
                DBG_NAV_PLACE_REASON = (fr == 2u) ? 3u : 1u;
                hit = 1u;
            }
            else if (Nav_Target_Check())
            {
                DBG_NAV_PLACE_REASON = 2; // 侧灰度兜底（偏外，说明前灰度没认到）
                hit = 1u;
            }

            if (hit)
            {
                Nav_Stop_Brake();   // ★靶位专用：先反向点一下刹住，抵消滑行
                Nav_StopSnap();
                nav_phase = PATH_PLACE;
            }
            else if ((uint32_t)Nav.seg_ticks >= sg_cap_ticks)
            {
                DBG_NAV_PLACE_REASON = 9; // 拍数上限
                Stop();
                Nav_StopSnap();
                nav_phase = PATH_ABORT;
            }
            break;
        }

        // --------------------------------------------------------------------
        // 对位 → 张开舵机放料 → 退后收尾
        //   先张开等料落稳再退，否则会把刚放的料带倒；
        //   退后不转向（原地转会扫飞车头前的料）。
        // --------------------------------------------------------------------
        case PATH_PLACE:
        {
            uint8_t staged = (sg_stage_slot < 2u) ? 1u : 0u;

            Stop();
            delay_ms(200);

            // 对位微调：正=前进、负=后退
            //   投靶位用 DBG_NAV_FINE_MS（料冲过靶心 → 取负）
            //   暂存点用 NAV_STORE_FINE_MS（一般 0）
            //   ★速度用 NAV_FINE_V（不是 NAV_INCH_V）：速度 8 太慢，4 轮滑移底盘
            //     起步要克服静摩擦，实测基本看不到位移 → 微调形同失效。
            //   ★★这段在 Steering_SET() 之前，"退后再放"就是靠它。
            //     张开之后再退（下面第二步）只抽夹爪，改变不了料已经落下的位置。
            DBG_NAV_FINE_APPLIED = sg_place_fine_ms; // 留档：实际用到的是多少
            if (sg_place_fine_ms > 0)
            {
                Diff_Drive(NAV_FINE_V, NAV_FINE_V);
                delay_ms((uint32_t)sg_place_fine_ms);
            }
            else if (sg_place_fine_ms < 0)
            {
                Diff_Drive(-NAV_FINE_V, -NAV_FINE_V);
                delay_ms((uint32_t)(-sg_place_fine_ms));
            }
            Stop_Retry(3);

            Steering_SET(); // 张开放料
            DBG_NAV_SERVO = 0;
            delay_ms(NAV_RELEASE_MS);

            // 退后脱离物料：Watch 里改 DBG_NAV_BACK_MS 可现场加长，下一次放料即生效
            {
                int16_t bo = DBG_NAV_BACK_MS;
                if (bo < 0)    { bo = 0; }
                if (bo > 1500) { bo = 1500; }
                Diff_Drive(-NAV_INCH_V, -NAV_INCH_V);
                delay_ms((uint32_t)bo);
                Stop_Retry(3);
            }

            // 记账
            if (staged)
            {
                sg_stage[sg_stage_slot] = sg_claw; // 记下暂存的是哪个颜色
                sg_stage_slot = 0xFFu;
            }
            sg_claw          = 0u;
            sg_place_fine_ms = DBG_NAV_FINE_MS;   // Watch 可改，重开局生效

            // 回圆心：从内圆点（暂存）退 390mm，从大圆（靶位）退 780mm
            sg_tgt_long  = 0u;
            sg_tgt_last  = 0u;
            sg_cap_ticks = staged ? NAV_BACK_MAX_TICKS : NAV_BACK_BIG_MAX_TICKS;
            Nav_Counter_Init(0);
            nav_phase = staged ? PATH_BACK_TO_CENTER : PATH_BACK_FROM_TARGET;
            break;
        }

        // --------------------------------------------------------------------
        // 决策：手上有料 → 判投/暂存；手上没料 → 选下一块 / 取回暂存 / 回家
        //   本相位不产生运动，一拍即跳转
        // --------------------------------------------------------------------
        case PATH_DECIDE:
        {
            uint8_t t;

            if (sg_claw == 0u)
            {
                if (!Nav_Plan_Pick())
                {
                    nav_phase = PATH_TURN_TO_HOME; // 三块取完、暂存清空 → 回家
                    break;
                }
                // Nav_Plan_Pick 已设好 sg_pick_angle 和 sg_retrieve
                nav_phase = PATH_TURN_NAV;
            }
            else
            {
                t = Nav_ColorToAngle(sg_claw);
                if (t == 0xFFu)
                {
                    Stop();
                    Nav_StopSnap();
                    nav_phase = PATH_ABORT;
                    break;
                }

                if (Nav_Blocked(t))
                {
                    // 目标线在 R≈390 还压着料 → 暂存到 B/D（B 优先）
                    sg_stage_slot    = (sg_stage[0] == 0u) ? 0u : 1u;
                    sg_pick_angle    = (sg_stage_slot == 0u) ? 45u : 135u;
                    sg_place_fine_ms = NAV_STORE_FINE_MS;
                    sg_retrieve      = 0u;
                    nav_phase        = PATH_TURN_NAV;
                }
                else
                {
                    // 可投 → 去靶位
                    sg_stage_slot    = 0xFFu;
                    sg_target_angle  = t;
                    sg_place_fine_ms = DBG_NAV_FINE_MS;   // Watch 可改，重开局生效
                    nav_phase        = PATH_TURN_TO_TARGET;
                }
            }
            break;
        }

        // --------------------------------------------------------------------
        // 靶位放完 → 倒退回圆心（大圆 → 圆心 780mm，不掉头）
        // --------------------------------------------------------------------
        case PATH_BACK_FROM_TARGET:
            Nav_Update();
            Trace_Backward();
            DBG_NAV_POS = DBG_pos;

            // 与 BACK_TO_CENTER 同一套一步停车：签名成立 = 车心已在圆心（根因见上）
            if (Nav_BackCenter_Check())
            {
                DBG_NAV_BACK_SIG_GAP = sg_gap;
                DBG_NAV_BACK_REASON  = 2u;
                Stop();
                delay_ms(200);
                Nav_StopSnap();
                nav_phase = PATH_DECIDE;
                break;
            }

            if (nav_phase == PATH_BACK_FROM_TARGET &&
                (uint32_t)Nav.seg_ticks >= sg_cap_ticks)
            {
                DBG_NAV_BACK_REASON = 9;
                Stop();
                Nav_StopSnap();
                nav_phase = PATH_ABORT;
            }
            break;

        // --------------------------------------------------------------------
        // 回家：圆心转向 270° → 沿 270° 出去，数第 4 条线停在大圆
        // --------------------------------------------------------------------
        case PATH_TURN_TO_HOME:
            Stop();
            delay_ms(NAV_TURN_SETTLE_MS);   // 停稳再转（与标定条件一致）
            Nav_Turn_To(NAV_HOME_ANGLE);
            DBG_NAV_TURN_POS  = F_GW_position();
            DBG_NAV_TURN_DONE = 1;

            // 长间隔线模式：跳过斜线等短线，只认同心圆（不受起步位置/朝向影响）
            sg_tgt_mode    = 1u;
            sg_tgt_need    = NAV_HOME_NEED;
            sg_tgt_min_gap = NAV_HOME_MIN_GAP;
            sg_tgt_long    = 0u;
            sg_tgt_last    = 0u;
            sg_tgt_inner_ticks = 0u;
            sg_cap_ticks   = NAV_HOME_MAX_TICKS;
            NAV_HOME_N     = 0u; // 回家段数线黑匣子清零
            Nav_Counter_Init(0);
            nav_phase = PATH_TO_HOME;
            break;

        case PATH_TO_HOME:
            Nav_Update();
            Trace_Forward();
            DBG_NAV_POS = DBG_pos;

            if (Nav_Target_Check())
            {
                Stop();
                delay_ms(500); // 停稳再结束，给裁判看
                Nav_StopSnap();
                nav_phase = PATH_DONE;
            }
            else if ((uint32_t)Nav.seg_ticks >= sg_cap_ticks)
            {
                // 兜底：停住不再动（车心多半仍在 D320 区内，保住返回分）
                Stop();
                Nav_StopSnap();
                nav_phase = PATH_DONE;
            }
            break;

        case PATH_DONE:
        case PATH_ABORT:
        case PATH_IDLE:
        default:
            Stop();
            break;
    }

    Nav_LogTick();
    Nav_TrcPhase();
}

uint8_t Nav_GetPhase(void)
{
    return (uint8_t)nav_phase;
}
