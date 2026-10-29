#ifndef _nav_H
#define _nav_H

#include "main.h"

// ============================================================================
// 导航层 —— 第一题：出发区 → 圆心 → 左转 90° → A 点
// ----------------------------------------------------------------------------
// 定位模型（用户定稿，见开发记录 16.1）：
//   侧方灰度探头的位置直接当作车的位置；探头"跳变才计数"。
//   出发区→圆心（沿 270° 线向北）探头依次跨过：
//     大圆(0) → 出发圆北缘(≈152) → 内圆(≈394) → 315°线(≈730) → 0°线(=圆心≈780)
//     间距 152 / 242 / 336 / 50 mm —— 最后一段只有上一段的 1/6.7，是圆心签名。
//   圆心→A（沿 0° 线向西）探头依次跨过：
//     315°线(≈50，跳过) → 内圆线(≈387 = A)
//
//   不使用编码器（驱动板 I2C 无上拉，编码器不可用）；
//   不用前/后灰度判圆心和 A（车停圆心时前灰度已离开黑芯，A 处有小白圆）。
//   运动一律用 trace.c 的 Trace_Forward()（前灰度 PD + 丢线回找，实车已验证）。
// ============================================================================

// ----------------------------------------------------------------------------
// 侧方灰度边沿状态机（跳变才计数）
//   SG_DEBOUNCE_K：黑连续 K 拍才确认一条线（10ms/拍，110mm/s ≈1.1mm/拍）
//   SG_REARM_M   ：确认后需白连续 M 拍才重新武装（≈11mm，防同一条线重复计数）
// ----------------------------------------------------------------------------
#define SG_DEBOUNCE_K 3
#define SG_REARM_M    10

// ----------------------------------------------------------------------------
// 出发→圆心段
//   NAV_CENTER_LINE       第 5 条线 = 0° 辐射线 = 圆心
//   NAV_CENTER_LINE_FB    兜底：数到第 6 条线（圆心后 50mm 的 45°线）强制停
//   NAV_CENTER_GAP_RATIO  圆心签名：本次间隔 ×2 < 上次间隔
//     ⚠ 实车实测：侧灰度横向偏移 d≈100mm（不是方案假设的50mm），
//       315°线→0°线 ≈100mm≈130拍，而 315°线自己的间隔 ≈430拍，
//       比例只有 3.3，用 ×4 会漏判。×2 仍能区分正常间隔（242/152=1.6、336/242=1.4）。
//   NAV_CENTER_PREV_MIN   签名要求上次间隔 ≥100 拍（@速度18 拍距1.10）≈110mm
//   NAV_CENTER_GAP_MAX    签名要求本次间隔 ≤176 拍≈216mm，排除噪声短间隔
//
//   NAV_CENTER_MAX_TICKS  ⚠ 只是「一条线都没数到」时的兜底。
//   实车实测：62mm/s、拍周期 11.5ms → 拍距 ≈0.72mm/拍，圆心 780mm ≈1040 拍，
//   固定 800 必在 570mm 截停（电池越弱越严重）。
//   现在每数到第 2/3/4 条线，就用刚测到的间隔换算「剩余里程需要多少拍」，
//   再乘 1.5~2 倍余量作为上限（见 nav.c 的 Nav_Update_Cap），
//   电池掉电、车速变化都不会再提前截停。
// ----------------------------------------------------------------------------
//   ⚠ 2026-10-27 速度 12→18：拍距 0.72→1.10mm/拍，下面门限已 ×0.655（=0.72/1.10）。
//     18 速未实测，v 由 12/20 两点插值得 95.8mm/s；改速度按「×0.72÷实测拍距」重算，见开发记录 §22。
#define NAV_CENTER_LINE       5
#define NAV_CENTER_LINE_FB    6
#define NAV_CENTER_GAP_RATIO  2
#define NAV_CENTER_PREV_MIN   100
#define NAV_CENTER_GAP_MAX    200
#define NAV_CENTER_MAX_TICKS  1200

// ----------------------------------------------------------------------------
// 圆心→A 段
//   只有"距上一条线足够远"的线才可能是内圆。
//   第一段斜线距起点只有 d（探头横向偏移≈100mm），内圆还要再走 ≈277mm，
//   两者比值约 2.8，所以门限取两者中间的 190mm：
//   实际值由转弯时用本段实测拍距换算（见 PATH_TURN_LEFT），
//   电池弱车速慢时门限自动变大，不会把斜线误认成 A。
//   NAV_A_MIN_TICKS 本段至少走 250 拍才允许认 A，
//                   防止转弯后起步加速慢时把 315° 线当成 A。
//   NAV_A_MAX_TICKS 兜底上限；正常情况下转弯前会用出发段最长间隔
//                   （315°线，≈336mm 对应 430 拍）换算出 390mm 需要多少拍，
//                   再乘 1.5 余量（见 PATH_TURN_LEFT），电池掉电也不会提前截停。
// ----------------------------------------------------------------------------
#define NAV_A_LINE_MIN_GAP 100   // 兜底值（150×0.655）；正常由 sg_max_gap×190/336 动态计算
#define NAV_A_MIN_TICKS    167  // 250×0.655≈164，取 167 ≈184mm
#define NAV_A_MAX_TICKS    1000

// ----------------------------------------------------------------------------
// 左转 90°（开环：固定转速 + 固定延时，必须实车标定）
//   NAV_TURN_SPIN_V : 旋转速度
//   NAV_TURN_SPIN_MS: 90° 对应毫秒 —— 开环定时，但**与电池电量无关**
//     电机板自带速度闭环（商家确认），电量不改变实际转速，
//     只有电压低到闭环拉不到 SPIN_V 时才会变慢（判据：同样时间角度明显变小）。
//     所以 800ms→65°、1600ms→180° 的非线性来自「起步滞后」，不是电池：
//       拟合 角度 = k × (t − t0)，t0 ≈ 350ms（电机从 0 爬到目标转速 + 前置延时），
//       k ≈ 0.14°/ms  →  90° ≈ 975ms
//     实测记录：800ms ≈65°（略过 45°射线，被循迹拉回，侧方停在 B 路口，判定正确）
//     调法：转完朝西北（45°射线，去 B）＝转少了，加 50ms；
//           朝西南（315°射线）或更过＝转多了，减 50ms。
//     ⚠ 若要彻底摆脱这个 t0，可改用 Spin_Angle()（编码器测角闭环），
//       但编码器当前因驱动板 I2C 无上拉而不可用。
//   NAV_TURN_SIGN   : -1 = 左转（逆时针，车头由北转向西 A 点方向）
//                     +1 = 右转（如果实车发现转完朝东、走向 E 点，就改成 +1）
// ----------------------------------------------------------------------------
//   ⚠ 多块料版已统一用 Nav_Turn_To()（下面 NAV_TURN_T0_MS / RATE_CENTI 的拟合式），
//     这两个只作标定参考保留，代码不再使用。
#define NAV_TURN_SPIN_V  30
#define NAV_TURN_SPIN_MS 950
#define NAV_TURN_SIGN    (-1)

// ----------------------------------------------------------------------------
// 任意角转向（靶位方向用）：同样是开环，但按拟合式换算，不能用 950 的比例！
//   拟合：角度 = 0.14°/ms × (t − 350ms)
//   反解：t = NAV_TURN_T0_MS + |deg| × 100 / NAV_TURN_RATE_CENTI
//   为什么不能用比例：45° 按 950 线性折算只有 475ms，而起步滞后就 350ms，
//   真正在转的只有 125ms → 车几乎不动。必须把 t0 加上。
//   实车标定：地上画 45/90/135/180 参考线各转 5~10 次取平均。
//   符号：Nav_SpinDeg 正 = 右转（顺时针，φ 增大），负 = 左转。
// ----------------------------------------------------------------------------
// 转弯前停稳时间（★2026-10-27 加）
//   原来是写死的 250ms，速度 12 时够用；提到 18/20 后车还没完全停住就开始 Spin，
//   前半段旋转被「从前进减速到零再反向」吃掉 → 实际转角偏小，而且每次差一点。
//   更要命的是：标定程序 Turn_Calib_Test 是**从静止起步**转的，实跑是**高速急停后**转的，
//   标定条件和实跑条件不一致，标出来的 RATE 搬过去必然偏。
//   加长到 600ms 后，实跑条件 ≈ 标定条件（都从完全静止起步）。
//   ⚠ 改这个值之后必须重新标 RATE，否则标定值对不上。
#define NAV_TURN_SETTLE_MS    450   // ★2026-10-29 600→450（减少停顿；仍远长于停车滑行时间）
#define NAV_TURN_T0_MS        110   // ★350→50→110。实跑 180° 偏大（90° 能靠循迹找回、
                                   //   180° 找不回），整体缩一档：RATE 10→11 把斜率抬起来，
                                   //   小角度那段用 T0 补回来，避免 45° 转少了。
// 转向结束后重新循迹的起步速度（随后自动爬坡回 LINE_SPEED）
//   20 速全速修正 = 外侧轮 28（比巡航快 40%）→ 猛回头、画蛇、把料甩出去。
//   降到 12 起步：外侧轮只有 17，修正平缓。爬坡 5 拍/档，12→20 约 0.46s 回满。
//   若转向后仍然摆：继续降到 10；若觉得加速太肉：把 TRACE_RAMP_STEP_TICKS 调小。
#define NAV_TURN_EXIT_SPEED   12
#define NAV_TURN_RATE_CENTI   11   // 0.11°/ms。180° 实测偏大后上调一档（10→11）
                                   // 自检（ms = 110 + deg×100/11）：
                                   //   45°→519、90°→928、135°→1337、180°→1746
                                   // 对比上一版（T0=50/RATE=10）：
                                   //   45° 500→519（+19ms，小角度补偿）
                                   //   90° 950→928（−22ms）
                                   //   180° 1850→1746（−104ms ≈ 少转 11°）
                                   // 只做粗定位，45° 格点上的 ±10° 由前灰度循迹拉回

// ----------------------------------------------------------------------------
// A 点取料 → 读色送靶位（单块闭环）
//   车停 A 时侧灰度在 A 十字路口：侧灰度装在车中间、夹具在车头前方，
//   此时物料已完整进入夹口（实测确认）→ **不需要微进**，停车即可夹。
//   夹住后夹具正好把颜色模块送到物料正前方 → **夹完再读色**。
// ----------------------------------------------------------------------------
//   ★2026-10-27 修：原来 300ms，但舵机全行程 59° 约需 0.6s（见 steering.c 注释），
//     加上 CLAMP_DELAY 300ms 总共只有 600ms —— 正好卡在「刚到位」，读色时夹爪还在动，
//     颜色模块没对准料 → 实测把红色读成黑色 → 送错靶位。
//     现在留 400ms 余量：300+700=1000ms，舵机肯定到位后再读。
#define NAV_CLAMP_SETTLE_MS 700  // 夹紧后等夹爪完全到位再读色（≥舵机全行程 600ms + 余量）
#define NAV_COLOR_SAMPLES   3    // ReadColorStable 投票次数（≈660ms）
#define NAV_COLOR_RETRY     2    // 读不到时原地重读次数，仍失败则 ABORT
#define NAV_INCH_V          12   // 张开后"退后脱离物料"用的速度。原来 8，实测
                                 //   低于滑移底盘起步门限 → 350ms 几乎不动，肉眼看不出后退。
#define NAV_FINE_V          12   // ★靶位对位微调用的速度。原来借用速度 8，
                                 //   实测太慢：4 轮滑移底盘静摩擦大，几百毫秒几乎不动。
                                 //   12 ≈ 0.065mm/ms（按 18 速 96.8mm/s 线性折算，实际略小）
#define NAV_BRAKE_V         10   // 靶位到点时的反向刹车速度（正=前进，取负=后退）
#define NAV_BRAKE_MS        120  // 反向刹车脉宽：先反向点一下再 Stop_Retry
#define NAV_RELEASE_MS      200  // 张开后等料落稳，再退（原 300）
#define NAV_BACKOFF_MS      200  // 张开后退后：只负责把夹爪从料上抽走（≈13mm）
                                 //   ★不影响落点（料在张开瞬间已脱手）。
                                 //   这个量越大，回程起点越靠后，越可能把身后那条
                                 //   大圆线重新压回去 → 数多一条线。有异常先调 0。
                                 //   现场用 DBG_NAV_BACK_MS 在 Watch 里直接改，无需重编译

//   回圆心（A→圆心倒退 390mm，后灰度 PD）
//   签名：最后两条线（斜线 → 主辐射线）间隔骤减，复用圆心判据
//   ★★2026-10-27 终极修复（「回圆心多退 ≈100mm」的根因）：
//     「签名在斜线处成立、再退一条线才到圆心」这个前提是错的。实际数线序列：
//       内圆(起步附近, prev_gap=0)      → 不成立
//       45°斜线(距圆心 d, prev_gap≈16)  → prev_gap<100，不成立
//       主辐射线(过圆心, prev_gap≈246)  → 成立，而此刻车心已在圆心
//     所以签名只会在跨主轴线时成立，再等下一根线必然多退 d≈100mm
//     （实车表现：侧灰度退到米字线右下 → 转 45° 后车不在射线上、前灰度全白）。
//     现在改为「签名成立即停」，见 nav.c 的 PATH_BACK_TO_CENTER / BACK_FROM_TARGET。
#define NAV_BACK_MAX_TICKS      900
#define NAV_BACK_CENTER_GRACE   90   // 已废弃（原两步停车的兜底），保留仅为兼容

//   靶位段（圆心→靶位 780mm，或绿的 A→大圆 ≈390mm）
//   非绿沿途：斜线(≈100mm 短) → 内圆(≈277mm 长) → 大圆(≈396mm 长)
//             → 数到第 2 条「长间隔线」就是大圆
//   绿  沿途：已过内圆，只有大圆一条 → 数到第 1 条就是大圆
#define NAV_TARGET_MIN_GAP        176  // 长间隔线门槛（190mm≈154拍，介于 100/277mm 之间）
#define NAV_TARGET_LONG_NEED      2    // 非绿：第 2 条长间隔线 = 大圆
#define NAV_TARGET_MAX_TICKS      1400 // 非绿绝对兜底（圆心→大圆 780mm≈1083拍）
#define NAV_TARGET_GREEN_MAX_TICKS 800  // 绿兜底（A→大圆 ≈390mm≈547拍）

//   靶位主判据改用【前灰度】——侧灰度装在车中心，它跨大圆时料已经冲出靶心
//   ≈12cm（料在车头），所以侧灰度的到点本身就是"偏外"的。
//
//   ★ 靶位特征（用户口述地图，不是全黑！）：
//     靶位中心是一个【对应颜色的实心圆盘】，直径 92mm，正好压在辐射线与大圆的
//     交点上。除了黑料靶，红/绿/蓝/白圆盘对红外都判成【白】——
//     所以前灰度跨过大圆时看到的不是"全黑"，而是：
//         中间变白（彩盘盖住了黑线） + 两侧仍然是黑（大圆那条黑线）
//     黑料靶例外：圆盘本身是黑的，和黑线连成一片 → 中间区域整段黑。
//
//   ⚠ 同样的特征在 A/B/C/D/E 五点的【白圆】处也会出现（它们也在内圆上），
//     所以用两道门区分（见下面的宏和 nav.c 的 Nav_Target_Front_Check）：
//      门1 硬性门：DBG_NAV_SERVO==1（夹着料才可能放料）。放完料张开、
//                  空车回来经过小圆时这道门自动关死，标志位不会被污染。
//      门2 位置门：侧灰度已跨过内圆（第 1 条长间隔线）。绿靶不需要，
//                  因为从 A 出发时已经在内圆外面。
//   NAV_TARGET_ARM_TICKS：先连续这么多拍"看到线"才允许判特征，
//     防止刚出圆心时阵列还压在圆心黑区里被误判。
//   NAV_TARGET_BLACK_K  ：特征连续这么多拍才算一次（消抖）。
#define NAV_TARGET_ARM_TICKS 20  // 30×0.6
#define NAV_TARGET_BLACK_K   2

//   ★2026-10-29 彩盘前缘提前触发（治「送料略微过冲」）
//   老判据要「两侧黑」，必须等阵列压到大圆黑线才成立；而料在阵列前方 31mm，
//   所以一触发就已经过冲 ≈31mm（开发记录 §20）。彩盘 Ø92，中间 4 个探头跨 ≈33mm：
//   车到盘心前 √(46²−16.5²)≈43mm 时中间区就被彩盘盖满 → 比「压线」早 ≈43mm，
//   料从「过冲 31mm」变成「欠 12mm」，扣消抖+滑行后基本落在靶心。
//   物理依据：沿射线走时中间区一直压着辐射线（必然有黑），只有进彩盘才全白。
//   黑料靶不适用（黑盘与黑线分不开），仍走老判据。
#define NAV_TARGET_EDGE_K         2    // 中间区全白连续拍数（消抖）
#define NAV_TARGET_EDGE_BLACK_MEM 15   // 中间区「见过黑」的记忆窗（拍），防丢线时全白误触发
#define NAV_TARGET_EDGE_ARM_TICKS 280  // 绿靶：段内走满这么多拍才允许前缘触发
                                       // （≈315mm；彩盘前缘在 344mm≈306 拍，留 26 拍余量）
                                       // 非绿靶：内圆之后又走满一个内圆间隔（sg_gap≈258 拍≈290mm）

//   前灰度字节位定义（黑=1、白=0；探头1=bit7 … 探头8=bit0）
//   判据放宽：不要求"正中间两个探头白"，只要
//     ① 中间区域（探头3~6）至少 2 个是白 —— 车偏了也能命中
//     ② 最左（探头1~2）、最右（探头7~8）各至少一个黑
//   宁可靠两道门（夹着料 + 已过内圆）挡误触发，不靠卡死像素位置。
#define NAV_FR_LEFT_BLACK   0xC0u // 探头1~2 至少一个黑
#define NAV_FR_MID_WHITE    0x3Cu // 中间区域：探头3~6
#define NAV_FR_RIGHT_BLACK  0x03u // 探头7~8 至少一个黑
#define NAV_FR_MID_MIN_WHITE 2u   // 中间区域至少几个白才算命中（1=更松，3=更严）

//   到点后的对位微调（实车标定）：正=前进、负=后退，单位 ms @速度 12（≈65mm/s）
//   料落在靶心外侧就往负调，落在内侧就往正调。
//   ★ 实测：前灰度阵列到物料圆心 ≈31mm，夹具在车头前方 → 料在阵列前方，
//     触发瞬间料已冲出靶心 31mm → 补偿量取【负】。
//   ★2026-10-29 这条就是"退后再开夹爪"的那个后退，默认给足一档：
//     -450 实测已明显改善，但仍差 1~1.5cm → 再加 200ms（≈13mm）→ -650。
//     （按 450ms ≈ 29mm 的实测比例折算：1cm ≈ 154ms）
//     注意它发生在 Steering_SET() 之前 —— 只有这之前的后退才能改变料的落点；
//     张开之后那次后退（NAV_BACKOFF_MS）只负责把夹爪从料上抽走，不影响落点。
#define NAV_TARGET_FINE_MS  (-650)
#define NAV_STORE_FINE_MS    0    // 暂存点（B/D 内圆）放料的微调，一般 0

// ----------------------------------------------------------------------------
// 回家段（圆心 → 出发区，沿 270°）
//   沿途：斜线(≈100mm) → 内圆(390) → 出发圆北缘(628) → 大圆(780)
//   数第 4 条线就停在大圆上（=出发区中心）；少一条时停在第 3 条（出发圆边缘），
//   车心距出发区中心 152mm < D320 半径 160mm，仍然在区内，保住返回分。
// ----------------------------------------------------------------------------
//   2026-10-27 改：不再数「第 4 条任意线」。
//   原因：倒退回圆心实际停在 r≈100mm（签名在斜线处触发），回家起步位置每次不同，
//   侧探头在起步瞬间可能压住「过圆心的东西主轴线」，压住就被 REARM 跳过、
//   没压住就多算一条 → 数出来 4 条或 5 条，停在大圆或出发圆，位置不唯一。
//   改为「长间隔线」模式，跳过两条 100mm 短线，只认同心圆：
//     斜线(100mm≈93拍,跳过) → 内圆(290mm≈271拍,1) → 出发圆(238mm≈222拍,2)
//     → 大圆(152mm≈141拍,3) 停
//   同心圆与车头朝向无关，起步偏了也不影响间隔。
#define NAV_HOME_ANGLE        270
#define NAV_HOME_NEED         3    // 长间隔线：内圆、出发圆、大圆
#define NAV_HOME_MIN_GAP      117  // 126mm：> 斜线81拍、< 大圆段124拍，两侧各留 ≈20%

//   ★2026-10-29 回家防数错（@18 速、拍距 1.124mm/拍）
//   根因：回家起点是圆心，倒退停车有 ±d 余量。若停在圆心前 50mm，第一条斜线
//   距离变成 150mm≈133 拍 > NAV_HOME_MIN_GAP=117 → 被误认成第 1 条长线，
//   于是「内圆=2、出发圆=3」→ 在出发圆就停，比大圆早 150mm（回不到起点）。
//   本段里程：内圆 390mm(347拍) → 出发圆 628mm(559拍) → 大圆 780mm(694拍+爬坡≈700)
#define NAV_HOME_FIRST_MIN_TICKS 280  // 第 1 条长线：间隔和总拍数都小的才丢弃（组合门）
                                      // 正常：斜线 gap≈89 拍/seg≈89 拍，两个都小 → 丢弃
                                      //       内圆 gap≈258 拍 → 只这一个就够，保留
                                      // 起点偏前 100mm：斜线 gap≈178/seg≈178 → 丢弃；内圆 gap≈258 → 保留
#define NAV_HOME_FIRST_MIN_GAP   200  // 与上面组合使用：gap<200 且 seg<280 才丢
#define NAV_HOME_MIN_TOTAL_TICKS 620  // 第 3 条线（大圆）必须 ≥ 此拍数（≈697mm）
                                      // 出发圆 559 拍 < 620 < 大圆 ≈700 拍
#define NAV_HOME_MAX_TICKS    800     // 兜底 ≈900mm（原 2600，会冲出场地）
                                      // 仍在出发区 Ø320 内（R≤940）；漏线时也保返回分

// 从大圆（靶位）倒退回圆心：780mm，车速降到 30mm/s 时约 2260 拍
#define NAV_BACK_BIG_MAX_TICKS 2600

// ----------------------------------------------------------------------------
// 夹爪（舵机 TIM9_CH1 / PE5，驱动见 APP/steering）
//   物料在 A 点，是车把它送进夹爪口的：出发前必须张开，停车后闭合即夹住。
//   张开 1.98ms ≈133°，夹紧 1.32ms ≈74°，全行程 59° 约需 0.6s。
//   NAV_SERVO_OPEN_MS  出发前等夹爪张开到位（不等的话夹爪还在摆车就冲出去了）
//   NAV_CLAMP_DELAY_MS A 点停车后先停稳再夹（避免车还在滑、物料相对夹爪还在动）
// ----------------------------------------------------------------------------
#define NAV_SERVO_OPEN_MS  700   // ★2026-10-29 600→700：张开量又加了 10°，
                                 //   舵机全行程 59°≈600ms，多 10° 约多 100ms
#define NAV_CLAMP_DELAY_MS 200   // ★2026-10-29 300→200（减少停顿）

// ==================== 导航状态（Cortex-Debug Watch 可直接看 Nav） ====================
typedef struct
{
    volatile int32_t  line_count; // 本段累计跨线数
    volatile uint16_t gap;        // 最近一条线距上一条线的拍数
    volatile uint16_t prev_gap;   // 上上次间隔（拍）
    volatile uint16_t seg_ticks;  // 本段已走拍数（10ms/拍）
} NavState;

extern NavState Nav;

// ==================== 初始化 / 更新 / 路径 ====================
void Nav_Init(void);       // 清调试量与路径状态（不动侧灰度计数器）
void Nav_Update(void);     // 每控制周期调用：侧灰度边沿状态机 + 拍数累计
void Nav_Task2_Start(void); // 启动第一题：出发→圆心→左转90°→A点
void Nav_Task2_Run(void);   // 第一题每控制周期推进

// 当前路径阶段（Watch 里看 nav_phase）：
//   0=IDLE 1=TO_CENTER 2=TURN_NAV(通用转向) 3=TO_POINT(通用去内圆点) 4=DONE 5=ABORT
//   6=CLAMP_READ(夹住+读色) 7=BACK_TO_CENTER 8=TURN_TO_TARGET 9=TO_TARGET 10=PLACE
//   11=DECIDE(查账本选下一步) 12=BACK_FROM_TARGET(大圆倒退) 13=TURN_TO_HOME 14=TO_HOME
uint8_t Nav_GetPhase(void);

// ==================== 开环转向标定（main.c 的 Turn_Calib_Test 用） ====================
// 标定程序里生效，正式导航仍用 NAV_TURN_T0_MS / NAV_TURN_RATE_CENTI 两个宏。
extern volatile int16_t  DBG_TURN_DEG;     // 目标角度：正=右转、负=左转（Watch 里改）
extern volatile uint16_t DBG_TURN_T0;      // 起步滞后 ms
extern volatile uint16_t DBG_TURN_RATE;    // 角速度，0.01°/ms（14 = 0.14°/ms）
extern volatile uint16_t DBG_TURN_LAST_MS; // 上一次实际延时（核对用）
extern volatile uint16_t DBG_TURN_MS_OVERRIDE; // 非 0 = 直接按此 ms 转，忽略 T0/RATE
extern volatile uint16_t DBG_TURN_COUNT;   // 累计转动次数
void Nav_Calib_SpinDeg(void);              // 按 DBG_TURN_DEG 转一次

// ==================== 无串口调试变量（Cortex-Debug Watch） ====================
extern volatile int16_t  DBG_NAV_POS;        // 前灰度位置（trace.c 的 DBG_pos 镜像）
extern volatile uint8_t  DBG_NAV_SGW;        // 侧灰度原始读数（1=黑 0=白）
extern volatile uint8_t  DBG_NAV_STATE;      // 侧灰度状态机：0=等沿 1=消抖 2=重武装
extern volatile uint16_t DBG_NAV_LINES;      // 本段跨线数
extern volatile uint16_t DBG_NAV_GAP;        // 最近一次间隔（拍）
extern volatile uint16_t DBG_NAV_PREVGAP;    // 上一次间隔（拍）
extern volatile uint16_t DBG_NAV_SEG_TICKS;  // 本段已走拍数
extern volatile uint16_t DBG_NAV_CAP;        // 当前段的自适应拍数上限（超过即 ABORT）
extern volatile uint8_t  DBG_NAV_CENTER_REASON; // 圆心判定原因：1=5条+签名 2=签名兜底 3=第6条 9=拍数上限
extern volatile uint8_t  DBG_NAV_A_REASON;      // A 点判定原因：1=长间隔线 9=拍数上限
extern volatile uint8_t  DBG_NAV_TURN_DONE;     // 左转已完成
extern volatile int16_t  DBG_NAV_TURN_POS;      // 转完后前灰度位置（≈35=抓到 0°线；-1=偏了；-2=黑芯）
extern volatile uint8_t  DBG_NAV_SERVO;         // 夹爪/物料状态：0=张开（空车） 1=夹住（有料）
                                                // ★靶位判据的硬性门：只有=1 才允许认靶位
extern volatile uint8_t  DBG_NAV_COLOR;         // 判色结果（1白 2黑 3红 4绿 5蓝，0=读不出来）
extern volatile uint8_t  DBG_NAV_BACK_REASON;   // 回圆心停因：2=签名成立停（正常） 9=拍数上限
extern volatile uint16_t DBG_NAV_BACK_SIG_GAP;  // 签名段的间隔拍数：≈89 拍(100mm) 说明正常跨到圆心
// 最近一次转向留档（区分「指令角度错」还是「执行不到位」）
extern volatile uint8_t  DBG_NAV_HEAD_BEFORE;   // 转向前 sg_heading
extern volatile uint8_t  DBG_NAV_TURN_TGT;      // 目标角
extern volatile int16_t  DBG_NAV_TURN_DEG;      // 实际下发转角（带符号，+=右转）
extern volatile uint16_t DBG_NAV_TURN_MS;       // 旋转延时 ms
extern volatile int16_t  DBG_NAV_FINE_MS;       // 靶位对位微调（ms@NAV_FINE_V，正=前进 负=后退）
                                                //   ★Watch 里直接改，按 KEY0 重新开局即生效
extern volatile int16_t  DBG_NAV_FINE_APPLIED;  // 实际生效值（≠ DBG_NAV_FINE_MS 说明没设进去）
extern volatile int16_t  DBG_NAV_BACK_MS;       // 放料后退后时长（ms@NAV_INCH_V，正=后退）
                                                //   ★Watch 里直接改，下一次放料即生效，不用重开局
                                                //   目标目视后退 2~3cm：约 310~460ms
extern volatile int16_t  DBG_NAV_BRAKE_MS;      // 靶位到点反向刹车脉宽（ms@NAV_BRAKE_V，限 0~400）
                                                //   ★Watch 里直接改，下一次到靶位即生效
                                                //   不够灵敏（还冲过）→ 加大；刹过头（提前停）→ 减小
// 整局转向流水：按顺序记录每次转向，d = TGT − HEAD（最短路径）
//   Watch 里看 NAV_TURN_LOG_NUM，再展开三个数组逐个核对
//   MS 核对：45°→447、90°→545、135°→643、180°→741
#define NAV_TURN_LOG_N  16
extern volatile uint8_t  NAV_TURN_LOG_NUM;
extern volatile uint8_t  NAV_TURN_LOG_HEAD[NAV_TURN_LOG_N];
extern volatile uint8_t  NAV_TURN_LOG_TGT[NAV_TURN_LOG_N];
extern volatile uint16_t NAV_TURN_LOG_MS[NAV_TURN_LOG_N];
extern volatile uint8_t  DBG_NAV_PLACE_REASON;  // 投料停因：1=前灰度压线(老判据) 2=侧灰度兜底
                                                //            3=彩盘前缘(新) 9=拍数上限
// 回家段黑匣子：把每一条确认线的间隔记下来，跑完 attach 看 NAV_HOME_GAP[0..NAV_HOME_N-1]
//   正常（从圆心起步）：≈89(斜线,不算长线) → 258(内圆) → 212(出发圆) → 135(大圆)
extern volatile uint8_t  NAV_HOME_N;
extern volatile uint16_t NAV_HOME_GAP[6];

// ==================== 黑匣子（attach 读 RAM 回放） ====================
// 每拍环形记录最近 NAV_LOG_N 拍；停车瞬间锁存 NAV_STOP_* 快照。
#define NAV_LOG_N 128

// ----------------------------------------------------------------------------
// 相位迁移黑匣子：每次进入新相位记一条（NAV_LOG_* 只留最近 128 拍 ≈1.3s，整局查不了）
//   跑完 attach 后按 0..NAV_TRC_NUM-1 顺序看，就能还原整局的流程走向。
//   NAV_TRC_PHASE  进入的相位（见 nav_phase 表）
//   NAV_TRC_HEAD   进入时的朝向估计 sg_heading（0°=西 90°=北 180°=东 270°=南）
//   NAV_TRC_AIM    该步的目标角（去点取料 = pick_angle，转靶位 = target_angle）
//   典型判读：「该去 A 却直接回家」→ 若记录的是 PATH_TURN_TO_HOME(13)，
//             说明 Nav_Plan_Pick 返回了 0（账本认为三块都取完），是账本错了。
// ----------------------------------------------------------------------------
#define NAV_TRC_N 40
extern volatile uint8_t NAV_TRC_NUM;
extern volatile uint8_t NAV_TRC_PHASE[NAV_TRC_N];
extern volatile uint8_t NAV_TRC_HEAD[NAV_TRC_N];
extern volatile uint8_t NAV_TRC_AIM[NAV_TRC_N];
//   NAV_TRC_LED    当时账本的打包快照（查「为什么它决定回家」的关键）：
//     bit0~1  sg_picked（已取几块，0~3）
//     bit2    taken[0]  A 已取     bit3  taken[1]  C 已取     bit4  taken[2]  E 已取
//     bit5    stage[0] 非空（白料暂存在 B）  bit6  stage[1] 非空（黑料暂存在 D）
//     bit7    sg_claw 非 0（手上夹着料）
//   判读：「料没投完却 TURN_TO_HOME(13)」→ 看 LED，若 picked=3 或 taken 全 1，就是账本错了。
extern volatile uint8_t NAV_TRC_LED[NAV_TRC_N];
extern volatile uint8_t  NAV_LOG_I;                    // 下一个写入位置
extern volatile int16_t  NAV_LOG_POS[NAV_LOG_N];       // 前灰度位置
extern volatile uint16_t NAV_LOG_LINES[NAV_LOG_N];     // 跨线数
extern volatile uint16_t NAV_LOG_GAP[NAV_LOG_N];       // 最近间隔（拍）
extern volatile uint16_t NAV_LOG_PREVGAP[NAV_LOG_N];   // 上次间隔（拍）
extern volatile uint16_t NAV_LOG_TICKS[NAV_LOG_N];     // 本段累计拍数
extern volatile uint8_t  NAV_LOG_PHASE[NAV_LOG_N];     // 路径阶段

extern volatile uint8_t  NAV_STOP_SNAPPED;   // 1=已锁存停车快照
extern volatile int16_t  NAV_STOP_POS;       // 停车瞬间：前灰度位置
extern volatile uint16_t NAV_STOP_LINES;     // 停车瞬间：跨线数
extern volatile uint16_t NAV_STOP_GAP;       // 停车瞬间：最近间隔
extern volatile uint16_t NAV_STOP_PREVGAP;   // 停车瞬间：上次间隔
extern volatile uint8_t  NAV_STOP_PHASE;     // 停车瞬间：路径阶段

// 1=「真丢线」停车时执行 BKPT 让 CPU 自停（nav 已改用 Trace_Forward 丢线回找，
// 不再有该分支；保留开关供将来调试）
#define NAV_STOP_BREAK 0

#endif
