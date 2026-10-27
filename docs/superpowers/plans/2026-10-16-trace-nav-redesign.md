# 循迹 + 导航子系统重构 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重写循迹（线型状态机 + 档位查表）与导航（里程定位 + 侧方数线门控），使差速小车从出发区稳定、准确地到达 A/C/E 物料点。

**Architecture:** 循迹层把 8bit 二值灰度用「边缘法」翻译成 7 档 `LineState`，再查表输出固定差速（彻底去掉连续 PD）；丢线用里程有界锯齿回摆，全黑用里程累计置「疑似圆心」标志。导航层以编码器里程为主做距离定位，侧方灰度数线只在里程预判窗口内做路标校正（只读不控），前灰度全黑/全白做节点确认，路径状态机「出发→圆心→转向→物料点」。

**Tech Stack:** C（C99）+ STM32F407 标准外设库 + EIDE/AC5（armcc，`D:\keil5\ARM\ARMCC\bin\armcc.exe`）+ 差速底盘（板载闭环电机，写 int8 速度寄存器）。host 单测用 D 盘 MinGW64 的 `gcc`。

**Spec:** [docs/superpowers/specs/2026-10-16-trace-nav-redesign.md](../specs/2026-10-16-trace-nav-redesign.md)

## Global Constraints

- 只能用现有硬件：前/后 8 路二值灰度（I2C 地址 0x12 读寄存器 0x30）+ 侧方 1 路（PD10）+ 编码器 ×4（读 0x3C）+ TCS3200 + SG90。**不加陀螺仪、不换模拟量灰度**。
- 差速底盘无横移、无 MPU；控制周期 10ms（`delay_ms(10)`）。
- 电机速度寄存器 int8（−127~127）；差速限幅必须满足 `U_WEAK < U_STRONG < LINE_SPEED`（保证单侧轮不反向）。
- 侧方灰度**只读不控**：绝不进入循迹差速计算；里程未预判到路标时不接受数线触发（门控）。
- 探头索引约定：`bit0 = 车左 … bit7 = 车右`（传感器反装已修复）；`u>0 = 左转`（线偏左）。
- 项目**非 git 仓库**：每个 Task 的「提交」步骤以「编译验证」或「实车验证」替代；如需版本管理请用户另行 `git init`。
- `color.h` 定义 `white/black/red/green/blue` 全局宏，代码中不得用这些单词做变量名。

## Review Focus

1. 线型分类的档位边界（`center=2.0/3.5/4.5/6.0` 归属）与全白/全黑极端输入 → Task 1 的 host 单测逐项断言。
2. 丢线回摆方向（线偏左应左摆）与超界停车（真丢线不冲）→ Task 2 实车验证步骤。
3. 转向符号（`u>0=左转`，线偏左时车左转）→ Task 2 实车首测 LED 确认；反了则整体取反 `U_TABLE`。
4. 圆心判定不误判普通路口（里程 + 全黑双条件）→ Task 4 实车验证步骤。
5. 侧方数线门控（里程窗口外触发被忽略，不干扰导航）→ Task 4 实车验证步骤。

---

## Task 0: 修复构建与硬件接线基线

**Files:**
- Modify: `.eide/eide.yml:37-50`（APP 虚拟目录的 `files` 列表）
- Modify: `APP/sensor/S_GREY_driver.c`（侧方灰度引脚 PD10 → PD12）
- Modify: `APP/sensor/S_GREY_driver.h`（注释里的 PD10 → PD12）

**Interfaces:**
- Produces: 一个能 Clean+Rebuild 通过、且侧方灰度读到真实引脚的工程基线（后续所有 Task 的编译与数线验证都依赖它）。

- [ ] **Step 1: 修正侧方灰度引脚 PD10 → PD12**

`S_GREY_driver.c` 的 `S_GW_Init()` 中 `GPIO_Pin_10` 改为 `GPIO_Pin_12`：

```c
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
```

`S_GW_Read()` 中同样改：

```c
    return (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12) == Bit_SET) ? 1u : 0u;
```

并把 `S_GREY_driver.h` 顶部注释里的 `PD10` 改为 `PD12`。（侧方灰度初始化链路已在 `main() → GREY_Init() → S_GW_Init()` 中，无需新增初始化；此步只改引脚。）

- [ ] **Step 2: 修正 APP 源文件列表**

在 `.eide/eide.yml` 的 `APP` 虚拟目录 `files` 里：删除 `APP/sensor/L_GREY_driver.c` 和 `APP/sensor/R_GREY_driver.c` 两行，新增 `APP/sensor/S_GREY_driver.c`、`APP/sensor/grey_i2c.c`、`APP/key/key.c`、`APP/nav/nav.c`、`APP/steering/steering.c` 五行。改后 `APP` 的 files 应为：

```yaml
    - name: APP
      files:
        - path: APP/led/led.c
        - path: APP/usart/usart.c
        - path: APP/motor/motor.c
        - path: APP/color/color.c
        - path: APP/sensor/sensor.c
        - path: APP/sensor/F_GREY_driver.c
        - path: APP/sensor/B_GREY_driver.c
        - path: APP/sensor/S_GREY_driver.c
        - path: APP/sensor/grey_i2c.c
        - path: APP/trace/trace.c
        - path: APP/VOFA/VOFA.c
        - path: APP/key/key.c
        - path: APP/nav/nav.c
        - path: APP/steering/steering.c
      folders: []
```

- [ ] **Step 3: 编译验证**

在 VSCode 中执行 EIDE「Rebuild」（或命令面板 `EIDE: Rebuild`）。
Expected: 编译通过，0 error；`build/` 下生成新的 `.axf`/`.hex`。若报错，检查是否还有残留的 `L_GREY_driver.c` 引用或缺失的 incList 目录（`APP/key`、`APP/nav`、`APP/steering` 已在 incList 中，无需改）。

---

## Task 1: 线型分类（GREY_classify）

**Files:**
- Create: `APP/sensor/GREY_classify.h`
- Modify: `APP/sensor/GREY_driver.h`（include + 声明）
- Modify: `APP/sensor/F_GREY_driver.c`（实现 `F_GW_classify`）
- Modify: `APP/sensor/B_GREY_driver.c`（实现 `B_GW_classify`）
- Test: `D:\temp\grey_test\test_grey_classify.c`（host 单测，用完可删）

**Interfaces:**
- Produces:
  - `LineState` 枚举：`LINE_LOST=0, LINE_LEFT2, LINE_LEFT1, LINE_CENTER, LINE_RIGHT1, LINE_RIGHT2, LINE_JUNCTION`
  - `LineState Grey_Classify(uint8_t data)`（static inline，纯函数）
  - `LineState F_GW_classify(void)` / `B_GW_classify(void)`（读灰度并分类）

- [ ] **Step 1: 新建 `GREY_classify.h`（纯逻辑，只依赖 stdint.h，不依赖 main.h，便于 host 单测）**

```c
#ifndef _GREY_classify_H
#define _GREY_classify_H

#include <stdint.h>

// 线型档位：二值灰度 8bit → 7 档离散状态
typedef enum {
    LINE_LOST = 0,   // 全白丢线
    LINE_LEFT2,      // 线偏左 2 档
    LINE_LEFT1,      // 线偏左 1 档
    LINE_CENTER,     // 线居中（死区）
    LINE_RIGHT1,     // 线偏右 1 档
    LINE_RIGHT2,     // 线偏右 2 档
    LINE_JUNCTION,   // 全黑（路口/圆心/宽线）
} LineState;

// data: 8bit 灰度（黑=1 白=0），bit0=车左 … bit7=车右
// 边缘法：取最左/最右黑探头的中点映射到 5 档 + 全白 + 全黑
static inline LineState Grey_Classify(uint8_t data)
{
    int left = -1, right = -1, count = 0;
    int i;
    for (i = 0; i < 8; i++) {
        if ((data >> i) & 0x01) {
            count++;
            if (left < 0) left = i;
            right = i;
        }
    }
    if (count == 0) return LINE_LOST;
    if (count == 8) return LINE_JUNCTION;

    float center = (left + right) / 2.0f;
    if (center < 2.0f)  return LINE_LEFT2;
    if (center < 3.5f)  return LINE_LEFT1;
    if (center <= 4.5f) return LINE_CENTER;
    if (center <= 6.0f) return LINE_RIGHT1;
    return LINE_RIGHT2;
}

#endif
```

- [ ] **Step 2: 在 `GREY_driver.h` 引入分类接口**

在 `GREY_driver.h` 顶部（`#include "main.h"` 之后）加：

```c
#include "GREY_classify.h"
```

并在文件末尾的接口声明区加：

```c
LineState F_GW_classify(void);
LineState B_GW_classify(void);
```

- [ ] **Step 3: 实现 `F_GW_classify` / `B_GW_classify`**

`F_GREY_driver.c` 末尾加：

```c
LineState F_GW_classify(void)
{
    return Grey_Classify(F_GW_digital_Read());
}
```

`B_GREY_driver.c` 末尾加：

```c
LineState B_GW_classify(void)
{
    return Grey_Classify(B_GW_digital_Read());
}
```

- [ ] **Step 4: 写 host 单测**

新建 `D:\temp\grey_test\test_grey_classify.c`：

```c
#include <stdio.h>
#include <stdint.h>
#include "GREY_classify.h"

int main(void)
{
    int fail = 0;
    if (Grey_Classify(0x00) != LINE_LOST)     { printf("FAIL 0x00 all-white\n"); fail++; }
    if (Grey_Classify(0xFF) != LINE_JUNCTION) { printf("FAIL 0xFF all-black\n"); fail++; }

    LineState expect_single[8] = { LINE_LEFT2, LINE_LEFT2, LINE_LEFT1, LINE_LEFT1,
                                   LINE_CENTER, LINE_RIGHT1, LINE_RIGHT1, LINE_RIGHT2 };
    int i;
    for (i = 0; i < 8; i++) {
        LineState got = Grey_Classify((uint8_t)(1u << i));
        if (got != expect_single[i]) {
            printf("FAIL single bit %d: got %d expect %d\n", i, got, expect_single[i]);
            fail++;
        }
    }

    LineState expect_pair[7] = { LINE_LEFT2, LINE_LEFT2, LINE_LEFT1, LINE_CENTER,
                                 LINE_CENTER, LINE_RIGHT1, LINE_RIGHT2 };
    for (i = 0; i < 7; i++) {
        uint8_t d = (uint8_t)((1u << i) | (1u << (i + 1)));
        LineState got = Grey_Classify(d);
        if (got != expect_pair[i]) {
            printf("FAIL pair %d-%d: got %d expect %d\n", i, i + 1, got, expect_pair[i]);
            fail++;
        }
    }

    if (fail == 0) printf("ALL PASS\n");
    else           printf("%d FAILURES\n", fail);
    return fail ? 1 : 0;
}
```

- [ ] **Step 5: 运行 host 单测**

Run: `gcc -I "D:/gongchengsai/code/APP/sensor" "D:/temp/grey_test/test_grey_classify.c" -o "D:/temp/grey_test/test_grey_classify.exe" && "D:/temp/grey_test/test_grey_classify.exe"`
Expected: 输出 `ALL PASS`，退出码 0。若某边界失败，检查 `center` 映射区间与期望表是否一致（本测试即「Review Focus 1」的锚点）。

- [ ] **Step 6: 编译验证 + 清理测试临时目录**

在 EIDE 中 Rebuild。
Expected: 编译通过。然后删除 `D:\temp\grey_test\` 目录（临时产物按 CLAUDE.md 清理）。

---

## Task 2: 循迹层重写（档位查表 + 丢线锯齿 + 全黑标志）

**Files:**
- Modify: `APP/trace/trace.h`（参数 + `TraceState` + 接口声明）
- Modify: `APP/trace/trace.c`（重写循迹逻辑；保留 `Diff_Drive`/`Spin`/`Spin_Angle`/`Stop`/`Encoder_*` 不动）
- Modify: `User/main.c`（KEY0 改为新循迹测试入口）

**Interfaces:**
- Consumes: `F_GW_classify()`（Task 1）、`Encoder_Odometer()`（trace.c 已有）、`Diff_Drive`/`Stop`（trace.c 已有）
- Produces:
  - `TraceState` 枚举：`TRACE_FOLLOWING, TRACE_LOST, TRACE_JUNCTION, TRACE_LOST_FAIL`
  - `void Trace_Init(void)`
  - `TraceState Trace_LineFollow(int16_t V)`（循迹一步，返回状态）
  - `uint8_t Trace_CenterHit(void)`（全黑里程超阈值，疑似圆心）

- [ ] **Step 1: 在 `trace.h` 加入参数与接口声明**

在 `trace.h` 的「巡线参数」区替换为（保留 `Spin_Angle` 相关参数不动，删除 `KP_LINE/KD_LINE/FILT_ALPHA`）：

```c
#define LINE_SPEED    8    // 循迹基准速度
#define U_WEAK        3    // 小纠偏差速
#define U_STRONG      5    // 大纠偏差速（必须 < LINE_SPEED）
#define SEARCH_V      4    // 丢线慢速前进基准
#define SEARCH_SWING  3    // 丢线摆动差速
#define SEARCH_SWING_DIST 20  // 每摆多少编码器脉冲换向一次
#define SEARCH_BOUND  80      // 丢线回摆总里程界（超过即真丢线）
#define DIST_CENTER_BLACK 60  // 圆心全黑持续里程阈值（脉冲，待实车标定）
```

并在接口区加：

```c
typedef enum {
    TRACE_FOLLOWING = 0,
    TRACE_LOST,
    TRACE_JUNCTION,
    TRACE_LOST_FAIL,
} TraceState;

void Trace_Init(void);
TraceState Trace_LineFollow(int16_t V);
uint8_t Trace_CenterHit(void);
```

- [ ] **Step 2: 在 `trace.c` 加入档位表与循迹状态机**

**保留旧的 `Line_Follow` 函数体不动**（`Trace_Backward` 的向后循迹仍用它，本期不重做向后）。在 `trace.c` 顶部（`e_last_f/e_last_b` 之后）**新增**以下内容：

```c
// ---- 循迹状态机（档位查表 + 丢线锯齿 + 全黑标志） ----
static const int16_t U_TABLE[7] = {
    [LINE_LOST]     = 0,
    [LINE_LEFT2]    = +U_STRONG,
    [LINE_LEFT1]    = +U_WEAK,
    [LINE_CENTER]   = 0,
    [LINE_RIGHT1]   = -U_WEAK,
    [LINE_RIGHT2]   = -U_STRONG,
    [LINE_JUNCTION] = 0,
};

static LineState  g_last_line   = LINE_CENTER;
static TraceState g_trace_state = TRACE_FOLLOWING;
static int32_t    g_lost_start  = 0;   // 丢线总起点里程
static int32_t    g_swing_odom  = 0;   // 当前摆动段起点
static int8_t     g_search_dir  = 1;
static uint8_t    g_in_black    = 0;
static int32_t    g_black_odom  = 0;

void Trace_Init(void)
{
    g_last_line   = LINE_CENTER;
    g_trace_state = TRACE_FOLLOWING;
    g_lost_start  = 0;
    g_swing_odom  = 0;
    g_search_dir  = 1;
    g_in_black    = 0;
    g_black_odom  = 0;
}

TraceState Trace_LineFollow(int16_t V)
{
    LineState st = F_GW_classify();
    int32_t odom;
    int16_t u;

    if (st == LINE_LOST) {
        if (g_trace_state != TRACE_LOST) {
            g_trace_state = TRACE_LOST;
            g_lost_start = Encoder_Odometer();
            g_swing_odom = g_lost_start;
            g_search_dir = (g_last_line <= LINE_LEFT1) ? 1 : -1;  // 线偏左→左摆
        }
        odom = Encoder_Odometer();
        if ((odom > g_lost_start ? odom - g_lost_start : g_lost_start - odom) > SEARCH_BOUND) {
            g_trace_state = TRACE_LOST_FAIL;
            Stop();
            return TRACE_LOST_FAIL;
        }
        if ((odom > g_swing_odom ? odom - g_swing_odom : g_swing_odom - odom) > SEARCH_SWING_DIST) {
            g_search_dir = -g_search_dir;   // 锯齿换向
            g_swing_odom = odom;
        }
        Diff_Drive(SEARCH_V - g_search_dir * SEARCH_SWING,
                   SEARCH_V + g_search_dir * SEARCH_SWING);
        return TRACE_LOST;
    }

    if (st == LINE_JUNCTION) {
        if (!g_in_black) { g_in_black = 1; g_black_odom = Encoder_Odometer(); }
        g_trace_state = TRACE_FOLLOWING;
        Diff_Drive(V, V);
        return TRACE_JUNCTION;
    }

    // 正常循迹：档位查表
    g_in_black = 0;
    g_last_line = st;
    g_trace_state = TRACE_FOLLOWING;
    u = U_TABLE[st];
    Diff_Drive(V - u, V + u);
    return TRACE_FOLLOWING;
}

uint8_t Trace_CenterHit(void)
{
    int32_t d;
    if (!g_in_black) return 0;
    d = Encoder_Odometer() - g_black_odom;
    return (d > DIST_CENTER_BLACK) ? 1 : 0;
}
```

保留 `Trace_Forward`/`Trace_Backward` 两个无参封装，把它们改为调用新循迹（`Trace_Backward` 本期可仍指向旧 `Line_Follow(-V)`，或先置空，注明后续再做向后）：

```c
void Trace_Forward(void)  { Trace_LineFollow(LINE_SPEED); }
void Trace_Backward(void) { Line_Follow(-LINE_SPEED); }  // 向后循迹后续再做
```

- [ ] **Step 3: 修改 main.c 的 KEY0 测试入口**

将 `main.c` 的 `Trace_Forward_Test()` 改为调用新接口，并在主循环里用 `Trace_Init()` 初始化：

```c
static void Trace_Forward_Test(void)
{
    Trace_Init();
    while (1) {
        Trace_LineFollow(LINE_SPEED);
        delay_ms(10);
    }
}
```

在 `main()` 的 `Nav_Init()` 之后、`Stop()` 之前加 `Trace_Init();`（与 `Nav_Init` 同级，Task 4 会用到）。

- [ ] **Step 4: 编译验证**

EIDE Rebuild。Expected: 0 error。若报 `LineState`/`TraceState` 未定义，检查 include 链（`trace.h` → `main.h` → `GREY_driver.h` → `GREY_classify.h`）。

- [ ] **Step 5: 实车验证（Review Focus 2、3 的锚点）**

烧录后按下 KEY0，把车放黑线上，观察：
1. 直线段：车沿黑线走，不蛇形、不冲出（`LINE_SPEED=8` 偏慢是预期，先求稳）。
2. 方向确认：把车头故意偏到线左侧 → 车应**左转**回线；偏右 → 右转。若方向相反，把 `U_TABLE` 里 `LINE_LEFT*` 与 `LINE_RIGHT*` 的符号整体对调（`U_STRONG/U_WEAK` 前取反）。
3. 丢线恢复：人为把车推出线 → 车应慢速前进 + 左右摆找回线，而非原地乱旋或直线冲。
4. 真丢线：把车放到无线的白纸中央 → 回摆超过 `SEARCH_BOUND` 后停车（不再冲）。

---

## Task 3: 转向闭环标定（ENC_PER_DEG）

**Files:**
- Modify: `User/main.c`（加转向标定测试入口）

**Interfaces:**
- Consumes: `Spin_Angle(int16_t deg)`、`Spin_Encoder()`（trace.c 已有）
- Produces: 标定好的 `ENC_PER_DEG` 常量（写入 trace.h）

- [ ] **Step 1: 在 main.c 加转向标定入口**

在 `main()` 主循环里，把 KEY1 的调试函数换成转向标定（保留 EMI 诊断可注释）：

```c
static void Turn_Calib_Test(void)
{
    while (1) {
        printf("spin 90...\r\n");
        Spin_Angle(90);
        delay_ms(1500);
        Spin_Angle(-90);
        delay_ms(1500);
    }
}
```

主循环 KEY1 分支改为调用 `Turn_Calib_Test()`。

- [ ] **Step 2: 实车标定 ENC_PER_DEG**

烧录后按 KEY1，车原地左右旋 90°。观察实际转角：
- 若实际转角 > 90°：`ENC_PER_DEG` 偏小，按比例调大（`新值 = 旧值 × 实际角/90`）。
- 若实际转角 < 90°：`ENC_PER_DEG` 偏大，按比例调小。
- 反复测到左右各转 90° 都准为止，把最终值写回 `trace.h` 的 `#define ENC_PER_DEG`。

同时确认旋转方向：`Spin_Angle(90)` 应左转（逆时针，车头朝北时朝西）。若方向反，把 `Spin_Angle` 里 `Diff_Drive(v, -v)` 的 v 取反（符号约定统一为「正 deg = 左转」）。

---

## Task 4: 导航层重写（里程定位 + 数线门控 + 路径状态机）

**Files:**
- Modify: `APP/nav/nav.h`（参数 + `NavState` + 接口）
- Modify: `APP/nav/nav.c`（重写：里程定位 + 侧方数线门控 + 路径状态机）
- Modify: `User/main.c`（KEY1 改为「出发→圆心→A」导航测试）

**Interfaces:**
- Consumes: `Trace_LineFollow`/`Trace_CenterHit`（Task 2）、`Spin_Angle`（Task 3）、`F_GW_classify`/`S_GW_Read`（灰度层）、`Encoder_Odometer`（trace.c）
- Produces:
  - `void Nav_Init(void)`
  - `void Nav_TaskStart(int16_t target_theta)`（target_theta：0=A，90=C，180=E）
  - `void Nav_TaskRun(void)`（每 10ms 推进一次）

- [ ] **Step 1: 重写 `nav.h`**

整文件替换为：

```c
#ifndef _nav_H
#define _nav_H

#include "main.h"

// 距离标定（编码器脉冲，实车标定）
#define DIST_START_TO_CENTER 15000  // 出发区→圆心（≈780mm）
#define DIST_CENTER_TO_NODE  7800   // 圆心→A/C/E（小圆半径）
#define DIST_ARRIVE_WINDOW   300    // 「到达」里程窗口
#define LANDMARK_WINDOW      200    // 数线门控窗口

// 出发区→圆心途中的路标（从段起点起算的里程，-1 结束）
//   路标[0] = 小圆线（≈小圆半径 DIST_CENTER_TO_NODE 处）
#define LANDMARK_TO_CENTER  DIST_CENTER_TO_NODE

typedef struct {
    int16_t theta;          // 所在辐射线角度
    int16_t target_theta;   // 目标辐射线角度
    int32_t odom_ref;       // 本段起点里程
    int32_t target_dist;    // 本段目标距离
    int16_t landmark[4];    // 预期路标里程（-1 结束）
    int16_t landmark_idx;   // 当前期待的路标序号
} NavState;

extern NavState Nav;

void Nav_Init(void);
void Nav_TaskStart(int16_t target_theta);
void Nav_TaskRun(void);

#endif
```

- [ ] **Step 2: 重写 `nav.c`**

整文件替换为：

```c
#include "nav.h"

NavState Nav;

typedef enum {
    NAV_IDLE = 0,
    NAV_TO_CENTER,
    NAV_TURN,
    NAV_TO_NODE,
    NAV_DONE,
} NavPhase;

static NavPhase nav_phase = NAV_IDLE;

void Nav_Init(void)
{
    Nav.theta = 270;
    Nav.target_theta = 0;
    Nav.odom_ref = 0;
    Nav.target_dist = 0;
    Nav.landmark[0] = -1;
    Nav.landmark_idx = 0;
    nav_phase = NAV_IDLE;
}

// 里程窗口内接受侧方灰度白→黑上升沿 → 校正 odom_ref（消除滑移漂移）
static uint8_t s_gw_last = 0;

static void Nav_UpdateLandmark(void)
{
    int16_t lm = Nav.landmark[Nav.landmark_idx];
    int32_t odom = Encoder_Odometer();
    int32_t d = odom - Nav.odom_ref;
    uint8_t gw = S_GW_Read();

    if (lm < 0) { s_gw_last = gw; return; }   // 无更多路标

    if (gw && !s_gw_last) {                    // 白→黑上升沿
        if (d > lm - LANDMARK_WINDOW && d < lm + LANDMARK_WINDOW) {
            Nav.odom_ref = odom;               // 路标确认 → 校正里程基准
            Nav.landmark_idx++;
        }
    }
    s_gw_last = gw;
}

// 到达圆心：里程接近 + 全黑超阈值
static uint8_t Nav_ReachedCenter(void)
{
    int32_t d = Encoder_Odometer() - Nav.odom_ref;
    if (d < 0) d = -d;
    return (d > Nav.target_dist - DIST_ARRIVE_WINDOW) &&
           (d < Nav.target_dist + DIST_ARRIVE_WINDOW) &&
           Trace_CenterHit();
}

// 到达物料点：里程接近 + 前灰度全白（空白圆）
static uint8_t Nav_ReachedNode(void)
{
    int32_t d = Encoder_Odometer() - Nav.odom_ref;
    if (d < 0) d = -d;
    return (d > Nav.target_dist - DIST_ARRIVE_WINDOW) &&
           (d < Nav.target_dist + DIST_ARRIVE_WINDOW) &&
           (F_GW_classify() == LINE_LOST);
}

void Nav_TaskStart(int16_t target_theta)
{
    Nav.theta = 270;
    Nav.target_theta = target_theta;
    Nav.odom_ref = Encoder_Odometer();
    Nav.target_dist = DIST_START_TO_CENTER;
    Nav.landmark[0] = LANDMARK_TO_CENTER;   // 小圆线是途中最清晰的路标
    Nav.landmark[1] = -1;
    Nav.landmark_idx = 0;
    s_gw_last = S_GW_Read();
    nav_phase = NAV_TO_CENTER;
}

// 270° 转到目标角的带符号转角（最短路径，正=左转）
static int16_t Nav_TurnAngle(void)
{
    int16_t diff = Nav.target_theta - Nav.theta;
    if (diff > 180)  diff -= 360;
    if (diff < -180) diff += 360;
    return diff;
}

void Nav_TaskRun(void)
{
    switch (nav_phase) {
    case NAV_TO_CENTER:
        Nav_UpdateLandmark();
        Trace_LineFollow(LINE_SPEED);
        if (Nav_ReachedCenter()) {
            Stop();
            nav_phase = NAV_TURN;
        }
        break;

    case NAV_TURN:
        Spin_Angle(Nav_TurnAngle());
        Nav.theta = Nav.target_theta;
        Nav.odom_ref = Encoder_Odometer();
        Nav.target_dist = DIST_CENTER_TO_NODE;
        Nav.landmark[0] = DIST_CENTER_TO_NODE;  // 终点即小圆线（物料点所在圆）
        Nav.landmark[1] = -1;
        Nav.landmark_idx = 0;
        s_gw_last = S_GW_Read();
        nav_phase = NAV_TO_NODE;
        break;

    case NAV_TO_NODE:
        Nav_UpdateLandmark();
        Trace_LineFollow(LINE_SPEED);
        if (Nav_ReachedNode()) {
            Stop();
            nav_phase = NAV_DONE;
        }
        break;

    case NAV_DONE:
    case NAV_IDLE:
    default:
        Stop();
        break;
    }
}
```

- [ ] **Step 3: 修改 main.c 加导航测试入口**

```c
static void Nav_Test(void)
{
    Nav_Init();
    Nav_TaskStart(0);   // 0 = A 点
    while (1) {
        Nav_TaskRun();
        delay_ms(10);
    }
}
```

主循环 KEY1 分支改为调用 `Nav_Test()`。

- [ ] **Step 4: 编译验证**

EIDE Rebuild。Expected: 0 error。若报 `Trace_CenterHit` 未声明，确认 `trace.h` 已 include 且声明在 Task 2 已加。

- [ ] **Step 5: 实车验证（Review Focus 4、5 的锚点）**

烧录后按 KEY1（车放出发区，车头朝圆心/朝北）：
1. 车沿 270° 线向北循迹，到圆心附近停车（里程 + 全黑双条件，普通路口不停）。
2. 圆心处左转 90°（去 A），沿 0° 线向西，到 A 空白圆停车。
3. 验证侧方数线不干扰：途中若侧方灰度扫到非预期黑线（窗口外），车不应有异常动作；`DIST_START_TO_CENTER` 等三组距离按实测调整（记录各段实际脉冲，回填 `nav.h`）。
4. 重复多次，确认到达 A 点位置一致（重复性）。

---

## Task 5: 路径参数化扩展到 C/E

**Files:**
- Modify: `User/main.c`（测试入口加目标角选择，或暂用编译宏切换）

**Interfaces:**
- Consumes: `Nav_TaskStart(int16_t target_theta)`（Task 4）

- [ ] **Step 1: 让导航测试入口覆盖 C/E**

`Nav_Test` 的目标角改为可切换（临时用宏，实测时改宏重编）：

```c
#define NAV_TEST_TARGET 0   // 0=A, 90=C, 180=E

static void Nav_Test(void)
{
    Nav_Init();
    Nav_TaskStart(NAV_TEST_TARGET);
    while (1) {
        Nav_TaskRun();
        delay_ms(10);
    }
}
```

- [ ] **Step 2: 实车分别验证 A/C/E**

改 `NAV_TEST_TARGET` 为 90、180 各烧录一次：
- C（90°）：圆心处 `Nav_TurnAngle()` 返回 0（270→90 是直线延伸，不转或转 0），车继续向北到 C。
- E（180°）：圆心处右转 90°，沿 180° 线向东到 E。
Expected: 三点都能准确到达并停车。若 C 点「270→90」的转向角计算有误（应为 0），检查 `Nav_TurnAngle` 的 360 归一逻辑。

- [ ] **Step 3: 收尾**

确认 A/C/E 三点导航都稳定后，把 `nav.h`/`trace.h` 里所有实测标定值回填，并更新 [开发记录.md](../../../开发记录.md) 的「当前进度」为「循迹重构 + 导航到 A/C/E 完成，待接夹取/识别」。
