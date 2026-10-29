#ifndef _TCS3200_H
#define _TCS3200_H

#include "main.h"

/* ============================================================================
 * TCS3200 / GY-31 颜色识别模块（STM32F407 + 标准外设库）
 * ============================================================================
 * 【模块引脚】GY-31（板载 TCS3200，2×5 排针，两路 VCC/GND 任选其一）
 *   VCC  -> 3.3V（推荐；5V 供电时 OUT 为 5V，且 S0~S3 的 3.3V 可能识别不到高电平）
 *   GND  -> GND
 *   S0   -> PC0   输出频率档位（本驱动用 GPIO 控制）
 *   S1   -> PC1
 *   S2   -> PC2   滤光片选择
 *   S3   -> PC3
 *   OUT  -> PC6   TIM3_CH1，脉冲计数输入
 *   LED  -> 3.3V  ★高电平点亮板载 4 个白光 LED（内部 10K 下拉，悬空=灯灭）
 *   OE   -> 模块内部已接地（使能输出），无需接
 *
 * 【原理】
 *   TCS3200 的 OUT 是频率随光强变化的方波：
 *       S0/S1 = 1/1 -> 100%（最高约 600kHz）
 *       S0/S1 = 0/1 ->  20%（最高约 120kHz，推荐）
 *       S0/S1 = 1/0 ->   2%
 *       S0/S1 = 0/0 -> 掉电
 *   滤光片：S2/S3 = 00 红、01 蓝、11 绿、10 白（Clear，无滤光片）。
 *   本驱动把 OUT 接到 TIM3_CH1，用 TIM3「外部时钟模式 1」（TI1FP1）硬件计数，
 *   软件只负责给固定闸门时间（默认 20ms）后读 CNT，不占 CPU、不受中断抖动影响。
 *
 * 【识别流程】
 *   依次切换 R/G/B/Clear 四个滤光片各测一次脉冲数
 *   -> 用白平衡得到的每通道参考值归一化到 0~255
 *   -> 按通道比例判色（黑/白/红/绿/蓝/黄/青/紫）。
 *
 * 【使用步骤】
 *   1. TCS3200_Init();
 *   2. 传感器对准白纸（距离约 1~2cm），调用 TCS3200_CalibrateWhite(8);
 *   3. TCS3200_ReadColor() 或 TCS3200_ReadColorStable(3)。
 *
 * 【示例】放在 main() 初始化之后：
 *   TCS3200_Init();
 *   delay_ms(300);                              // 等模块上电稳定
 *   if (TCS3200_CalibrateWhite(8) != 0) {
 *       printf("TCS3200 白平衡失败：请对准白纸\r\n");
 *   }
 *   while (1) {
 *       TCS3200_DebugPrint();                   // 打印 RAW / RGB / 颜色名
 *       delay_ms(300);
 *   }
 * ==========================================================================*/

/* ---------------------------- 引脚配置（按实际接线改） ---------------------- */
#define TCS3200_GPIO_CLK RCC_AHB1Periph_GPIOC

#define TCS3200_S0_PORT  GPIOC
#define TCS3200_S0_PIN   GPIO_Pin_0

#define TCS3200_S1_PORT  GPIOC
#define TCS3200_S1_PIN   GPIO_Pin_1

#define TCS3200_S2_PORT  GPIOC
#define TCS3200_S2_PIN   GPIO_Pin_2

#define TCS3200_S3_PORT  GPIOC
#define TCS3200_S3_PIN   GPIO_Pin_3

/* OUT 必须是 TIM3_CH1 的复用引脚：PC6（默认）/ PA6 / PB4。
   若改到别的引脚，记得同步改 PIN_SOURCE 与 AF。 */
#define TCS3200_OUT_PORT       GPIOC
#define TCS3200_OUT_PIN        GPIO_Pin_6
#define TCS3200_OUT_PIN_SOURCE GPIO_PinSource6
#define TCS3200_OUT_AF         GPIO_AF_TIM3
#define TCS3200_TIM            TIM3
#define TCS3200_TIM_CLK        RCC_APB1Periph_TIM3

/* ------------------------- 识别参数（按场地/物料微调） ---------------------- */
/* 闸门时间：每切换一次滤光片计数的时长。约束：TCS3200_GATE_MS * 600 < 65536
   （600 = 100% 档 600kHz 折算成 "每 ms 的脉冲数"），50ms 对应最多 30000。
   现场白纸 RAW 只有十几，先把闸门从 20ms 加到 50ms 提高信噪比。 */
#define TCS3200_GATE_MS 50

/* 切换滤光片后等待输出稳定的时间 */
#define TCS3200_SETTLE_MS 5

/* 上电默认档位：现场信号偏弱，先用 100% 档把计数放大；
   若出现计数饱和（ReadRGB 返回 1），再改回 TCS3200_SCALE_20PCT。 */
#define TCS3200_DEFAULT_SCALE TCS3200_SCALE_100PCT

/* 归一化后（0~255）三通道最大值 < 该值 -> 黑 */
#define TCS3200_BLACK_MAX 45

/* 归一化后三通道最小值 >= 该值，且三通道接近 -> 白 */
#define TCS3200_WHITE_MIN 150

/* 饱和度上限（%）：饱和度 <= 该值视为无彩色（白/灰） */
#define TCS3200_WHITE_SPREAD 25

/* 有彩色最小饱和度（%）：低于该值不判色，防止灰/弱光误判 */
#define TCS3200_COLOR_MIN_SAT 30

/* 五色模式的色相分区（度，0~359）：
   红 [TCS3200_HUE_BLUE_HIGH, 360) ∪ [0, TCS3200_HUE_RED_HIGH)
   绿 [TCS3200_HUE_RED_HIGH, TCS3200_HUE_GREEN_HIGH)
   蓝 [TCS3200_HUE_GREEN_HIGH, TCS3200_HUE_BLUE_HIGH)
   色相只比通道比例，不受整体亮度影响。 */
#define TCS3200_HUE_RED_HIGH   60
#define TCS3200_HUE_GREEN_HIGH 180
#define TCS3200_HUE_BLUE_HIGH  300

/* 1 = 额外识别黄/青/紫（混色）；0 = 只识别黑/白/红/绿/蓝。
   置 0 时任何有彩色都会归到色相最近的红/绿/蓝。 */
#define TCS3200_ENABLE_MIXED_COLORS 0

/* 多次采样投票的默认次数 */
#define TCS3200_STABLE_SAMPLES 3

/* 白平衡时，白纸原始计数的最小值：低于该值说明太暗/没对准，标定失败。
   100% 档 + 50ms 下白纸 R/G/B 约 900~2000，取 300 可拒绝黑/深色表面误标定。 */
#define TCS3200_WB_MIN_REF 300

/* ------------------- 比赛模式：固定白平衡参考值 ---------------------------- */
/* 1 = 直接使用下面实测参考值，运行时禁止标定（比赛用）
   0 = 允许 KEY0 / TCS3200_CalibrateWhite() 现场标定（调试用） */
#define TCS3200_USE_FIXED_WHITE_REF 1

/* ★2026-10-29 重标：夹爪结构改动后模块离料更近，白纸 RAW 从 1715 涨到 6747（3.9 倍）。
   旧参考偏小 → 所有通道归一化后被削顶到 255 → 绿料饱和度塌到 16%（判成白）、
   黑料被放大到 108（掉进 45~150 灰区判 UNKNOWN）。按新几何实测白料 RAW 重设：
     R/G/B = 6747 / 6994 / 8096（车上夹持位实测）
   判色只用 R/G/B，Clear 不参与，保持旧值不影响结果。 */
#define TCS3200_FIXED_REF_R 6747u
#define TCS3200_FIXED_REF_G 6994u
#define TCS3200_FIXED_REF_B 8096u
#define TCS3200_FIXED_REF_C 5186u

/* ------------------------------- 类型定义 ---------------------------------- */
typedef enum {
    TCS3200_SCALE_POWER_DOWN = 0, /* S0=0, S1=0 */
    TCS3200_SCALE_2PCT       = 1, /* S0=1, S1=0 */
    TCS3200_SCALE_20PCT      = 2, /* S0=0, S1=1 */
    TCS3200_SCALE_100PCT     = 3  /* S0=1, S1=1 */
} TCS3200_Scale_t;

typedef enum {
    TCS3200_FILTER_RED   = 0, /* S2=0, S3=0 */
    TCS3200_FILTER_BLUE  = 1, /* S2=0, S3=1 */
    TCS3200_FILTER_CLEAR = 2, /* S2=1, S3=0 */
    TCS3200_FILTER_GREEN = 3  /* S2=1, S3=1 */
} TCS3200_Filter_t;

/* 判色结果：1~5 与 color.h 里的 white/black/red/green/blue 取值一致，
   方便直接替换原来的 color_get() 返回值。 */
typedef enum {
    TCS3200_COLOR_NONE    = 0, /* 无结果 / 采样失败 */
    TCS3200_COLOR_WHITE   = 1, /* 白 */
    TCS3200_COLOR_BLACK   = 2, /* 黑 */
    TCS3200_COLOR_RED     = 3, /* 红 */
    TCS3200_COLOR_GREEN   = 4, /* 绿 */
    TCS3200_COLOR_BLUE    = 5, /* 蓝 */
    TCS3200_COLOR_YELLOW  = 6, /* 黄（红+绿） */
    TCS3200_COLOR_CYAN    = 7, /* 青（绿+蓝） */
    TCS3200_COLOR_PURPLE  = 8, /* 紫（红+蓝） */
    TCS3200_COLOR_UNKNOWN = 9  /* 无法判断（如灰色/过渡色） */
} TCS3200_Color_t;

/* 四路原始计数（闸门时间内的脉冲个数，Clear 为无滤光片通道） */
typedef struct
{
    uint32_t R;
    uint32_t G;
    uint32_t B;
    uint32_t Clear;
} TCS3200_Raw_t;

/* 白平衡归一化后的 RGB（0~255） */
typedef struct
{
    uint8_t R;
    uint8_t G;
    uint8_t B;
} TCS3200_RGB_t;

/* ------------------------------- 接口 -------------------------------------- */

/* 初始化 GPIO + TIM3（会先做一次 20% 档、Clear 滤光片设置） */
void TCS3200_Init(void);

/* 设置输出频率档位（掉电/2%/20%/100%） */
void TCS3200_SetScale(TCS3200_Scale_t scale);

/* 设置滤光片 */
void TCS3200_SetFilter(TCS3200_Filter_t filter);

/* 单独测量某一滤光片在 gate_ms 内的脉冲数（调试用） */
uint32_t TCS3200_MeasurePulse(TCS3200_Filter_t filter, uint16_t gate_ms);

/* 采集 R/G/B/Clear 四路原始计数。gate_ms 传 0 时使用 TCS3200_GATE_MS */
void TCS3200_ReadRaw(TCS3200_Raw_t *raw, uint16_t gate_ms);

/* 白平衡：对准白纸采样 samples 次取平均，成功后归一到 0~255。
   返回 0=成功，1=失败（太暗/未接模块/未对准），失败时保留旧参考值。
   比赛模式（TCS3200_USE_FIXED_WHITE_REF=1）下本函数不采样，直接返回 0。 */
uint8_t TCS3200_CalibrateWhite(uint8_t samples);

/* 读取当前白平衡参考值（未标定时全为 0） */
void TCS3200_GetWhiteRef(TCS3200_Raw_t *ref);

/* 是否已完成白平衡（1=已标定） */
uint8_t TCS3200_IsWhiteBalanced(void);

/* 原始计数 -> 归一化 RGB。
   已标定：各通道除以自己的白纸参考值；
   未标定：按本次最强通道自动增益（只能看比例，不能区分黑/白）。 */
void TCS3200_RawToRGB(const TCS3200_Raw_t *raw, TCS3200_RGB_t *rgb);

/* 按归一化 RGB 判色（不采样，纯计算，可单独测试阈值） */
TCS3200_Color_t TCS3200_Classify(const TCS3200_RGB_t *rgb);

/* 采样一次并判色，返回 0=正常，1=某通道计数饱和（建议缩短闸门/降档位） */
uint8_t TCS3200_ReadRGB(TCS3200_RGB_t *rgb);

/* 采样一次并判色。注意：未做白平衡时返回 TCS3200_COLOR_NONE
   （没有白色参考就无法区分黑/白，宁可不给结果） */
TCS3200_Color_t TCS3200_ReadColor(void);

/* 采样 samples 次，多数投票，抗单次误判 */
TCS3200_Color_t TCS3200_ReadColorStable(uint8_t samples);

/* 颜色名（ASCII，便于串口打印） */
const char *TCS3200_ColorName(TCS3200_Color_t color);

/* 调试：打印 RAW R/G/B/C、归一化 RGB 与颜色名（走 printf -> USART1） */
void TCS3200_DebugPrint(void);

/* ============================================================================
 * 颜色标定模式（本车无串口，改用 Watch 读）
 *   调用一次 = 采样一次（4 个滤光片 × 50ms ≈ 220ms），把结果写进下面这些 volatile 变量。
 *   与 TCS3200_ReadColor() 不同：这里**不丢 UNKNOWN**，判定结果原样给出，
 *   用于看清「料被判成什么、为什么被判成这个」。
 * ========================================================================== */
extern volatile uint8_t  DBG_COLOR_ID;      /* 本次判定：0 NONE 1 白 2 黑 3 红 4 绿 5 蓝 9 UNKNOWN */
extern volatile uint8_t  DBG_COLOR_R;       /* 归一化 R（0~255，白纸参考下白=255） */
extern volatile uint8_t  DBG_COLOR_G;
extern volatile uint8_t  DBG_COLOR_B;
extern volatile uint32_t DBG_COLOR_RAW_R;   /* 原始脉冲数（白纸基准 1715/1776/2030/5186） */
extern volatile uint32_t DBG_COLOR_RAW_G;
extern volatile uint32_t DBG_COLOR_RAW_B;
extern volatile uint32_t DBG_COLOR_RAW_C;
extern volatile uint16_t DBG_COLOR_N;       /* 采样次数，在涨 = 程序活着 */

void TCS3200_CalibSample(void);

#endif
