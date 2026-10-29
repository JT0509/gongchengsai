#include "tcs3200.h"

/* 闸门时间 × 100% 档最高频率(600kHz) 必须小于 16 位计数器量程 */
#if (TCS3200_GATE_MS * 600 > 65535)
#error "TCS3200_GATE_MS 太大：TIM3 是 16 位计数器，且 100% 档最高约 600kHz（闸门需 < 109ms）"
#endif

#if TCS3200_USE_FIXED_WHITE_REF
/* 比赛模式：固定参考值，初始化即为“已标定” */
static TCS3200_Raw_t s_white_ref = {
    TCS3200_FIXED_REF_R,
    TCS3200_FIXED_REF_G,
    TCS3200_FIXED_REF_B,
    TCS3200_FIXED_REF_C};
static uint8_t s_white_ready = 1u;
#else
/* 调试模式：上电后需要 KEY0 / TCS3200_CalibrateWhite() 标定 */
static TCS3200_Raw_t s_white_ref;
static uint8_t s_white_ready = 0u;
#endif

/* ------------------------------ 底层工具 ---------------------------------- */

static void TCS3200_WritePin(GPIO_TypeDef *port, uint16_t pin, uint8_t level)
{
    GPIO_WriteBit(port, pin, (level != 0u) ? Bit_SET : Bit_RESET);
}

/* 取 0~255，超量程钳位（白纸比标定时更亮时会溢出） */
static uint8_t TCS3200_Normalize(uint32_t value, uint32_t ref)
{
    uint32_t t;

    if (ref == 0u) {
        ref = 1u;
    }
    t = value * 255u;
    t /= ref;
    if (t > 255u) {
        t = 255u;
    }
    return (uint8_t)t;
}

static uint32_t TCS3200_MaxOf3(uint32_t a, uint32_t b, uint32_t c)
{
    uint32_t m = a;

    if (b > m) {
        m = b;
    }
    if (c > m) {
        m = c;
    }
    return m;
}

/* ------------------------------ 初始化 ------------------------------------ */

void TCS3200_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    RCC_AHB1PeriphClockCmd(TCS3200_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(TCS3200_TIM_CLK, ENABLE);

    /* S0~S3：推挽输出 */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = TCS3200_S0_PIN;
    GPIO_Init(TCS3200_S0_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TCS3200_S1_PIN;
    GPIO_Init(TCS3200_S1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TCS3200_S2_PIN;
    GPIO_Init(TCS3200_S2_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TCS3200_S3_PIN;
    GPIO_Init(TCS3200_S3_PORT, &GPIO_InitStructure);

    /* OUT：复用为 TIM3_CH1，上拉保证模块未接时电平确定 */
    GPIO_PinAFConfig(TCS3200_OUT_PORT, TCS3200_OUT_PIN_SOURCE, TCS3200_OUT_AF);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin  = TCS3200_OUT_PIN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(TCS3200_OUT_PORT, &GPIO_InitStructure);

    /* TIM3：外部时钟模式 1，TI1FP1 每个上升沿使 CNT+1 */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TCS3200_TIM, &TIM_TimeBaseStructure);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    /* 滤波 f_DTS/4、N=6（约 0.29us 采样窗口）：滤掉短毛刺，不影响 600kHz 信号 */
    TIM_ICInitStructure.TIM_ICFilter = 0x06;
    TIM_ICInit(TCS3200_TIM, &TIM_ICInitStructure);

    TIM_SelectInputTrigger(TCS3200_TIM, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TCS3200_TIM, TIM_SlaveMode_External1);
    TIM_Cmd(TCS3200_TIM, DISABLE);

    /* 默认：100% 档 + Clear 滤光片（见 TCS3200_DEFAULT_SCALE） */
    TCS3200_SetScale(TCS3200_DEFAULT_SCALE);
    TCS3200_SetFilter(TCS3200_FILTER_CLEAR);
}

/* ------------------------------ 档位/滤光片 -------------------------------- */

void TCS3200_SetScale(TCS3200_Scale_t scale)
{
    switch (scale) {
        case TCS3200_SCALE_POWER_DOWN: /* 00 */
            TCS3200_WritePin(TCS3200_S0_PORT, TCS3200_S0_PIN, 0);
            TCS3200_WritePin(TCS3200_S1_PORT, TCS3200_S1_PIN, 0);
            break;

        case TCS3200_SCALE_2PCT: /* 10 */
            TCS3200_WritePin(TCS3200_S0_PORT, TCS3200_S0_PIN, 1);
            TCS3200_WritePin(TCS3200_S1_PORT, TCS3200_S1_PIN, 0);
            break;

        case TCS3200_SCALE_20PCT: /* 01 */
            TCS3200_WritePin(TCS3200_S0_PORT, TCS3200_S0_PIN, 0);
            TCS3200_WritePin(TCS3200_S1_PORT, TCS3200_S1_PIN, 1);
            break;

        case TCS3200_SCALE_100PCT: /* 11 */
        default:
            TCS3200_WritePin(TCS3200_S0_PORT, TCS3200_S0_PIN, 1);
            TCS3200_WritePin(TCS3200_S1_PORT, TCS3200_S1_PIN, 1);
            break;
    }
}

void TCS3200_SetFilter(TCS3200_Filter_t filter)
{
    switch (filter) {
        case TCS3200_FILTER_RED: /* S2=0, S3=0 */
            TCS3200_WritePin(TCS3200_S2_PORT, TCS3200_S2_PIN, 0);
            TCS3200_WritePin(TCS3200_S3_PORT, TCS3200_S3_PIN, 0);
            break;

        case TCS3200_FILTER_BLUE: /* S2=0, S3=1 */
            TCS3200_WritePin(TCS3200_S2_PORT, TCS3200_S2_PIN, 0);
            TCS3200_WritePin(TCS3200_S3_PORT, TCS3200_S3_PIN, 1);
            break;

        case TCS3200_FILTER_GREEN: /* S2=1, S3=1 */
            TCS3200_WritePin(TCS3200_S2_PORT, TCS3200_S2_PIN, 1);
            TCS3200_WritePin(TCS3200_S3_PORT, TCS3200_S3_PIN, 1);
            break;

        case TCS3200_FILTER_CLEAR: /* S2=1, S3=0，无滤光片 */
        default:
            TCS3200_WritePin(TCS3200_S2_PORT, TCS3200_S2_PIN, 1);
            TCS3200_WritePin(TCS3200_S3_PORT, TCS3200_S3_PIN, 0);
            break;
    }
}

/* ------------------------------ 采样 -------------------------------------- */

uint32_t TCS3200_MeasurePulse(TCS3200_Filter_t filter, uint16_t gate_ms)
{
    uint32_t count;

    if (gate_ms == 0u) {
        gate_ms = TCS3200_GATE_MS;
    }

    TCS3200_SetFilter(filter);
    delay_ms(TCS3200_SETTLE_MS); /* 等输出频率稳定 */

    TCS3200_TIM->CNT = 0;
    TIM_ClearFlag(TCS3200_TIM, TIM_FLAG_Update);
    TIM_Cmd(TCS3200_TIM, ENABLE); /* 开始外部脉冲计数 */

    delay_ms(gate_ms); /* 闸门时间 */

    TIM_Cmd(TCS3200_TIM, DISABLE);
    count = (uint32_t)TCS3200_TIM->CNT;

    return count;
}

void TCS3200_ReadRaw(TCS3200_Raw_t *raw, uint16_t gate_ms)
{
    if (raw == 0) {
        return;
    }
    if (gate_ms == 0u) {
        gate_ms = TCS3200_GATE_MS;
    }

    raw->R     = TCS3200_MeasurePulse(TCS3200_FILTER_RED, gate_ms);
    raw->G     = TCS3200_MeasurePulse(TCS3200_FILTER_GREEN, gate_ms);
    raw->B     = TCS3200_MeasurePulse(TCS3200_FILTER_BLUE, gate_ms);
    raw->Clear = TCS3200_MeasurePulse(TCS3200_FILTER_CLEAR, gate_ms);
}

/* ------------------------------ 白平衡 ------------------------------------ */

uint8_t TCS3200_CalibrateWhite(uint8_t samples)
{
#if TCS3200_USE_FIXED_WHITE_REF
    /* 比赛模式：禁止现场标定，直接用固定参考值 */
    (void)samples;
    return 0u;
#else
    TCS3200_Raw_t raw;
    TCS3200_Raw_t sum = {0u, 0u, 0u, 0u};
    uint8_t i;

    if (samples == 0u) {
        samples = 1u;
    }

    for (i = 0; i < samples; i++) {
        TCS3200_ReadRaw(&raw, TCS3200_GATE_MS);
        sum.R += raw.R;
        sum.G += raw.G;
        sum.B += raw.B;
        sum.Clear += raw.Clear;
    }

    sum.R /= samples;
    sum.G /= samples;
    sum.B /= samples;
    sum.Clear /= samples;

    /* 太暗说明没对准白纸或模块没接，保留旧参考值 */
    if (sum.R < TCS3200_WB_MIN_REF ||
        sum.G < TCS3200_WB_MIN_REF ||
        sum.B < TCS3200_WB_MIN_REF) {
        return 1u;
    }

    s_white_ref   = sum;
    s_white_ready = 1u;
    return 0u;
#endif
}

void TCS3200_GetWhiteRef(TCS3200_Raw_t *ref)
{
    if (ref != 0) {
        *ref = s_white_ref;
    }
}

uint8_t TCS3200_IsWhiteBalanced(void)
{
    return s_white_ready;
}

void TCS3200_RawToRGB(const TCS3200_Raw_t *raw, TCS3200_RGB_t *rgb)
{
    uint32_t max_raw;

    if (raw == 0 || rgb == 0) {
        return;
    }

    if (s_white_ready) {
        /* 已标定：各通道除以自己的白纸参考值，白色 -> 255/255/255 */
        rgb->R = TCS3200_Normalize(raw->R, s_white_ref.R);
        rgb->G = TCS3200_Normalize(raw->G, s_white_ref.G);
        rgb->B = TCS3200_Normalize(raw->B, s_white_ref.B);
    } else {
        /* 未标定：按本次最强通道自动增益，保证仍能按比例判色 */
        max_raw = TCS3200_MaxOf3(raw->R, raw->G, raw->B);
        if (max_raw == 0u) {
            rgb->R = 0u;
            rgb->G = 0u;
            rgb->B = 0u;
        } else {
            rgb->R = TCS3200_Normalize(raw->R, max_raw);
            rgb->G = TCS3200_Normalize(raw->G, max_raw);
            rgb->B = TCS3200_Normalize(raw->B, max_raw);
        }
    }
}

/* ------------------------------ 判色 -------------------------------------- */

/* 色相 0~359 度（纯整数运算）。delta = max-min，max 为最强通道 */
static uint16_t TCS3200_CalcHue(uint16_t r, uint16_t g, uint16_t b, uint16_t max, uint16_t delta)
{
    int32_t h;

    if (delta == 0u) {
        return 0u;
    }

    if (max == r) {
        h = (int32_t)60 * ((int32_t)g - (int32_t)b) / (int32_t)delta;
        if (h < 0) {
            h += 360;
        }
    } else if (max == g) {
        h = 120 + (int32_t)60 * ((int32_t)b - (int32_t)r) / (int32_t)delta;
    } else {
        h = 240 + (int32_t)60 * ((int32_t)r - (int32_t)g) / (int32_t)delta;
    }

    if (h < 0) {
        h += 360;
    }
    if (h >= 360) {
        h -= 360;
    }
    return (uint16_t)h;
}

TCS3200_Color_t TCS3200_Classify(const TCS3200_RGB_t *rgb)
{
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t max;
    uint16_t min;
    uint16_t delta;
    uint16_t sat; /* 饱和度 % */
    uint16_t hue; /* 色相 0~359 */

    if (rgb == 0) {
        return TCS3200_COLOR_NONE;
    }

    r = rgb->R;
    g = rgb->G;
    b = rgb->B;

    /* 全 0：全黑或没信号 */
    if (r == 0u && g == 0u && b == 0u) {
        return TCS3200_COLOR_BLACK;
    }

    /* 找最强/最弱通道 */
    max = r;
    if (g > max) {
        max = g;
    }
    if (b > max) {
        max = b;
    }

    min = r;
    if (g < min) {
        min = g;
    }
    if (b < min) {
        min = b;
    }

    /* 1) 亮度上限：整体太暗 -> 黑 */
    if (max < TCS3200_BLACK_MAX) {
        return TCS3200_COLOR_BLACK;
    }

    delta = (uint16_t)(max - min);
    sat   = (uint16_t)(((uint32_t)delta * 100u) / max);

    /* 2) 饱和度上限：三通道接近 -> 白（够亮）或灰 */
    if (sat <= TCS3200_WHITE_SPREAD) {
        if (min >= TCS3200_WHITE_MIN) {
            return TCS3200_COLOR_WHITE;
        }
        return TCS3200_COLOR_UNKNOWN;
    }

    /* 3) 饱和度下限：有彩色但不够鲜艳 -> 不判 */
    if (sat < TCS3200_COLOR_MIN_SAT) {
        return TCS3200_COLOR_UNKNOWN;
    }

    /* 4) 色相上下限判色 */
    hue = TCS3200_CalcHue(r, g, b, max, delta);

#if TCS3200_ENABLE_MIXED_COLORS
    if (hue >= 340u || hue < 20u) {
        return TCS3200_COLOR_RED;
    }
    if (hue >= 40u && hue < 75u) {
        return TCS3200_COLOR_YELLOW;
    }
    if (hue >= 80u && hue < 160u) {
        return TCS3200_COLOR_GREEN;
    }
    if (hue >= 170u && hue < 200u) {
        return TCS3200_COLOR_CYAN;
    }
    if (hue >= 200u && hue < 260u) {
        return TCS3200_COLOR_BLUE;
    }
    if (hue >= 270u && hue < 330u) {
        return TCS3200_COLOR_PURPLE;
    }
    return TCS3200_COLOR_UNKNOWN;
#else
    if (hue < TCS3200_HUE_RED_HIGH || hue >= TCS3200_HUE_BLUE_HIGH) {
        return TCS3200_COLOR_RED;
    }
    if (hue < TCS3200_HUE_GREEN_HIGH) {
        return TCS3200_COLOR_GREEN;
    }
    return TCS3200_COLOR_BLUE;
#endif
}

/* ------------------------------ 对外接口 ---------------------------------- */

uint8_t TCS3200_ReadRGB(TCS3200_RGB_t *rgb)
{
    TCS3200_Raw_t raw;
    uint8_t saturated = 0u;

    TCS3200_ReadRaw(&raw, TCS3200_GATE_MS);
    TCS3200_RawToRGB(&raw, rgb);

    /* 计数接近 16 位满量程：说明闸门内脉冲过多，结果可能已卷绕 */
    if (raw.R >= 0xFFF0u || raw.G >= 0xFFF0u || raw.B >= 0xFFF0u) {
        saturated = 1u;
    }
    return saturated;
}

TCS3200_Color_t TCS3200_ReadColor(void)
{
    TCS3200_RGB_t rgb;

    /* 未白平衡：绝对亮度未知，黑/白不可靠，直接返回 NONE */
    if (!s_white_ready) {
        return TCS3200_COLOR_NONE;
    }

    TCS3200_ReadRGB(&rgb);
    return TCS3200_Classify(&rgb);
}

TCS3200_Color_t TCS3200_ReadColorStable(uint8_t samples)
{
    uint8_t votes[TCS3200_COLOR_UNKNOWN + 1] = {0};
    uint8_t i;
    uint8_t best_count = 0u;
    uint8_t best_index = TCS3200_COLOR_NONE;
    TCS3200_Color_t c;

    if (samples == 0u) {
        samples = 1u;
    }
    if (samples > 15u) {
        samples = 15u; /* 限制单次阻塞时间 */
    }

    for (i = 0; i < samples; i++) {
        c = TCS3200_ReadColor();
        if ((uint8_t)c <= TCS3200_COLOR_UNKNOWN) {
            votes[(uint8_t)c]++;
        }
    }

    /* 从 1 开始：NONE 不参与投票 */
    for (i = 1u; i <= TCS3200_COLOR_UNKNOWN; i++) {
        if (votes[i] > best_count) {
            best_count = votes[i];
            best_index = i;
        }
    }

    return (best_count == 0u) ? TCS3200_COLOR_NONE : (TCS3200_Color_t)best_index;
}

/* ---------------------- 颜色标定模式（Watch 读值用） ---------------------- */

volatile uint8_t  DBG_COLOR_ID    = 0;
volatile uint8_t  DBG_COLOR_R     = 0;
volatile uint8_t  DBG_COLOR_G     = 0;
volatile uint8_t  DBG_COLOR_B     = 0;
volatile uint32_t DBG_COLOR_RAW_R = 0;
volatile uint32_t DBG_COLOR_RAW_G = 0;
volatile uint32_t DBG_COLOR_RAW_B = 0;
volatile uint32_t DBG_COLOR_RAW_C = 0;
volatile uint16_t DBG_COLOR_N     = 0;

/* 采样一次并留档。★与跑车时不同：UNKNOWN(9) 也原样写出来，方便看黑料到底落在哪。 */
void TCS3200_CalibSample(void)
{
    TCS3200_Raw_t   raw;
    TCS3200_RGB_t   rgb;
    TCS3200_Color_t c;

    TCS3200_ReadRaw(&raw, TCS3200_GATE_MS);
    TCS3200_RawToRGB(&raw, &rgb);
    c = TCS3200_Classify(&rgb);

    DBG_COLOR_RAW_R = raw.R;
    DBG_COLOR_RAW_G = raw.G;
    DBG_COLOR_RAW_B = raw.B;
    DBG_COLOR_RAW_C = raw.Clear;

    DBG_COLOR_R = rgb.R;
    DBG_COLOR_G = rgb.G;
    DBG_COLOR_B = rgb.B;

    DBG_COLOR_ID = (uint8_t)c;
    DBG_COLOR_N++;
}

const char *TCS3200_ColorName(TCS3200_Color_t color)
{
    switch (color) {
        case TCS3200_COLOR_WHITE:
            return "WHITE";
        case TCS3200_COLOR_BLACK:
            return "BLACK";
        case TCS3200_COLOR_RED:
            return "RED";
        case TCS3200_COLOR_GREEN:
            return "GREEN";
        case TCS3200_COLOR_BLUE:
            return "BLUE";
        case TCS3200_COLOR_YELLOW:
            return "YELLOW";
        case TCS3200_COLOR_CYAN:
            return "CYAN";
        case TCS3200_COLOR_PURPLE:
            return "PURPLE";
        case TCS3200_COLOR_UNKNOWN:
            return "UNKNOWN";
        case TCS3200_COLOR_NONE:
        default:
            return "NONE";
    }
}

void TCS3200_DebugPrint(void)
{
    TCS3200_Raw_t raw;
    TCS3200_RGB_t rgb;
    TCS3200_Color_t color;

    TCS3200_ReadRaw(&raw, TCS3200_GATE_MS);
    TCS3200_RawToRGB(&raw, &rgb);
    color = s_white_ready ? TCS3200_Classify(&rgb) : TCS3200_COLOR_NONE;

    if (raw.R == 0u && raw.G == 0u && raw.B == 0u && raw.Clear == 0u) {
        printf("TCS3200: no signal (check wiring / power)\r\n");
        return;
    }

    printf("TCS3200 RAW R=%lu G=%lu B=%lu C=%lu | RGB %3u %3u %3u | %s\r\n",
           (unsigned long)raw.R,
           (unsigned long)raw.G,
           (unsigned long)raw.B,
           (unsigned long)raw.Clear,
           (unsigned int)rgb.R,
           (unsigned int)rgb.G,
           (unsigned int)rgb.B,
           TCS3200_ColorName(color));

    if (!s_white_ready) {
        printf("  (not calibrated: aim at white paper and call TCS3200_CalibrateWhite)\r\n");
    }
}
