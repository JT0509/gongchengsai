#include "line_counter.h"

// 数线参数（实车标定）：
//   LC_DEBOUNCE：黑连续 N 次采样才认一条线（滤噪声毛刺）
//   LC_REARM   ：认完一条线后，需连续白 M 次才能认下一条（防一条线数两遍）
#define LC_DEBOUNCE 3
#define LC_REARM    10

typedef enum {
    LC_WAIT_EDGE = 0,   // 等白到黑上升沿
    LC_DEBOUNCE_ST,     // 消抖中（黑连续计数）
    LC_REARM_ST,        // 重装中（白连续计数）
} LcState;

static uint8_t lc_count;
static LcState lc_state;
static uint8_t lc_cnt;

void LineCounter_Init(void)
{
    // 出发场景：无条件等沿。脚下若是黑线（出发区大圆）也算第 1 条，不丢。
    lc_count = 0;
    lc_state = LC_WAIT_EDGE;
    lc_cnt   = 0;
}

void LineCounter_Init_Skip(void)
{
    // 转弯后场景：先等白，跳过脚下已数过的线（圆心 0 度线），再从下一条线数。
    lc_count = 0;
    lc_state = LC_REARM_ST;
    lc_cnt   = 0;
}

void LineCounter_Update(void)
{
    uint8_t sval = S_GW_Read();   // 1=黑 0=白（极性待标定）

    switch (lc_state) {
    case LC_WAIT_EDGE:                          // 等白到黑沿
        if (sval) { lc_state = LC_DEBOUNCE_ST; lc_cnt = 1; }
        break;

    case LC_DEBOUNCE_ST:                        // 消抖：黑要连续 N 次
        if (sval) {
            if (++lc_cnt >= LC_DEBOUNCE) {      // 确认一条线
                lc_count++;
                lc_state = LC_REARM_ST;
                lc_cnt   = 0;
            }
        } else {                                // 毛刺，回到等沿
            lc_state = LC_WAIT_EDGE;
            lc_cnt   = 0;
        }
        break;

    case LC_REARM_ST:                           // 重装：需连续白 M 次
        if (!sval) {
            if (++lc_cnt >= LC_REARM) {         // 足够白，可认下一条
                lc_state = LC_WAIT_EDGE;
                lc_cnt   = 0;
            }
        } else {
            lc_cnt = 0;                         // 还黑着，继续等白
        }
        break;
    }
}

uint8_t LineCounter_GetCount(void)
{
    return lc_count;
}
