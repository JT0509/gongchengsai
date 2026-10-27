#include "turn_counter.h"

// 角度计数器参数（实车标定）：
//   TC_DEBOUNCE：黑连续 N 次采样才认一条线（滤噪声毛刺）
//   TC_REARM   ：认完一条线后，需连续白 M 次才能认下一条（防一条线数两遍）
#define TC_DEBOUNCE 3
#define TC_REARM    10

typedef enum {
    TC_WAIT_EDGE = 0,   // 等白到黑上升沿
    TC_DEBOUNCE_ST,     // 消抖中（黑连续计数）
    TC_REARM_ST,        // 重装中（白连续计数）
} TcState;

static uint8_t tc_count;
static TcState tc_state;
static uint8_t tc_cnt;

void TurnCounter_Init(void)
{
    tc_count = 0;
    // 转弯开始时脚下压着一条放射线（黑），先等白跳过它，再从下一条线数。
    tc_state = TC_REARM_ST;
    tc_cnt   = 0;
}

void TurnCounter_Update(void)
{
    uint8_t sval = S_GW_Read();   // 1=黑 0=白

    switch (tc_state) {
    case TC_WAIT_EDGE:                          // 等白到黑沿
        if (sval) { tc_state = TC_DEBOUNCE_ST; tc_cnt = 1; }
        break;

    case TC_DEBOUNCE_ST:                        // 消抖：黑要连续 N 次
        if (sval) {
            if (++tc_cnt >= TC_DEBOUNCE) {      // 确认一条线
                tc_count++;
                tc_state = TC_REARM_ST;
                tc_cnt   = 0;
            }
        } else {                                // 毛刺，回到等沿
            tc_state = TC_WAIT_EDGE;
            tc_cnt   = 0;
        }
        break;

    case TC_REARM_ST:                           // 重装：需连续白 M 次
        if (!sval) {
            if (++tc_cnt >= TC_REARM) {         // 足够白，可认下一条
                tc_state = TC_WAIT_EDGE;
                tc_cnt   = 0;
            }
        } else {
            tc_cnt = 0;                         // 还黑着，继续等白
        }
        break;
    }
}

uint8_t TurnCounter_GetCount(void)
{
    return tc_count;
}
