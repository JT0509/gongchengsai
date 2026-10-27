#ifndef _turn_counter_H
#define _turn_counter_H

#include "main.h"

// 角度计数器：车在圆心原地旋转时，数侧方灰度扫过的放射线数，用于精确转角。
//   圆心是 8 条放射线的交点，相邻两条夹角 45°；原地转 N 条线 = N×45°。
//   与 line_counter（定位数线：直行横向穿线）是两个独立计数器，各管各的，互不干扰。
// 每 2ms 调一次 TurnCounter_Update（内部读 S_GW_Read，GPIO 极快）。

void TurnCounter_Init(void);       // 转弯开始：先等白，跳过脚下已压的那条线
void TurnCounter_Update(void);     // 数旋转扫过的放射线（白->黑沿）
uint8_t TurnCounter_GetCount(void);

#endif
