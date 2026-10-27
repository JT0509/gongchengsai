#ifndef _line_counter_H
#define _line_counter_H

#include "main.h"

// 侧方灰度数线（边沿检测 + 消抖 + 最小间隔重装）
// 用途：按「线序」定位，不靠距离
//   出发区到圆心：第 5 条线（0 度线）= 圆心
//   圆心到 A：第 2 条线（小圆线）= A 点
// 每 2ms 调一次 LineCounter_Update（内部读 S_GW_Read，GPIO 极快）

void LineCounter_Init(void);       // 出发：从脚下开始数（脚下黑线算第 1 条）
void LineCounter_Init_Skip(void);  // 转弯后：跳过脚下已数过的线，先等白
void LineCounter_Update(void);
uint8_t LineCounter_GetCount(void);

#endif
