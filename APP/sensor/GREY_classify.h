#ifndef _GREY_classify_H
#define _GREY_classify_H

#include <stdint.h>

// 线型档位：二值灰度 8bit → 7 档离散状态（场景判定用）
typedef enum {
    LINE_LOST = 0,   // 全白丢线
    LINE_LEFT2,      // 线偏左 2 档
    LINE_LEFT1,      // 线偏左 1 档
    LINE_CENTER,     // 线居中（死区）
    LINE_RIGHT1,     // 线偏右 1 档
    LINE_RIGHT2,     // 线偏右 2 档
    LINE_JUNCTION,   // 全黑（路口/圆心/宽线）
} LineState;

// 边缘法线中心（×10，0~70，中心 35）。-1=全白丢线，-2=全黑
// data: 8bit 灰度（黑=1 白=0），bit0=车右（探头1）… bit7=车左（探头8）
static inline int16_t Grey_Center(uint8_t data)
{
    int left = -1, right = -1, count = 0;
    int i;
    for (i = 0; i < 8; i++) {
        if ((data >> (7 - i)) & 0x01) {   // i=0=bit7（车左）… i=7=bit0（车右）
            count++;
            if (left < 0) left = i;
            right = i;
        }
    }
    if (count == 0) return -1;   // 全白
    if (count == 8) return -2;   // 全黑
    return (int16_t)((left + right) * 5);   // (left+right)/2 ×10
}

// 线型档位：把线中心位置 c（0=最左、35=居中、70=最右）离散成 5 档
// 对称分档（以 35 为轴）：LEFT2↔RIGHT2(3档)、LEFT1↔RIGHT1(2档)、CENTER(5档)
static inline LineState Grey_Classify(uint8_t data)
{
    int16_t c = Grey_Center(data);
    if (c == -1) return LINE_LOST;      // 全白丢线
    if (c == -2) return LINE_JUNCTION;  // 全黑（路口/宽线）
    if (c < 15)  return LINE_LEFT2;     // 0,5,10    线最左
    if (c < 25)  return LINE_LEFT1;     // 15,20     线偏左
    if (c <= 45) return LINE_CENTER;    // 25~45     线居中（死区 ±10）
    if (c <= 55) return LINE_RIGHT1;    // 50,55     线偏右
    return LINE_RIGHT2;                 // 60,65,70  线最右
}

#endif
