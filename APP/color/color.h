#ifndef _color_H
#define _color_H

#include "main.h"

// 颜色识别阈值定义
#define WHITE_THRESHOLD 240    // 白色识别阈值 (0-255)
#define BLACK_THRESHOLD 60     // 黑色识别阈值 (0-255) - 提高阈值
#define BLACK_MAX_VAL 70       // 黑色最大值限制
#define COLOR_DIFF_THRESH 20   // 颜色差异阈值 (红、绿、蓝识别灵敏度)

#define white 1
#define black 2
#define red 3
#define green 4
#define blue 5

uint8_t max3(uint8_t a, uint8_t b, uint8_t c);
uint8_t min3(uint8_t a, uint8_t b, uint8_t c);

extern volatile uint8_t frame_buffer[8]; // 改为8字节缓冲区
extern volatile uint8_t data_ready;

void USART_SendByte(USART_TypeDef* USARTx, u8 data);
void USART_SendArray(USART_TypeDef* USARTx, u8 *array, u16 length);
void USART_SendString(USART_TypeDef* USARTx, char *str);
void color_init(void);
int color_get(void);
void USART3_Init(u32 bound);

#endif
