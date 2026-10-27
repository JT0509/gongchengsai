#ifndef _key_H
#define _key_H

#include "main.h"

/* 物理按键丝印/接线与默认相反时，把 KEY_SWAP 改成 1：
   0 = KEY0=PE4（启动）、KEY1=PE3（急停）
   1 = KEY0=PE3（启动）、KEY1=PE4（急停）
   判断方法：Watch 里看 DBG_KEY0/DBG_KEY1，按物理 KEY0，
   变 0 的是哪个就用哪个；若变的是 DBG_KEY1，把这里改成 1。 */
#define KEY_SWAP 0

/* 按键有效电平：
   1 = 按下为低（按键接地，板载按键最常见）
   0 = 按下为高（按键接 VCC）
   判断方法：Watch 里看 GPIOE->IDR 的 bit3/bit4，
   松手时是高、按下变低 -> 1；松手是低、按下变高 -> 0。 */
#define KEY_ACTIVE_LOW 1

#if KEY_SWAP
#define START_FLAG GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) /* KEY0（PE3） */
#define KEY1_FLAG  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) /* KEY1（PE4） */
#else
#define START_FLAG GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) /* KEY0（PE4） */
#define KEY1_FLAG  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) /* KEY1（PE3） */
#endif

#if KEY_ACTIVE_LOW
#define KEY0_DOWN() (START_FLAG == 0)
#define KEY1_DOWN() (KEY1_FLAG == 0)
#else
#define KEY0_DOWN() (START_FLAG != 0)
#define KEY1_DOWN() (KEY1_FLAG != 0)
#endif

void START_Init(void); /* KEY0 初始化 */
void KEY1_Init(void);  /* KEY1 初始化 */

#endif