#ifndef _GREY_driver_H
#define _GREY_driver_H

#include "main.h"

// 八路灰度红外传感器（官方 IR 灰度，硬件 I2C）
// 设备地址 0x12，从寄存器 0x30 读 1 字节，bit7~bit0 = 探头1~8（bit7=探头1 最左）
#define GW_GRAY_ADDR_DEF 0x12 // 设备地址
#define GW_GRAY_DATA     0x30 // 数据寄存器

// ----------------------------------------------------------------------------
// 灰度黑白极性开关（GW_BLACK_IS_ONE）——「不能循迹」时优先查这里！
//   1 = 对寄存器取反后当作「黑=1」（适用于传感器直出黑=0 的板子）
//   0 = 寄存器直出，直接当作「黑=1」（适用于传感器直出黑=1 的板子）
// 【判定方法】把车摆在**白色地面**上不压线，用调试器看 DBG_pos：
//     正确 → 读到 -1（全白），压上黑线读到 0~70
//     反了 → 白底读到 -2（全黑）、压线读到 -1（全白）
// 已确认当前板子应取反，保持为 1
// ----------------------------------------------------------------------------
#define GW_BLACK_IS_ONE 1

// 硬件 I2C 公共接口（grey_i2c.c）
void Grey_I2C1_Init(void); // I2C1（PB6=SCL / PB7=SDA，前灰度）
void Grey_I2C2_Init(void); // I2C2（PB10=SCL / PB11=SDA，后灰度）
uint8_t Grey_I2C_ReadReg(I2C_TypeDef *I2Cx, uint8_t reg);

// 灰度传感器接口
void F_GW_digital_Init(void);
uint8_t F_GW_digital_Read(void);
void B_GW_digital_Init(void);
uint8_t B_GW_digital_Read(void);

int16_t F_GW_digital_eight(void);
int16_t B_GW_digital_eight(void);

// 线质心位置（×10）：0~70，中心 35；-1=全白丢线，-2=全黑（路口/宽线）
int16_t F_GW_position(void);
int16_t B_GW_position(void);

#endif
