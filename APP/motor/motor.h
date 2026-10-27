#ifndef _motor_H
#define _motor_H

#include "main.h"

/* ============================================================================
   软 I2C 引脚选择（用于判断"读不到编码器"是 MCU 侧还是驱动板侧）
     0 = PF6(SCL) / PF5(SDA)  ← 当前接线
     1 = PD0(SCL) / PD1(SDA)  ← 备用组 A
     2 = PB0(SCL) / PB1(SDA)  ← 备用组 B
   改这个宏之后，【必须把驱动板的 SCL/SDA 两根线物理挪到新引脚】，
   否则连线不同、现象不变，白测一次。
   测完把值改回 0，并恢复原接线。
   ============================================================================ */
#define IIC_PIN_SET 0

#if   IIC_PIN_SET == 0
#define IIC_GPIO        GPIOF
#define IIC_GPIO_CLK    RCC_AHB1Periph_GPIOF
#define IIC_SCL_PIN_N   6
#define IIC_SDA_PIN_N   5
#define IIC_SCL         PFout(6)
#define IIC_SDA         PFout(5)
#define READ_SCL        PFin(6)
#define READ_SDA        PFin(5)
#elif IIC_PIN_SET == 1
#define IIC_GPIO        GPIOD
#define IIC_GPIO_CLK    RCC_AHB1Periph_GPIOD
#define IIC_SCL_PIN_N   0
#define IIC_SDA_PIN_N   1
#define IIC_SCL         PDout(0)
#define IIC_SDA         PDout(1)
#define READ_SCL        PDin(0)
#define READ_SDA        PDin(1)
#elif IIC_PIN_SET == 2
#define IIC_GPIO        GPIOB
#define IIC_GPIO_CLK    RCC_AHB1Periph_GPIOB
#define IIC_SCL_PIN_N   0
#define IIC_SDA_PIN_N   1
#define IIC_SCL         PBout(0)
#define IIC_SDA         PBout(1)
#define READ_SCL        PBin(0)
#define READ_SDA        PBin(1)
#else
#error "IIC_PIN_SET 只能是 0 / 1 / 2"
#endif

/* GPIO_Init 用的引脚掩码（GPIO_Pin_x 就是 1<<x） */
#define IIC_SCL_PIN_MASK (1u << IIC_SCL_PIN_N)
#define IIC_SDA_PIN_MASK (1u << IIC_SDA_PIN_N)

#define CAM_DEFAULT_I2C_ADDRESS     0x34 // I2C地址
#define MOTOR_TYPE_ADDR             0x14 // 编码电机类型设置寄存器地址
#define MOTOR_FIXED_SPEED_ADDR      0x33 // 速度寄存器地址。属于闭环控制
#define MOTOR_ENCODER_POLARITY_ADDR 0x15 // 电机编码方向极性地址
#define MOTOR_FIXED_PWM_ADDR        0x1F // 固定PWM控制地址，属于开环控制
#define MOTOR_ENCODER_TOTAL_ADDR    0x3C // 4个编码电机各自的总脉冲值
#define ADC_BAT_ADDR                0x00 // 电压地址

// 电机类型具体地址
#define MOTOR_TYPE_WITHOUT_ENCODER 0 // 无编码器的电机,磁环每转是44个脉冲减速比:90  默认
#define MOTOR_TYPE_TT              1 // TT编码电机
#define MOTOR_TYPE_N20             2 // N20编码电机
#define MOTOR_TYPE_JGB             3 // 磁环每转是44个脉冲   减速比:90  默认

typedef enum {
    STOP,
    FRONT,
    BACK,
    RIGHT,
    LEFT,
    REVOLVE
} DIRECTION_MODE;

typedef struct
{
    float Target_Speed;
    float Real_Speed;
    float PID_Output;
} Motor_InitTypeDef;

extern Motor_InitTypeDef Motor_201, Motor_202, Motor_203, Motor_204; // 电机结构体
// === Motor direction polarity: whole sign reversed so +speed = forward ===
#define MOTOR_DIR_201 (-1)  // left-front
#define MOTOR_DIR_202 (+1)  // right-front
#define MOTOR_DIR_203 (-1)  // right-back
#define MOTOR_DIR_204 (+1)  // left-back

extern DIRECTION_MODE Direction;

// 速度写入 I2C 失败（NACK）累计次数：Cortex-Debug Watch 直接看，正常应≈0
extern volatile uint16_t DBG_MOTOR_NACK;

void IIC_init(void);
uint8_t I2C_Read_Byte(uint8_t Reg, uint8_t *Val);
uint8_t I2C_Read_Len(uint8_t Reg, uint8_t *Buf, uint8_t Len);
uint8_t I2C_Write_Len(uint8_t Reg, int8_t *Buf, uint8_t Len);
void MOTOR_Init(void);
// 返回 0=整帧写入成功、1=重试+总线恢复后仍失败（板子会保持上一次速度）
uint8_t MOTOR_CONTROL_DATA_Predit(Motor_InitTypeDef *M1, Motor_InitTypeDef *M2, Motor_InitTypeDef *M3, Motor_InitTypeDef *M4);
void IIC_Bus_Recover(void);
void MOTOR_ENCODER_DATA_Predit(void);

#endif