#include "GREY_driver.h"

// ============================================================================
// 八路灰度硬件 I2C 驱动（标准外设库）
//   I2C1: PB6(SCL) / PB7(SDA)   → 前灰度
//   I2C2: PB10(SCL) / PB11(SDA) → 后灰度
//   设备地址 0x12，读寄存器 0x30（见 GREY_driver.h）
// ============================================================================

// I2C1 GPIO 初始化（PB6/PB7 → AF4，开漏，上拉）
static void Grey_I2C1_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
}

// I2C2 GPIO 初始化（PB10/PB11 → AF4，开漏，上拉）
static void Grey_I2C2_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
}

// 前灰度 I2C1 初始化（100kHz）
void Grey_I2C1_Init(void)
{
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    Grey_I2C1_GPIO_Init();

    I2C_InitStructure.I2C_ClockSpeed          = 100000;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStructure);

    I2C_Cmd(I2C1, ENABLE);
}

// 后灰度 I2C2 初始化（100kHz）
void Grey_I2C2_Init(void)
{
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    Grey_I2C2_GPIO_Init();

    I2C_InitStructure.I2C_ClockSpeed          = 100000;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2, &I2C_InitStructure);

    I2C_Cmd(I2C2, ENABLE);
}

// ----------------------------------------------------------------------------
// 带超时的等待：防总线异常/从机不应答时 while 死循环 —— 这是车「卡死」的常见原因
// 返回 0=等到事件，1=超时
// ----------------------------------------------------------------------------
#define GW_I2C_TIMEOUT 100000u

static uint8_t I2C_WaitEvt(I2C_TypeDef *I2Cx, uint32_t event)
{
    uint32_t to = GW_I2C_TIMEOUT;

    while (!I2C_CheckEvent(I2Cx, event)) {
        if (--to == 0) return 1; // 超时
    }
    return 0;
}

// 硬件 I2C 读寄存器（阻塞轮询，全程带超时）：
// 先写寄存器地址，repeated-start 后读 1 字节。返回 bit7~bit0 = 探头1~8。
// 异常时返回 0（取反后 = 0xFF 全黑）→ 循线会走「全黑直行」安全分支，不会抳死
uint8_t Grey_I2C_ReadReg(I2C_TypeDef *I2Cx, uint8_t reg)
{
    uint8_t data = 0;
    uint32_t to;

    // 等总线空闲（带超时）
    to = GW_I2C_TIMEOUT;
    while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) {
        if (--to == 0) return 0;
    }

    // 写设备地址 + 寄存器地址
    I2C_GenerateSTART(I2Cx, ENABLE);
    if (I2C_WaitEvt(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) return 0;

    I2C_Send7bitAddress(I2Cx, GW_GRAY_ADDR_DEF << 1, I2C_Direction_Transmitter);
    if (I2C_WaitEvt(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2Cx, ENABLE);
        return 0;
    }

    I2C_SendData(I2Cx, reg);
    if (I2C_WaitEvt(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
        I2C_GenerateSTOP(I2Cx, ENABLE);
        return 0;
    }

    // repeated-start + 读设备地址 + 读 1 字节
    I2C_GenerateSTART(I2Cx, ENABLE);
    if (I2C_WaitEvt(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {
        I2C_GenerateSTOP(I2Cx, ENABLE);
        return 0;
    }

    I2C_Send7bitAddress(I2Cx, GW_GRAY_ADDR_DEF << 1, I2C_Direction_Receiver);
    if (I2C_WaitEvt(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        I2C_GenerateSTOP(I2Cx, ENABLE);
        return 0;
    }

    // 读 1 字节（最后字节）：先 NACK + 提前 STOP
    I2C_AcknowledgeConfig(I2Cx, DISABLE);
    I2C_GenerateSTOP(I2Cx, ENABLE);
    if (I2C_WaitEvt(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) return 0;
    data = I2C_ReceiveData(I2Cx);

    I2C_AcknowledgeConfig(I2Cx, ENABLE); // 必须恢复 ACK，否则下次读取可能异常

    return data;
}