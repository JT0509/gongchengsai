#include "S_GREY_driver.h"

// 侧方灰度初始化：PD12 上拉输入（DO 数字输出，读到高/低电平）
void S_GW_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;   // 上拉输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

// 读侧方灰度：约定 1=黑线、0=白
// 【极性待实车标定】若实际黑线输出低电平，把下面 Bit_SET 改成 Bit_RESET
uint8_t S_GW_Read(void)
{
    return (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12) == Bit_SET) ? 1u : 0u;
}
