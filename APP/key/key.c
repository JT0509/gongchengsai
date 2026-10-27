#include "key.h"

// KEY0 初始化（PE4）
void START_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#if KEY_ACTIVE_LOW
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 按下接地 -> 上拉
#else
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // 按下接 VCC -> 下拉
#endif
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

// KEY1 初始化（PE3）
void KEY1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
#if KEY_ACTIVE_LOW
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 按下接地 -> 上拉
#else
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // 按下接 VCC -> 下拉
#endif
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}