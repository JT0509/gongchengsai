#include "steering.h"

// ★2026-10-29 夹爪硬件改过，张开量加大（原 1980µs≈133°）。
//   线性换算：1980→1320 共 660µs ≈59° → 约 11.2µs/°。
//   调法：不够就按 +60µs（≈+5.4°）一档加，建议不超过 2300（再大可能顶机械限位、堵转发热）。
//   ⚠ 张开行程变大后到位时间也略长：若上电起车时夹爪还没张到位，
//     把 nav.h 的 NAV_SERVO_OPEN_MS 从 600 加到 700。
//   ★2026-10-29 再张开 10°：2100 → 2210 → 2320（11.2µs/° × 10 ≈ 112µs）。
//     ⚠ 2320 已超过之前估的 2300 软上限，正在逼近机械限位。
//       上电后听/看夹爪：到头有"吱"的堵转声或齿条顶死抖动 → 立刻退回 2280。
//       堵转超过几秒会烧舵机，别长时间停在最开位置。
#define STEERING_OPEN_US   2320
#define STEERING_CLOSE_US  1320

u8 STEERING_FLAG = 0; //0:RESET 1:SET
u16 Steering_Delay = 0;

void Steering_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler = 168-1;
    TIM_TimeBaseStructure.TIM_Period = 20000-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM9, ENABLE);

    TIM_Cmd(TIM9, ENABLE);
}

// 夹爪「收紧」：1.32ms ≈ 74°（收紧=夹住物料）
void Steering_RESET(void)
{
    TIM_SetCompare1(TIM9, STEERING_CLOSE_US);
}

// 夹爪「张开」：2.10ms ≈ 144°（张开=物料进出工作位；原 1.98ms）
void Steering_SET(void)
{
    TIM_SetCompare1(TIM9, STEERING_OPEN_US);
}

void Engine_Working(void)
{
    if(Steering_Delay==0) 
	{
		Steering_SET();		
		Steering_Delay++;
	}
	else if(Steering_Delay==1000) 
	{
		STEERING_FLAG=0;
		Steering_Delay++;
	}
	else if(Steering_Delay==1001)
	{
		Steering_RESET();
		Steering_Delay++;
	}
	else if(Steering_Delay==1500)
	{
		STEERING_FLAG=0;
		Steering_Delay=0;
	}
	else Steering_Delay++;
}