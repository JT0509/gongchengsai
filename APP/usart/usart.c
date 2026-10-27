#include "usart.h"

/* 串口中断触发次数：Watch 里看。数值暴涨 = RX 在持续触发中断，
   会把主循环饿死（车不停、按键失效）。现已关闭 RX 中断，正常应恒为 0。 */
volatile uint32_t DBG_USART_IRQ = 0;

int fputc(int ch,FILE *p)
{
	USART_SendData(USART1,(u8)ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}

void USART1_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate=bound;
	USART_InitStructure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
	USART_ClearFlag(USART1,USART_FLAG_TC);
	/* 2026-10-25：关闭 RX 中断（原代码开了 RXNE 中断，ISR 里还有一句
	   while(TC!=SET) 无超时死等）。本工程没有串口调试，PA10 悬空时会被噪声
	   持续触发 RXNE，主循环被 ISR 反复抢占 → 表现为「车一直跑、按键全失效」。
	   确实需要串口接收时，再打开中断并把 ISR 里的死等去掉。 */
	/* USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure); */
	(void)NVIC_InitStructure;
}

/* 保留 ISR（向量表用），但去掉阻塞等待：即使以后误开中断也不会卡死主循环 */
void USART1_IRQHandler(void)
{
	DBG_USART_IRQ++;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{
		(void)USART_ReceiveData(USART1); // 读一下清标志，不回显、不等待
	}
	USART_ClearFlag(USART1,USART_FLAG_TC);
}
