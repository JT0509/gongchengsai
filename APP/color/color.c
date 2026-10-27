#include "color.h"

// 颜色传感器数据
u8 color_R = 0, color_G = 0, color_B = 0;
u8 color_detected = 0;

volatile uint8_t frame_index = 0;
volatile uint8_t frame_started = 0;

volatile uint8_t frame_buffer[8] = {0}; // 改为8字节缓冲区
volatile uint8_t data_ready = 0;

void color_init(void)
{
		uint8_t led_cmd[] = {0xA5, 0x65, 0x0A}; // 0x65: 亮度等级5 (0-10, 0最亮)
//		uint8_t balance_cmd[] = {0xA5, 0xBB, 0x60};
		uint8_t config_cmd[] = {0xA5, 0x81, 0x26}; // 0x81: 自动输出，RGB=1
		
    USART_SendArray(USART3, led_cmd, sizeof(led_cmd));
    delay_ms(200);
//    USART_SendArray(USART3, balance_cmd, sizeof(balance_cmd));
//		delay_ms(3000);
    USART_SendArray(USART3, config_cmd, sizeof(config_cmd));
    delay_ms(200);
}
int color_get(void)
{
		uint32_t counter = 0;
		int color=0;
	
		int max=0;
		int white1=0;
		int red1=0;
		int green1=0;
		int blue1=0;
		int black1=0;
	
	 while(counter<10) {

        if(data_ready) {
            // 检查数据类型是否为0x45（处理后RGB）
            if(frame_buffer[2] == 0x45) {
                // 解析颜色数据 (位置4=R, 5=G, 6=B)
                uint8_t R = frame_buffer[4];
                uint8_t G = frame_buffer[5];
                uint8_t B = frame_buffer[6];
                               
                // 计算RGB最大值和最小值
                uint8_t max_val = max3(R, G, B);
                uint8_t min_val = min3(R, G, B);
                
                // 优化后的颜色识别 - 只识别红、黑、绿、白、蓝
                // 1. 先判断白色（高灵敏度）
                if(R >= WHITE_THRESHOLD && G >= WHITE_THRESHOLD && B >= WHITE_THRESHOLD) {
                   // printf(" => White R=%d G=%d B=%d\r\n", R, G, B);
									//color=white;
									white1++;
                } 
                // 2. 判断黑色（增强识别逻辑）
                else if(max_val <= BLACK_MAX_VAL && min_val <= BLACK_THRESHOLD) {
                    // 增强黑色识别：最大值不超过70，最小值不超过60
                    //printf(" => Black R=%d G=%d B=%d (max=%d, min=%d)\r\n", R, G, B, max_val, min_val);
								//	color=black;
									black1++;
                }
                // 3. 判断红色（提高灵敏度）
                else if(R > G + COLOR_DIFF_THRESH && R > B + COLOR_DIFF_THRESH) {
                    //printf(" => Red R=%d G=%d B=%d\r\n", R, G, B);
									//color=red;
									red1++;
                }
                // 4. 判断绿色
                else if(G > R + COLOR_DIFF_THRESH && G > B + COLOR_DIFF_THRESH) {
                    //printf(" => Green R=%d G=%d B=%d\r\n", R, G, B);
									//color=green;
									green1++;
                }
                // 5. 判断蓝色
                else if(B > R + COLOR_DIFF_THRESH && B > G + COLOR_DIFF_THRESH) {
                    //printf(" => Blue R=%d G=%d B=%d\r\n", R, G, B);
									//color=blue;
									blue1++;
                }
                // 6. 其他情况视为未知
                else {
                    printf(" => Unknown R=%d G=%d B=%d (max=%d, min=%d)\r\n", R, G, B, max_val, min_val);
                }
            } else {
                //printf(" => Unhandled frame type: 0x%02X\r\n", frame_buffer[2]);
            }
            data_ready = 0;
        }
        // 添加小延时防止主循环跑飞
        delay_ms(10);
        counter++;
    }
			max=white1;
			color=white;
		if (red1 > max) {
        max = red1;
				color=red;
    }
    if (green1 > max) {
        max = green1;
				color=green;
    }
    if (blue1 > max) {
        max = blue1;
				color=blue;
    }
    if (black1 > max) {
        max = black1;
				color=black;
    }

	 return color;
}
// 计算RGB最大值
uint8_t max3(uint8_t a, uint8_t b, uint8_t c) {
    uint8_t m = a;
    if (b > m) m = b;
    if (c > m) m = c;
    return m;
}

// 计算RGB最小值
uint8_t min3(uint8_t a, uint8_t b, uint8_t c) {
    uint8_t m = a;
    if (b < m) m = b;
    if (c < m) m = c;
    return m;
}

void USART_SendByte(USART_TypeDef* USARTx, u8 data) {
    USART_SendData(USARTx, data);
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
}
void USART_SendArray(USART_TypeDef* USARTx, u8 *array, u16 length) 
	{
		u16 i=0;
    for(; i < length; i++) 
		{
        USART_SendByte(USARTx, array[i]);
    }
}


// 发送字符串
void USART_SendString(USART_TypeDef* USARTx, char *str) {
    while(*str) {
        USART_SendByte(USARTx, *str++);
    }
}

void USART3_Init(u32 bound) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    
    // 配置TX引脚(PB10)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 配置RX引脚(PB11)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // 配置USART3
    USART_InitStruct.USART_BaudRate = bound;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStruct);
    
    // 配置中断（优先级比USART1低）
    NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void) {
    static uint8_t rx_buffer[8]; // 存储接收数据的缓冲区
    static uint8_t frame_index = 0; // 缓冲区索引
		int i=0;  
	
    if(USART_GetITStatus(USART3, USART_IT_RXNE)) 
			{
        uint8_t data = USART_ReceiveData(USART3);
        
        // 帧头检测
        if(frame_index == 0) {
            if(data == 0x5A) {
                rx_buffer[frame_index++] = data;
            }
        } 
        else if(frame_index == 1) {
            if(data == 0x5A) {
                rx_buffer[frame_index++] = data;
            } else {
                frame_index = 0; // 帧头错误，重置
            }
        } 
        else {
            // 存储数据到缓冲区
            rx_buffer[frame_index++] = data;
            
            // 检查是否收到完整帧 (8字节)
            if(frame_index >= 8) {
                // 验证数据类型 (0x45)
                if(rx_buffer[2] == 0x45) {
                    // 复制到全局缓冲区
                    for(; i < 8; i++) {
                        frame_buffer[i] = rx_buffer[i];
                    }
                    data_ready = 1;
                }
                // 重置接收状态
                frame_index = 0;
            }
        }
        
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}
