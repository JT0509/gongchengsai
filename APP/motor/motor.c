#include "motor.h"

Motor_InitTypeDef Motor_201, Motor_202, Motor_203, Motor_204;
int8_t MOTOR_Data[4];
int8_t MotorEncoderPolarity; // 电机极性控制变量
uint32_t EncodeTotal[4];      // 用于暂存电机累积转动量的值，正转递增，反转递减
int8_t MotorType;            // 设置电机类型
DIRECTION_MODE Direction;
volatile uint16_t DBG_MOTOR_NACK = 0; // 速度写入 I2C 失败次数（Watch 观察，正常应≈0）

/* I2C 读失败的阶段统计（定位「读编码器为什么失败」用）：
     DBG_NACK_AW → 发 地址+写 就没应答（板子没响应/总线卡死）
     DBG_NACK_RG → 发 寄存器地址 没应答（寄存器地址非法）
     DBG_NACK_AR → 发 地址+读 没应答（板子不接受读请求）
     DBG_ACK_TMO → 应答超时总次数（上面三项之和）
     DBG_READ_OK → 整帧读成功次数（和 DBG_ACK_TMO 对照看成功率） */
volatile uint16_t DBG_NACK_AW = 0;
volatile uint16_t DBG_NACK_RG = 0;
volatile uint16_t DBG_NACK_AR = 0;
volatile uint16_t DBG_ACK_TMO = 0;
volatile uint16_t DBG_READ_OK = 0;

/* 写路径的阶段统计（空闲时每秒约 100 帧停车命令，失败会很显眼） */
volatile uint16_t DBG_WNACK_AW  = 0; // 写：地址阶段没应答
volatile uint16_t DBG_WNACK_RG  = 0; // 写：寄存器地址阶段没应答
volatile uint16_t DBG_WNACK_DAT = 0; // 写：数据字节阶段没应答

/* SDA 引脚自环测试结果（上电时做一次）：
     0x00 = 还没测
     0xA5 = 正常：驱动 0 读回 0，驱动 1 读回 1（引脚和读函数都没问题）
     0x5A = 异常：读数与驱动值不符（引脚被外部钳住 / 读的不是这根脚） */
volatile uint8_t DBG_SDA_LOOP = 0;

/* ============================================================================
   硬件病因判别变量（上电时各做一次，全是纯 GPIO 操作，不碰 I2C 时序）
   ----------------------------------------------------------------------------
      DBG_IDLE_SDA / DBG_IDLE_SCL
          还没发任何 I2C 之前的空闲电平（输入+上拉）。正常都应为 1。
          = 0 → 线被别的东西拉死（短路 / 从机卡住 / 接错脚）

      DBG_SDA_PD / DBG_SCL_PD
          把线配成「输入 + 内部下拉」再读：内部下拉约 40kΩ。
            读到 1 → 外部上拉比 40k 强（外部有上拉电阻，且线是通的）
            读到 0 → 外部没有上拉（或弱于 40k）→ 缺上拉
          这一项直接回答「上拉电阻够不够」。

      DBG_SDA_RISE[6] / DBG_SCL_RISE[6]
          把线拉低 → 释放成「输入+上拉」→ 在 1/2/5/10/20/50µs 各读一次电平。
          用来量上升时间：
            第 0 个(1µs) 就是 1        → 上拉很强，线没问题
            5~20µs 才变 1              → 上拉偏弱（长线 / 接触不良 / 阻值太大）
            50µs 仍是 0                → 几乎没有上拉，或线被拉死
   ============================================================================ */
volatile uint8_t DBG_IDLE_SDA    = 0;
volatile uint8_t DBG_IDLE_SCL    = 0;
volatile uint8_t DBG_SDA_PD      = 0;
volatile uint8_t DBG_SCL_PD      = 0;
volatile uint8_t DBG_SDA_RISE[6] = {0};
volatile uint8_t DBG_SCL_RISE[6] = {0};

// 把 SCL(is_scl=1) 或 SDA(is_scl=0) 配成输入；pullup=1 内部上拉，0 内部下拉
static void IIC_Line_Input(uint8_t is_scl, uint8_t pullup)
{
    GPIO_InitTypeDef g;

    GPIO_StructInit(&g);
    g.GPIO_Pin   = is_scl ? (uint16_t)IIC_SCL_PIN_MASK : (uint16_t)IIC_SDA_PIN_MASK;
    g.GPIO_Mode  = GPIO_Mode_IN;
    g.GPIO_PuPd  = pullup ? GPIO_PuPd_UP : GPIO_PuPd_DOWN;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_GPIO, &g);
}

// 把 SCL(is_scl=1) 或 SDA(is_scl=0) 配成推挽输出
static void IIC_Line_Output(uint8_t is_scl)
{
    GPIO_InitTypeDef g;

    GPIO_StructInit(&g);
    g.GPIO_Pin   = is_scl ? (uint16_t)IIC_SCL_PIN_MASK : (uint16_t)IIC_SDA_PIN_MASK;
    g.GPIO_Mode  = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_GPIO, &g);
}

// 上升时间扫描：拉低 → 释放 → 分别在 1/2/5/10/20/50µs 读电平，结果写 out[6]
static void IIC_Rise_Scan(uint8_t is_scl, volatile uint8_t *out)
{
    static const uint16_t dly[6] = {1, 2, 5, 10, 20, 50};
    uint8_t k;

    for (k = 0; k < 6; k++) {
        IIC_Line_Output(is_scl); // 推挽拉低，把线彻底放干
        if (is_scl) IIC_SCL = 0;
        else        IIC_SDA = 0;
        delay_us(50);

        IIC_Line_Input(is_scl, 1); // 释放：只剩上拉（内部 40k + 外部）
        delay_us(dly[k]);
        if (is_scl) out[k] = READ_SCL ? 1 : 0;
        else        out[k] = READ_SDA ? 1 : 0;

        delay_us(50); // 让线完全稳定，再进入下一轮
    }
}

void I2C_SDA_OUT(void) // SDA输出方向配置
{
    IIC_Line_Output(0);
}

void I2C_SDA_IN(void) // SDA输入方向配置
{
    IIC_Line_Input(0, 1); // 输入 + 上拉
}

// 以下为模拟IIC总线函数
void IIC_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(IIC_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = (uint16_t)(IIC_SDA_PIN_MASK | IIC_SCL_PIN_MASK);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(IIC_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(IIC_GPIO, (uint16_t)(IIC_SDA_PIN_MASK | IIC_SCL_PIN_MASK));
}

void IIC_start() // 起始信号
{
    I2C_SDA_OUT();
    IIC_SDA = 1;
    IIC_SCL = 1;
    delay_us(5);
    IIC_SDA = 0;
    delay_us(5);
    IIC_SCL = 0;
}

void IIC_stop() // 终止信号
{
    I2C_SDA_OUT();
    IIC_SCL = 0;
    IIC_SDA = 0;
    delay_us(5);
    IIC_SCL = 1;
    IIC_SDA = 1;
    delay_us(5);
}

// 主机发送应答信号
void IIC_ack()
{
    IIC_SCL = 0;
    I2C_SDA_OUT();
    IIC_SDA = 0;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

// 主机发送非应答信号
void IIC_noack()
{
    IIC_SCL = 0;
    I2C_SDA_OUT();
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

/* 应答等待超时次数。
   【2026-10-25 定论】实测这块驱动板的 I2C 从机**几乎从不应答 ACK**
   （读 21 次全 NACK、写每帧 5 次重试全 NACK），但数据本身是能正常收发的
   ——车能跑能停就是证明。官方例程也是 `IIC_wait_ack();` 直接忽略返回值。
   所以这里只统计、不中止事务：超时后**不拉 STOP**，让后续字节继续走完。 */
#define IIC_ACK_TIMEOUT 600

// 等待从机应答信号
// 返回值：1 表示没等到应答（仅作统计，调用方不再据此中止事务）
//        0 表示应答成功
u8 IIC_wait_ack()
{
    u16 tempTime = 0; // 原来是 u8，装不下 250 以上的超时值（会回绕）
    I2C_SDA_IN();
    IIC_SDA = 1;
    delay_us(1);
    IIC_SCL = 1;
    delay_us(1);

    while (READ_SDA) {
        tempTime++;
        if (tempTime > IIC_ACK_TIMEOUT) {
            /* 关键：这里绝对不能 IIC_stop()。
               一旦在事务中途发 STOP，后面的字节全废；而这块板子本来就不应答，
               等于每次读都必然作废。让 SCL 保持高、交给下一个字节拉低即可。 */
            DBG_ACK_TMO++;
            IIC_SCL = 0;
            return 1;
        }
    }

    IIC_SCL = 0;
    return 0;
}

void IIC_send_byte(u8 txd)
{
    u8 i = 0;
    I2C_SDA_OUT();
    IIC_SCL = 0;
    ; // 拉低时钟开始传输数据
    for (i = 0; i < 8; i++) {
        IIC_SDA = (txd & 0x80) >> 7; // 取最高位
        txd <<= 1;
        delay_us(2);
        IIC_SCL = 1;
        delay_us(2); // 保持时钟
        IIC_SCL = 0;
        delay_us(2);
    }
}

// 读取一个字节
u8 IIC_read_byte(u8 ack)
{
    u8 i = 0, receive = 0;
    I2C_SDA_IN();
    for (i = 0; i < 8; i++) {
        IIC_SCL = 0;
        delay_us(2);
        IIC_SCL = 1;
        /* 这里与厂商例程完全一致：拉高后立即采样。之前加过 delay_us(2)，
           但与官方可直接跑通的参考代码不符，已撤回，保持与参考实现一字不差。 */
        receive <<= 1; // 左移
        if (READ_SDA)
            receive++; // 读取数据位
        delay_us(1);
    }

    if (!ack)
        IIC_noack();
    else
        IIC_ack();

    return receive; // 返回读取到的字节
}

// 读一个字节（一次完整事务：Start→地址+W→Reg→Start→地址+R→1字节→Stop）
// 注：这块板子不支持按字节地址逐条读，仅保留给诊断。
// 与官方例程一致：**ACK 只统计、不中止**。
uint8_t I2C_Read_Byte(uint8_t Reg, uint8_t *Val)
{
    IIC_start();
    IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0); // 地址+写
    if (IIC_wait_ack() == 1) DBG_NACK_AW++;
    IIC_send_byte(Reg); // 寄存器地址
    if (IIC_wait_ack() == 1) DBG_NACK_RG++;
    IIC_start();
    IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 1); // 地址+读
    if (IIC_wait_ack() == 1) DBG_NACK_AR++;
    *Val = IIC_read_byte(0);
    IIC_stop();
    return 0;
}

// 读取多个字节：Reg 起始寄存器地址  Buf 存放数据  Len 数据长度
//
// 【2026-10-25 第十五轮：改成官方例程的「两条独立事务」写法】
// 官方 Arduino / STM32 例程的 readFromAddr 是：
//     WireWriteByte(reg);                 // 事务1：只写寄存器地址，然后 STOP
//     Wire.requestFrom(I2C_ADDR, len);    // 事务2：START + 地址+读 + 连读 len 字节
// 两段之间是**真的 STOP**，不是重复起始（repeated START）。
// 我们之前一直用「START + 地址W + reg + 重复起始 + 地址R + 读」一条龙，
// 只有这一处和官方不同 —— 现在按官方分开，排除掉这个差异。
//
// ACK 仍然只统计、不中止：这块板子基本不应答，但字节内容可能是好的
// （官方例程同样忽略 ACK）。返回值恒 0，有效性由调用方看数据内容判断
// （全 0xFF = 总线悬空）。
uint8_t I2C_Read_Len(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    uint8_t i;
    uint8_t clean = 1;

    /* ---- 事务1：写寄存器地址，然后 STOP（和官方 WireWriteByte 一致）---- */
    IIC_start();
    IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0); // 地址+写
    if (IIC_wait_ack() == 1) {
        DBG_NACK_AW++;
        clean = 0;
    }
    IIC_send_byte(Reg); // 寄存器地址
    if (IIC_wait_ack() == 1) {
        DBG_NACK_RG++;
        clean = 0;
    }
    IIC_stop();
    delay_us(50); // 给从机时间挂上读指针（官方靠两次传输之间的间隔）

    /* ---- 事务2：直接发「地址+读」，连续读 Len 字节 ---- */
    IIC_start();
    IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 1); // 地址+读
    if (IIC_wait_ack() == 1) {
        DBG_NACK_AR++;
        clean = 0;
    }
    for (i = 0; i < Len; i++) {
        Buf[i] = IIC_read_byte((u8)(i != Len - 1)); // 最后一字节回 NACK
    }
    IIC_stop();

    if (clean) DBG_READ_OK++; // 全程有应答的「干净」帧个数（仅供对照）
    return 0;                 // 事务已完成，字节内容照常使用
}

// 循环写入多个数据：Reg寄存器地址  Buf写入数据  Len数据长度
uint8_t I2C_Write_Len(uint8_t Reg, int8_t *Buf, uint8_t Len) // I2C写数据函数
{
    uint8_t i;
    uint8_t clean = 1;
    IIC_start();                                       // 发送起始信号，随后发送7位从机地址+1位方向位，0表示写、1表示读
    IIC_send_byte((CAM_DEFAULT_I2C_ADDRESS << 1) | 0); // 发送设备地址+写操作
    if (IIC_wait_ack() == 1) {                         // ACK 不中止：整帧必须发完，板子才认这一帧
        DBG_WNACK_AW++;
        clean = 0;
    }
    IIC_send_byte(Reg); // 发送寄存器地址
    if (IIC_wait_ack() == 1) {
        DBG_WNACK_RG++;
        clean = 0;
    }
    for (i = 0; i < Len; i++) // 循环len次写数据
    {
        IIC_send_byte(Buf[i]); // 发送第i位的8位数据
        if (IIC_wait_ack() == 1) {
            DBG_WNACK_DAT++;
            clean = 0;
        }
    }
    IIC_stop(); // 发送结束，发送停止信号

    /* 返回值只作「这一帧有没有被确认」的状态，供 MOTOR_CONTROL_DATA_Predit 决定
       要不要补总线恢复 + 重发。
       注意：ACK 失败也**不中止事务**（字节照常发完），所以即使返回 1，
       板子仍可能收到了这一帧——这是为了同时满足
         (a) 板子不应答时数据也不能被丢弃（官方例程同样忽略 ACK）
         (b) 真丢帧时要有重发兜底。 */
    return clean ? 0 : 1;
}

void MOTOR_Init(void)
{
    IIC_init();

    /* ================== 硬件病因判别（必须在发任何 I2C 之前做） ==================
       目的：分清「上拉不够」还是「驱动板 SDA 坏了」。全程只动 GPIO，不发 I2C。 */

    /* A. 空闲电平：输入+上拉，看两根线有没有被别人拉死 */
    IIC_Line_Input(1, 1);
    IIC_Line_Input(0, 1);
    delay_us(100);
    DBG_IDLE_SCL = READ_SCL ? 1 : 0;
    DBG_IDLE_SDA = READ_SDA ? 1 : 0;

    /* B. 外部上拉是否存在：内部下拉(约40k)对抗外部上拉
          读到 1 = 外部上拉强于 40k（有上拉且线通）
          读到 0 = 外部没有上拉 → 就是"上拉不够" */
    IIC_Line_Input(0, 0);
    delay_us(200);
    DBG_SDA_PD = READ_SDA ? 1 : 0;
    IIC_Line_Input(1, 0);
    delay_us(200);
    DBG_SCL_PD = READ_SCL ? 1 : 0;

    /* C. 上升时间扫描（1/2/5/10/20/50µs） */
    IIC_Rise_Scan(0, DBG_SDA_RISE);
    IIC_Rise_Scan(1, DBG_SCL_RISE);

    /* 恢复成正常的推挽输出高，后面继续走原流程 */
    IIC_init();
    delay_us(100);
    /* ========================================================================= */

    /* 编码器极性 = 0。手册原文：「设置编码方向极性，范围 0 或 1，默认 0。
       如果发现电机转速根本不受控制，要么最快速度转动，要么停止，
       可以将此地址的值重新设置一下」——极性要按实际电机试出来。
       实测：这台车填 1 时四个电机只会正转，填 0 才正常（差速能立起来）。 */
    MotorEncoderPolarity =0 ;
    MotorType            = MOTOR_TYPE_JGB;           // 520 减速电机 → JGB37-520（类型 3）
    I2C_Write_Len(MOTOR_TYPE_ADDR, &MotorType, 4); // 写入电机类型
    I2C_Write_Len(MOTOR_ENCODER_POLARITY_ADDR, &MotorEncoderPolarity, 1); // 写入编码器极性
    delay_ms(5);

    /* 引脚自环测试：把我们自己的 SDA 脚当输出驱动高/低，再读回来。
       这一步只验证「MCU 侧引脚 + READ_SDA 读数」是否正常，跟从机无关。
       目的是把「软件读不到」和「板子不驱动」彻底分开。 */
    {
        uint8_t ok = 1;

        I2C_SDA_OUT();
        IIC_SDA = 0;
        delay_us(10);
        if (READ_SDA) ok = 0; // 驱动低却读回高
        IIC_SDA = 1;
        delay_us(10);
        if (!READ_SDA) ok = 0; // 驱动高却读回低（外部把线钳住了）
        DBG_SDA_LOOP = ok ? 0xA5 : 0x5A;
    }
}

// 总线恢复：从机（驱动板）把 SDA 拉死时，主机补 9 个 SCL 脉冲把它顶出来，再补一个 STOP。
// 现象对应「重试 3 次全 NACK、停了还在跑」：板子一直保持上一次收到的速度。
void IIC_Bus_Recover(void)
{
    uint8_t i;

    I2C_SDA_IN(); // 释放 SDA（上拉），让从机有机会放开
    IIC_SDA = 1;
    for (i = 0; i < 9; i++) {
        IIC_SCL = 0;
        delay_us(5);
        IIC_SCL = 1;
        delay_us(5);
    }
    IIC_stop();
}

uint8_t MOTOR_CONTROL_DATA_Predit(Motor_InitTypeDef *M1, Motor_InitTypeDef *M2, Motor_InitTypeDef *M3, Motor_InitTypeDef *M4)
{
    // 发送顺序固定：M1=203, M2=202, M3=204, M4=201（板子通道约定，勿改）
    // 方向极性修正：202(右前)/204(左后) 安装方向相反，速度取反（宏见 motor.h）
    // 驱动板速度寄存器协议：128=停止，>128=正转，<128=反转（Hiwonder SA8870C）
    MOTOR_Data[0] = (int8_t)(M1->PID_Output * MOTOR_DIR_203);   // 203 右后
    MOTOR_Data[1] = (int8_t)(M2->PID_Output * MOTOR_DIR_202);   // 202 右前
    MOTOR_Data[2] = (int8_t)(M3->PID_Output * MOTOR_DIR_204);   // 204 左后
    MOTOR_Data[3] = (int8_t)(M4->PID_Output * MOTOR_DIR_201);   // 201 左前

    /* 写入速度：失败（NACK）时重发，最多 3 次；仍失败则做一次总线恢复再补一帧。
       2026-10-24 左偏根因：写入顺序固定 203,202,204,201，中途 NACK 时后两个字节
       （左轮 204/201）没写进去 → 右轮有速度、左轮停 0 → 无纠偏持续左偏。
       2026-10-25：若重试全部失败，板子会保持上一次速度（表现为「命令停车但车不停」），
       必须做总线恢复后再试，并把结果返回给调用方。 */
    {
        uint8_t retry;

        /* 每次失败都补一次总线恢复（9 个 SCL 脉冲把卡住的从机顶出来）再重发，
           最多 5 次。驱动板收不到帧会一直保持上一次的速度——这就是
           「明明下发了停车、车却不停」的机制。 */
        for (retry = 0; retry < 5; retry++) {
            if (I2C_Write_Len(MOTOR_FIXED_SPEED_ADDR, MOTOR_Data, 4) == 0) {
                return 0; // 整帧写成功
            }
            DBG_MOTOR_NACK++;
            IIC_Bus_Recover();
            delay_us(100);
        }
        return 1; // 彻底失败：这一帧没进驱动板
    }
}

/**
 * @brief 读取并处理电机编码器数据，然后通过串口发送
 * @note  该函数通过I2C读取电机编码器的原始数据，将其转换为浮点数，
 *        然后通过USART1发送出去
 */
void MOTOR_ENCODER_DATA_Predit(void)//
{
    // 通过I2C从指定地址读取16字节的编码器数据到EncodeTotal数组
    I2C_Read_Len(MOTOR_ENCODER_TOTAL_ADDR,(uint8_t*)EncodeTotal,16);
    // 定义一个浮点数组，用于存储转换后的数据
    float Data[4];

    
    // 将读取的原始数据转换为16位有符号整数，再转换为浮点数
    // 这里假设每两个字节组成一个16位数据
    Data[0] = (float)((int16_t)EncodeTotal[0]);  // 转换第一个编码器数据
    Data[1] = (float)((int16_t)EncodeTotal[1]);  // 转换第二个编码器数据
    Data[2] = (float)((int16_t)EncodeTotal[2]);  // 转换第三个编码器数据
    Data[3] = (float)((int16_t)EncodeTotal[3]);  // 转换第四个编码器数据
    // 通过USART1发送转换后的浮点数据，发送4个数据点
    JustFloat_Send(Data, 4, USART1);

}
