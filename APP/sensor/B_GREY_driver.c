#include "GREY_driver.h"

// 后灰度（B）：硬件 I2C2（PB10/PB11），设备地址 0x12，读寄存器 0x30

void B_GW_digital_Init(void)
{
    Grey_I2C2_Init();
}

// 读后灰度数字量：从寄存器 0x30 读 1 字节，bit7~bit0 = 探头1~8
// 极性由 GW_BLACK_IS_ONE 决定（见 GREY_driver.h），黑线对应探头位应为 1
uint8_t B_GW_digital_Read(void)
{
    uint8_t d = Grey_I2C_ReadReg(I2C2, GW_GRAY_DATA);
#if GW_BLACK_IS_ONE
    return d ^ 0xFF; // 黑=1、白=0
#else
    return d; // 传感器直出（黑=0、白=1）
#endif
}

// 后灰度加权线位置（同前灰度）
int16_t B_GW_digital_eight(void)
{
    uint8_t data = B_GW_digital_Read();
    int16_t s1   = (data >> 7) & 0x01;
    int16_t s2   = (data >> 6) & 0x01;
    int16_t s3   = (data >> 5) & 0x01;
    int16_t s4   = (data >> 4) & 0x01;
    int16_t s5   = (data >> 3) & 0x01;
    int16_t s6   = (data >> 2) & 0x01;
    int16_t s7   = (data >> 1) & 0x01;
    int16_t s8   = (data >> 0) & 0x01;

    return s1 * 7 + s2 * 5 + s3 * 3 + s4 * 1 + s5 * (-1) + s6 * (-3) + s7 * (-5) + s8 * (-7);
}

// 后灰度线质心位置（×10）：0~70，中心 35；-1=全白丢线，-2=全黑（路口/宽线）
// 探头编号 0=探头1(bit7 最左) ~ 7=探头8(bit0 最右)，黑=1 白=0
int16_t B_GW_position(void)
{
    uint8_t data  = B_GW_digital_Read();
    int16_t count = 0, sum = 0;
    int16_t i;

    for (i = 0; i < 8; i++) {
        if ((data >> (7 - i)) & 0x01) { // 探头 i 黑
            count++;
            sum += i;
        }
    }

    if (count == 0) return -1; // 全白：丢线
    if (count == 8) return -2; // 全黑：路口/宽线
    return sum * 10 / count;   // 质心 ×10（0~70）
}
