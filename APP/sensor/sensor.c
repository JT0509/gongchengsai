#include "sensor.h"

GREY_InitTypeDef GREY;

void GREY_Init(void)
{
    F_GW_digital_Init();
    B_GW_digital_Init();
    S_GW_Init();
}