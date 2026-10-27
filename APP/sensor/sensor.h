#ifndef _sensor_h
#define _sensor_h

#include "main.h"

#define FRONT_GREY_SENSOR_READ  F_GW_digital_Read()
#define BACK_GREY_SENSOR_READ   B_GW_digital_Read()

typedef struct
{
    uint8_t FRONT_SENSOR_COUNT;
    uint8_t BACK_SENSOR_COUNT;
    uint8_t LEFT_SENSOR_COUNT;
    uint8_t RIGHT_SENSOR_COUNT;

    uint8_t FRONT_SENSOR;
    uint8_t BACK_SENSOR;
    uint8_t LEFT_SENSOR;
    uint8_t RIGHT_SENSOR;

    uint8_t FRONT_SENSOR_INFORMATION;
    uint8_t BACK_SENSOR_INFORMATION;
    uint8_t LEFT_SENSOR_INFORMATION;
    uint8_t RIGHT_SENSOR_INFORMATION;
} GREY_InitTypeDef;

extern GREY_InitTypeDef GREY;   // 灰度定位结构体

void GREY_Init(void);

#endif            