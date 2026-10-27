#ifndef _VOFA_H
#define _VOFA_H

#include "main.h"

typedef union
{
    float fdata;
    unsigned long ldata;
}FLoatLongType;

void JustFloat_Send(float * fdata,uint16_t fdata_num,USART_TypeDef *Usart_choose);

#endif