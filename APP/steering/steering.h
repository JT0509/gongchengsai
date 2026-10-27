#ifndef _steering_H
#define _steering_H

#include "main.h"

void Steering_Init(void);
void Steering_RESET(void);
void Steering_SET(void);
void Engine_Working(void);

extern u8 STEERING_FLAG;

#endif