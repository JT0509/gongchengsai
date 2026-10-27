#ifndef _led_H
#define _led_H

#include "main.h"

#define LED_RED PAout(6)
#define LED_GREEN PAout(7)

#define LED_TEAM PBout(8)
#define LED_ROUTE PEout(1)

void LED_Init(void);
void LED_EX_Init(void);

#endif
