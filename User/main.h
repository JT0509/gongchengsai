#ifndef _main_H
#define _main_H

/* 相对路径引用：EIDE 回退成旧工程模型（incList 丢了 APP/key、APP/nav、
 * APP/steering）时，本文件仍然能找到全部头文件。 */
#include "../Public/system.h"
#include "../Public/SysTick.h"
#include "../APP/led/led.h"
#include "../APP/usart/usart.h"
#include "../APP/motor/motor.h"
#include "../APP/sensor/GREY_driver.h"
#include "../APP/trace/trace.h"
#include "../APP/VOFA/VOFA.h"
#include <stdio.h>
#include "../APP/sensor/sensor.h"
#include "../APP/key/key.h"
#include "../APP/color/color.h"
#include "../APP/color/tcs3200.h"
#include "../APP/steering/steering.h"
#include "../APP/sensor/S_GREY_driver.h"
#include "../APP/nav/nav.h"

#define GREY_SENSOR_LIMITED 3

#endif