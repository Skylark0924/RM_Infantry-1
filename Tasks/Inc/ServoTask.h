#ifndef __SERVOTASK_H
#define __SERVOTASK_H

//#include "utilities_debug.h"
#include "usart.h"
#include "stm32f4xx_hal_uart.h"
//#include "peripheral_define.h"
#include <stdio.h>

void servoUartRxCpltCallback(void);

void InitServoUart(void);

#endif
