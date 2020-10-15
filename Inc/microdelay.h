#ifndef _MICRODELAY_H
#define _MICRODELAY_H

#include "stm32f3xx_hal.h"

uint8_t init_DWT(void);
void usDelay(__IO uint32_t micros);

#endif
