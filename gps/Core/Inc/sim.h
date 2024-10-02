#ifndef _SIM_H_
#define _SIM_H_

#include "stm32f4xx_hal.h"


void simInit();
void httpConfig();
void updateGPS(char *data);

#endif
