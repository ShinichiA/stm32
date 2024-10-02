#ifndef _GPS_H_
#define _GPS_H_

#include "stm32f4xx_hal.h"
#include <string.h>

void gpsInit();
void getGPS(float *lat, float *lon, uint8_t *datetime);
void getGPSData(char *gps_data);

#endif
