#include "gps.h"

void gpsInit() {

}

void getGPS(float *lat, float *lon, uint8_t *datetime) {

}

void getGPSData(char *gps_data) {
	char *data = "12.12441,124.12454,50,1727845307";
	strcpy(gps_data, data);
}
