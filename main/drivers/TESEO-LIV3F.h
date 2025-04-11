/******************************************************************************
 * TESEO-LIV3F.h
 *****************************************************************************/
#ifndef TESEO_H
#define TESEO_H

#include "esp_err.h"

// Functions
esp_err_t teseoInit(void);
esp_err_t teseoUartSend(const char *nmea_command);
esp_err_t teseoUartRead(void);
void parseNMEA(char *nmea);
void resetTeseo();
char createChecksum(const char *nmea_command);
void createNMEA(const char *nmea_command, char **formatted_command);
void requestGpsData();
void setTeseoBuild();
float convertDegrees(float nmea_coord);

#endif // TESEO_H