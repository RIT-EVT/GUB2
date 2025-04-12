/******************************************************************************
 * TESEO-LIV3F.h
 *****************************************************************************/
#ifndef TESEO_H
#define TESEO_H

#include "esp_err.h"
#include "GUB_Conf.h"

#define TESEO_UART_PORT UART_NUM_2 // Using UART2
#define TESEO_UART_BAUD_RATE 9600  // The UART baud rate
#define UART_BUFFER_SIZE 1024      // Buffer size for incoming data
#define GPIO_NUM_1 1
#define GPIO_NUM_3 3

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