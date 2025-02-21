/******************************************************************************
 * TESEO-LIV3F.h
 *****************************************************************************/
#ifndef TESEO_H
#define TESEO_H

#include "esp_err.h"

// Functions
esp_err_t teseo_init(void);
esp_err_t teseo_uart_send(const char *nmea_command);
esp_err_t teseo_uart_read(void);
void parse_nmea(char *nmea);
void reset_teseo();
char create_checksum(const char *nmea_command);
void create_nmea(const char *nmea_command, char **formatted_command);
void set_teseo_build();
float convert_degrees(float nmea_coord);

#endif // TESEO_H