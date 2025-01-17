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
void parse_nmea(const char *nmea);

#endif // TESEO_H