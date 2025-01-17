#include "TESEO-LIV3f.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

// UART configuration

#define TESEO_UART_PORT UART_NUM_2 // Using UART2
#define TESEO_UART_RX_PIN 4        // The UART RX pin being used
#define TESEO_UART_TX_PIN 5        // The UART TX pin being used
#define TESEO_UART_BAUD_RATE 9600  // The UART baud rate
#define UART_BUFFER_SIZE 1024      // Buffer size for incoming data

static const char *TAG = "TESEO_GPS";

/* Setup UART */
esp_err_t teseo_init(void)
{
    // UART configuration
    const uart_config_t uart_config = {
        .baud_rate = TESEO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    // Configure UART parameters
    uart_param_config(TESEO_UART_PORT, &uart_config);

    // Set UART pins
    uart_set_pin(TESEO_UART_PORT, TESEO_UART_TX_PIN, TESEO_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver
    uart_driver_install(TESEO_UART_PORT, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "TESEO GPS initialized over UART");
    return ESP_OK;
}

esp_err_t teseo_uart_send(const char *nmea_command)
{
    int len = strlen(nmea_command);
    uart_write_bytes(TESEO_UART_PORT, nmea_command, len);
    ESP_LOGI(TAG, "Sent: '%s'", nmea_command);
    return ESP_OK;
}

// Read data from TESEO-LIV3F
esp_err_t teseo_uart_read(void)
{
    uint8_t data[UART_BUFFER_SIZE];
    int length = 0;

    // Continuously read data from UART
    while (1)
    {
        length = uart_read_bytes(TESEO_UART_PORT, data, sizeof(data), pdMS_TO_TICKS(1000));
        if (length > 0)
        {
            data[length] = '\0'; // Null-terminate the string
            ESP_LOGI(TAG, "Received: %s", data);

            // Parse Response
            parse_nmea((char *)data);
        }
    }
    return ESP_OK;
}

void parse_nmea(const char *nmea)
{
    if (strncmp(nmea, "$GPGGA", 6) == 0)
    {
        char *token = strtok((char *)nmea, ",");
        int field = 0;
        float latitude = 0.0, longitude = 0.0, time = 0.0;

        while (token != NULL)
        {
            field++;
            if (field == 2)
            { // Time
                time = atof(token);
            }
            else if (field == 3)
            { // Latitude
                latitude = atof(token);
            }
            else if (field == 5)
            { // Longitude
                longitude = atof(token);
            }
            token = strtok(NULL, ",");
        }

        ESP_LOGI(TAG, "Parsed GPGGA: Time: =%f, Latitude = %f, Longitude = %f", time, latitude, longitude);
    }
}
