/*
can be done with gpgga (everything but speed) and gpvtg (speed)

Date & Time is on GPRMC. Find out how to get the nmea message, but only a single time
*/

#include <string.h>
#include <math.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

#include "TESEO-LIV3F.h"

// UART configuration

#define TESEO_UART_PORT UART_NUM_2 // Using UART2
#define TESEO_UART_RX_PIN 5        // The UART RX pin being used
#define TESEO_UART_TX_PIN 4        // The UART TX pin being used
#define TESEO_UART_BAUD_RATE 9600  // The UART baud rate
#define UART_BUFFER_SIZE 1024      // Buffer size for incoming data
#define GPIO_NUM_1 1
#define GPIO_NUM_3 3
#define PIN_NUM_GPS_RESET 9

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
    uart_driver_install(TESEO_UART_PORT, UART_BUFFER_SIZE, 0, 0, NULL, 0);

    reset_teseo();

    ESP_LOGI(TAG, "TESEO GPS initialized over UART");
    return ESP_OK;
}

char get_checksum(const char *nmea_command)
{
    char checksum = 0;
    for (int i = 0; nmea_command[i] != '\0'; i++)
    {
        checksum ^= nmea_command[i]; // XOR each character
    }
    return checksum;
}

void create_nmea(const char *nmea_command, char **formatted_command)
{
    char checksum = get_checksum(nmea_command);

    // Allocate memory for the full formatted NMEA string
    *formatted_command = (char *)malloc(strlen(nmea_command) + 10); // Extra space for $, *, checksum, \r\n, and null terminator
    if (*formatted_command == NULL)
    {
        return; // Memory allocation failed
    }

    // Format the final NMEA message
    sprintf(*formatted_command, "$%s*%02X\r\n", nmea_command, checksum);
}

void reset_teseo()
{
    gpio_reset_pin(PIN_NUM_GPS_RESET);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PIN_NUM_GPS_RESET, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_NUM_GPS_RESET, 0);

    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(PIN_NUM_GPS_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
}

esp_err_t teseo_uart_send(const char *nmea_com)
{
    char *nmea_command = NULL;
    create_nmea(nmea_com, &nmea_command);

    int length = strlen(nmea_command);
    uart_write_bytes(TESEO_UART_PORT, nmea_command, length);
    ESP_LOGI(TAG, "Sent: '%s'", nmea_command);
    char response[1024];
    uart_read_bytes(TESEO_UART_PORT, response, sizeof(response), pdMS_TO_TICKS(1000));

    free(nmea_command);
    return ESP_OK;
}

// Read data from TESEO-LIV3F
esp_err_t teseo_uart_read(void)
{
    char data[UART_BUFFER_SIZE];
    int length = 0;

    // read data from UART
    length = uart_read_bytes(TESEO_UART_PORT, data, sizeof(data), pdMS_TO_TICKS(1000));
    if (length > 0)
    {
        data[length] = '\0'; // Null-terminate the string
        // ESP_LOGI(TAG, "Received: %s", data);

        // Parse Response
        parse_nmea(data);
    }
    return ESP_OK;
}

void request_gps_data()
{
    teseo_uart_send("PSTMNMEAREQUEST,0x12,0x2000");
}

void set_teseo_build()
{
    teseo_uart_send("PSTMSTANDBYENABLE,0");
    teseo_uart_send("PSTMINITGPS,435.047,N,7740.545,W,0600,04,03,2025,01,36,02");
    teseo_uart_send("PSTMSETPAR,1201,0x12"); // disable all lower 32 messages except GGA and VTG
    // teseo_uart_send("$PSTMGETPAR,1201*21\r\n");
    teseo_uart_send("PSTMSAVEPAR"); // Save it to non volatile memory
    teseo_uart_send("PSTMSRR");     // reset it to activate the changes
}

void parse_nmea(char *nmea)
{
    ESP_LOGI(TAG, "\n\n\nParsing NMEA: %s\n\n\n", nmea);
    char *temp = strtok(nmea, "\n");
    while (temp != NULL)
    {
        // if (strncmp(temp, "$GPGGA", 6) == 0)
        // {
        //     char *token = strtok(temp, ",");
        //     int field = 0;
        //     float latitude = 0.0, longitude = 0.0, time = 0.0;
        //     while (token != NULL)
        //     {
        //         field++;
        //         if (field == 2)
        //         { // Time
        //             time = atof(token);
        //         }
        //         else if (field == 3)
        //         { // Latitude
        //             latitude = atof(token);
        //         }
        //         else if (field == 5)
        //         { // Longitude
        //             longitude = atof(token);
        //         }
        //         token = strtok(NULL, ",");
        //     }

        //     ESP_LOGI(TAG, "Parsed GPGGA: Time = %f, Latitude = %f, Longitude = %f\n\n\n\n", time, convert_degrees(latitude), convert_degrees(longitude));
        // } else if (strncmp(temp, "$GPVTG", 6) == 0) {
        //     char *token = strtok(temp, ",");
        //     int field = 0;
        //     float true_track = 0.0, speed_kmh = 0.0;
        //     while (token != NULL)
        //     {
        //         field++;
        //         if (field == 2)
        //         { // Time
        //             time = atof(token);
        //         }
        //         else if (field == 3)
        //         { // Latitude
        //             latitude = atof(token);
        //         }
        //         else if (field == 8)
        //         { // Speed in km/h
        //            speed_kmh = atof(token);
        //         }
        //         token = strtok(NULL, ",");
        //     }

        // ESP_LOGI(TAG, "Parsed GPVTG: True Track = %.2f°, Speed = %.2f km/h\n\n\n\n", true_track, speed_kmh);
        // }
        temp = strtok(NULL, "\n");
    }
}

float convert_degrees(float nmea_coord)
{
    int degrees = (int)(nmea_coord / 100); // Extract degrees
    float minutes = fmod(nmea_coord, 100); // Extract minutes
    return degrees + (minutes / 60.0);     // Convert to decimal degrees
}
