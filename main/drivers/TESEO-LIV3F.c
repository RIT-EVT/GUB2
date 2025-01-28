#include <string.h>
#include <math.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

#include "TESEO-LIV3f.h"

// UART configuration

#define TESEO_UART_PORT UART_NUM_2 // Using UART2
#define TESEO_UART_RX_PIN 4        // The UART RX pin being used
#define TESEO_UART_TX_PIN 5        // The UART TX pin being used
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

    gpio_reset_pin(PIN_NUM_GPS_RESET);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PIN_NUM_GPS_RESET, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_NUM_GPS_RESET, 0);

    vTaskDelay(pdMS_TO_TICKS(500));

    gpio_set_level(PIN_NUM_GPS_RESET, 1);

    ESP_LOGI(TAG, "TESEO GPS initialized over UART");
    // teseo_uart_read();
    // esp_rom_delay_us(1000000);                        // Wait 1 second
    // teseo_uart_send("$PSTMSETNMEARATE,GGA,1*29\r\n"); // Set GPGGA NMEA rate to 4 seconds
    // esp_rom_delay_us(1000000);                        // Wait 1 second
    // teseo_uart_send("$PSTMSETNMEARATE,GSA,0*25\r\n"); // Disable GPGSA NMEA
    // teseo_uart_send("$PSTMSETNMEARATE,VTG,0*2E\r\n"); // Disable GPVTG NMEA
    // teseo_uart_send("$PSTMGETSWVER,2*17\r\n");
    // teseo_uart_send("$PSTMSETNMEARATE,GSV,0*26\r\n"); // Disable GPGSV NMEA
    // teseo_uart_send("$PSTMSETNMEARATE,GSA,0*25\r\n"); // Disable GPGSA NMEA
    // teseo_uart_send("$PSTMSAVEPAR*3D\r\n"); // Save parameters
    // esp_rom_delay_us(1000000);              // Wait 1 second
    // teseo_uart_send("$PSTMRESTART*4A\r\n"); // Restart TESEO to apply changes
    // esp_rom_delay_us(1000000);              // Wait 1 second
    ESP_LOGI(TAG, "TESEO GPS configured");
    return ESP_OK;

    //     // FOR TESTING PURPOSES TO PRINT TO TERMINAL
    // uart_config_t uart_config = {
    //     .baud_rate = 9600,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .source_clk = UART_SCLK_APB};

    // // Configure UART parameters
    // uart_param_config(UART_NUM_1, &uart_config);

    // // Set UART pins
    // uart_set_pin(UART_NUM_1, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // // Install UART driver
    // uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0);
}

esp_err_t teseo_uart_send(const char *nmea_command)
{
    int length = strlen(nmea_command);
    uart_write_bytes(TESEO_UART_PORT, nmea_command, length);
    ESP_LOGI(TAG, "Sent: '%s'", nmea_command);
    char response[1024];
    int len = uart_read_bytes(TESEO_UART_PORT, response, sizeof(response), pdMS_TO_TICKS(1000));
    if (len > 0)
    {
        response[len] = '\0';
        ESP_LOGI(TAG, "Response: %s", response);
        if (strstr(response, "$PSTMSUCCESS"))
        {
            ESP_LOGI(TAG, "Command successful");
        }
        else
        {
            ESP_LOGW(TAG, "Command failed");
        }
    }
    else
    {
        ESP_LOGW(TAG, "No response from TESEO");
    }
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

void parse_nmea(char *nmea)
{
    ESP_LOGI(TAG, "\n\n\nParsing NMEA: %s\n\n\n", nmea);
    char *temp = strtok(nmea, "\n");
    while (temp != NULL)
    {
        if (strncmp(temp, "$GPGGA", 6) == 0)
        {
            char *token = strtok(temp, ",");
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

            ESP_LOGI(TAG, "Parsed GPGGA: Time = %f, Latitude = %f, Longitude = %f\n\n\n\n", time, convert_degrees(latitude), convert_degrees(longitude));
        }
        temp = strtok(NULL, "\n");
    }
}

float convert_degrees(float nmea_coord)
{
    int degrees = (int)(nmea_coord / 100); // Extract degrees
    float minutes = fmod(nmea_coord, 100); // Extract minutes
    return degrees + (minutes / 60.0);     // Convert to decimal degrees
}
