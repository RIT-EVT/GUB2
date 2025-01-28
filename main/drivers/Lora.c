// #include "Lora.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_BUFFER_SIZE 1024

void uart0_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB};

    // Configure UART0
    uart_param_config(UART_NUM_0, &uart_config);

    // Use default pins for UART0: GPIO1 (TX), GPIO3 (RX)
    uart_set_pin(UART_NUM_0, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver
    uart_driver_install(UART_NUM_0, UART_BUFFER_SIZE, 0, 0, NULL, 0);

    ESP_LOGI("UART_INIT", "UART0 initialized on GPIO1 (TX) and GPIO3 (RX)");
}

void app_main(void)
{
    // Initialize UART0
    uart0_init();

    // Send a test message
    const char *message = "BY GOLLY GEE WIZZ IT WORKS!!\n";
    uart_write_bytes(UART_NUM_0, message, strlen(message));

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
        uart_write_bytes(UART_NUM_0, "Still working...\n", 17);
    }
}