#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "MCP251XFD.h"

#define CANSPI_HOST SPI3_HOST
#define PIN_NUM_CAN_MISO 37
#define PIN_NUM_CAN_MOSI 35
#define PIN_NUM_CAN_CLK  36
#define PIN_NUM_CAN_CS   45

#define PIN_NUM_HEARTBEAT 48

static const char *TAG = "GUB2";

static led_strip_handle_t led_strip;
static uint8_t s_led_state = 0;

static void toggleLED(void){
    #if PIN_NUM_HEARTBEAT==48
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 0, 16, 0);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
    #else

    gpio_set_level(PIN_NUM_HEARTBEAT, s_led_state);
    #endif
}

static void initLED(void){
    #if PIN_NUM_HEARTBEAT==48
    ESP_LOGD(TAG, "Heartbeat with RGB");
    led_strip_config_t strip_config = {
        .strip_gpio_num = PIN_NUM_HEARTBEAT,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);

    #else
    ESP_LOGD(TAG, "Heartbeat with GPIO");
    gpio_reset_pin(PIN_NUM_HEARTBEAT);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PIN_NUM_HEARTBEAT, GPIO_MODE_OUTPUT);
    #endif
}

void sendMessages(spi_device_handle_t spi, uint8_t data){
    ESP_LOGD("SPI3", "Preparing to send byte");
    spi_device_acquire_bus(spi, portMAX_DELAY);
    spi_transaction_t t;
    memset(&t, 0 , sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    t.user = (void*) 1;
    ESP_LOGD("SPI3", "Transmitting");
    assert(spi_device_polling_transmit(spi, &t) == ESP_OK);

    spi_device_release_bus(spi);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting GUB2");
    ESP_LOGD(TAG, "debug logging");
    initLED();

    ESP_LOGI(TAG, "Setting up SPI");

    // esp_err_t ret;
    spi_device_handle_t spi;
    
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_CAN_MISO,
        .mosi_io_num=PIN_NUM_CAN_MOSI,
        .sclk_io_num=PIN_NUM_CAN_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
    };

    spi_device_interface_config_t devcfgCAN={
        .clock_speed_hz=1000000,                //Clock out at 1 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CAN_CS,           //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=NULL,  //Specify pre-transfer callback to handle D/C line
    };

    ESP_ERROR_CHECK(spi_bus_initialize(CANSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(CANSPI_HOST, &devcfgCAN, &spi));
    
    uint8_t data = 0;
    for(;;){
        toggleLED();
        sendMessages(spi,data);
        ESP_LOGI("Counter", "%d", data);
        s_led_state = !s_led_state;
        data++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}
