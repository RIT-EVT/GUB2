#include <stdint.h>

#include "GUB2.h"
#include "led_strip.h"
#include "esp_log.h"

void toggleLED(){
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
    
    s_led_state = !s_led_state;
}

void initLED(){
    #if PIN_NUM_HEARTBEAT==48
    ESP_LOGI("GUB_INIT", "Heartbeat with RGB");
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
    ESP_LOGI("GUB_INIT", "Heartbeat with GPIO");
    gpio_reset_pin(PIN_NUM_HEARTBEAT);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PIN_NUM_HEARTBEAT, GPIO_MODE_OUTPUT);
    #endif
}