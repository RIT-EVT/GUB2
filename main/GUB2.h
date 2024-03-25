#ifndef GUB2_H
#define GUB2_H

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <sdmmc_cmd.h>

#include "GUB_Conf.h"

#if LED_IS_NEOPIXEL == 1
#include "led_strip.h"
#endif

#define GUB_STACK_SIZE 4096

/**
 * Enum to represent the current SD card state
 */
enum SDCardState {
    SD_NOT_CONNECTED = 0,
    SD_NOT_MOUNTED,
    SD_READY,
};

enum GUBSystemEvents {
    SD_CONNECT_EVENT = 0x01,
    SD_DISCONNECT_EVENT = 0x02,
    CAN_EVENT = 0x04,
};

/**
 * GUB State Struct to hold the global states of the board
 */
typedef struct {
    EventGroupHandle_t gubEvents;
    TaskHandle_t mainTaskHandler;
    sdmmc_card_t *SDcard;
#ifdef LED_IS_NEOPIXEL
    led_strip_handle_t ledStrip;
#endif
    // Heartbeat
    uint8_t ledState;
    uint64_t lastHeartbeat;

    //SD card
    uint64_t lastSDCheck;
    uint64_t lastMountAttempt;
    uint8_t sdCardState;

} GUBState_t;

static const char *SD_CARD_BASE_PATH = SD_CARD_MOUNT_PATH;

// GUB Setup
void GUBInit();
void GUBloop(void *pvParam);

// LED methods
void GUBInitLED();
void GUBToggleLED();
void GUBHeartbeatUpdate();

// Key Methods
int GUBMountSDCard(const char *basePath);
int checkSDCardStatus();

// Debug
void printGUBStatus();
int listDir(const char *base_path);

#endif // GUB2H