#ifndef GUB2_H
#define GUB2_H

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include "led_strip.h"
#include "GUB_Conf.h"

#include "sdmmc_cmd.h"

#define GUB_STACK_SIZE 4096

/**
 * Enum to represent the current SD card state
*/
enum SDCardState {
    SD_NOT_CONNECTED = 0,
    SD_READY,
    SD_NOT_MOUNTED,
};

enum GUBSystemEvents{
    SD_CONNECT_EVENT    = 0x01,
    SD_DISCONNECT_EVENT = 0x02,
    CAN_EVENT           = 0x04,
};

/**
 * GUB State Struct to hold the global states of the board
*/
typedef struct
{   
#ifdef LED_IS_NEOPIXEL
    led_strip_handle_t ledStrip; 
#endif
    uint8_t ledState;
    uint8_t sdCardState;
    uint64_t lastHeartbeat;
    uint64_t lastSDCheck;
    bool SDconnected;
    EventGroupHandle_t gubEvents;
} GUBState_t;

static const char* SDcardBasePath = "/data";
static const char* canLogPath = "/data/CANLogs";

// GUB Setup 
void GUBInit();
void GUBStart();
void GUBloop(void *pvParam);

void GUBInitLED();
void GUBToggleLED();
void GUBHeartbeatUpdate();

// Key Methods
int GUBMountSDCard(const char* basePath, sdmmc_card_t *card);

// Debug
void printGUBStatus();
int listDir(const char *base_path);

#endif //GUB2H