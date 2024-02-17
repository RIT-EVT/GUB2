#ifndef GUB2_H
#define GUB2_H

#include <stdint.h>
#include "led_strip.h"
#include "GUB_Conf.h"
#include "GUB_conf.h"
#include "freertos/task.h"
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
} GUBState_t;

// esp_event_loop_handle_t GUBEventLoop;

static const char* SDcardBasePath = "/data";
static const char* canLogPath = "/data/CANLogs";

void GUBInit();
void GUBStart();
void GUBloop(void *pvParam);

void GUBInitLED();
void GUBToggleLED();
void GUBHeartbeatHandler();

int GUBMountSDCard(const char* basePath, sdmmc_card_t *card);
int listDir(const char *base_path);

#endif //GUB2H