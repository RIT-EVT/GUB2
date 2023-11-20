#ifndef GUB2_H
#define GUB2_H

#include <stdint.h>
#include "led_strip.h"
#include "GUB_Conf.h"
#include "sdmmc_cmd.h"

static led_strip_handle_t led_strip;
static uint8_t s_led_state = 0;

void toggleLED();
void initLED();

int mountSDCard(const char* basePath, sdmmc_card_t *card);

#endif //GUB2H