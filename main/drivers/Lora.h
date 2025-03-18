/******************************************************************************
 * Lora.h
 * This is using the RN2903 chip.
 *****************************************************************************/
#ifndef LORA_H
#define LORA_H

#include "driver/uart.h"
#include "esp_err.h"
#include <string.h>

#define LORA_UART_NUM UART_NUM_1
#define LORA_UART_TX_PIN 18
#define LORA_UART_RX_PIN 17
#define LORA_RESET_PIN 1
#define LORA_INT1_PIN 8
#define LORA_UART_BAUDRATE 57600
#define LORA_UART_BUF_SIZE 256

#define LORA_OK_RESPONSE "ok"
#define LORA_ERROR_RESPONSE "invalid_param"

enum LORA_JOIN_TYPE
{
    OTAA = 0,
    ABP = 1
};

static char _deveui[17] = {0};
static char _appeui[17] = {0};
static char _appskey[33] = {0};
static char _devAddr[9] = {0};
static char _nwkskey[33] = {0};
static bool _otaa = false;

esp_err_t lora_init(void);
esp_err_t lora_join_network(LORA_JOIN_TYPE joinType);
bool sendRawCommand(const char *command, char *response, size_t response_size);
void sendMacSet(const char *key, const char *value);
bool setAdaptiveDataRate(bool enabled);
bool setAutomaticReply(bool enabled);
bool setTXoutputPower(int pwridx);
void initOTAA(const char *appEui, const char *appKey);
void initABP(const char *devAddr, const char *appSKey, const char *nwkSKey);

#endif // LORA_H