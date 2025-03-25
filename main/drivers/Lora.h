/******************************************************************************
 * Lora.h
 * This is using the RN2903 chip.
 *****************************************************************************/
#ifndef LORA_H
#define LORA_H

#include "esp_err.h"
#include <string.h>

#define LORA_UART_NUM UART_NUM_1
#define LORA_UART_TX_PIN 18
#define LORA_UART_RX_PIN 17
#define LORA_RESET_PIN 1
#define LORA_INT0_PIN 15
#define LORA_INT1_PIN 8
#define LORA_UART_BAUDRATE 57600
#define UART_BUFFER_SIZE 1024

#define LORA_OK_RESPONSE "ok"
#define LORA_ERROR_RESPONSE "invalid_param"

typedef enum
{
    OTAA = 0,
    ABP = 1
} LORA_JOIN_TYPE;

typedef enum
{
    TX_FAIL = 0, // The transmission failed.
                 // If you sent a confirmed message and it is not acked,
                 // this will be the returned value.

    TX_SUCCESS = 1, // The transmission was successful.
                    // Also the case when a confirmed message was acked.

    TX_WITH_RX = 2 // A downlink message was received after the transmission.
                   // This also implies that a confirmed message is acked.
} TX_RETURN_TYPE;

typedef enum
{
    BUSY,
    FRAME_COUNTER_ERR_REJOIN_NEEDED,
    INVALID_DATA_LEN,
    INVALID_PARAM,
    MAC_ERR,
    MAC_PAUSED,
    MAC_RX,
    MAC_TX_OK,
    NO_FREE_CH,
    NOT_JOINED,
    OK,
    RADIO_ERR,
    RADIO_TX_OK,
    SILENT,
    UNKNOWN
} LORA_RESPONSE_TYPE;

void lora_init(void);
void lora_uart_init(void);
bool lora_join_network(LORA_JOIN_TYPE joinType);
bool sendRawCommand(const char *command, char *response, size_t response_size);
bool sendMacSet(const char *key, const char *value);
bool setAdaptiveDataRate(bool enabled);
bool setAutomaticReply(bool enabled);
bool setTXoutputPower(int pwridx);
bool initOTAA(const char *appEui, const char *appKey, const char *DevEUI);
bool initABP(const char *devAddr, const char *appSKey, const char *nwkSKey);

TX_RETURN_TYPE lora_tx(const char *data);

TX_RETURN_TYPE lora_txBytes(const char *data, uint8_t size);

TX_RETURN_TYPE lora_txUncnf(const char *data);

TX_RETURN_TYPE lora_txCnf(const char *data);

TX_RETURN_TYPE lora_txCommand(const char *command, const char *data, bool shouldEncode);

#endif // LORA_H