#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "Lora.h"
#define BUFFER_SIZE 1024
#define UART_TIMEOUT_MS 2000

static const char *TAG = "LORA";
static char _deveui[17] = {0};
static char _appeui[17] = {0};
static char _appskey[33] = {0};
static char _devAddr[9] = {0};
static char _nwkskey[33] = {0};
static char _rxMessage[128] = {0};
static bool _otaa = false;

void lora_init(void)
{
    lora_uart_init();
    lora_join_network(OTAA);
    ESP_LOGI(TAG, "LORA INITIALIZED");
}

void lora_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = LORA_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB};

    // Configure UART0
    uart_param_config(LORA_UART_NUM, &uart_config);

    uart_set_pin(LORA_UART_NUM, LORA_UART_TX_PIN, LORA_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install UART driver
    uart_driver_install(LORA_UART_NUM, UART_BUFFER_SIZE, 0, 0, NULL, 0);

    ESP_LOGI("UART_INIT", "UART initialized for LORA");
}

bool lora_join_network(LORA_JOIN_TYPE joinType)
{
    switch (joinType)
    {
    case OTAA:
    {
        const char *appEui = "70B3D57ED00001A6";
        const char *appKey = "A23C96EE13804963F8C2BD6285448198";
        ESP_LOGI(TAG, "STARTING OTAA STUFFES");
        return initOTAA(appEui, appKey, "0");
    }
    case ABP:
    { // ABP
        const char *devAddr = "02017201";
        const char *nwkSKey = "AE17E567AECC8787F749A62F5541D522";
        const char *appSKey = "8D7FFEF938589D95AAD928C2E2E7E48F";
        return initABP(devAddr, appSKey, nwkSKey);
    }
    }
    return false;
}

bool initABP(const char *DevAddr, const char *AppSKey, const char *NwkSKey)
{
    _otaa = false;
    strcpy(_devAddr, DevAddr);
    strcpy(_appskey, AppSKey);
    strcpy(_nwkskey, NwkSKey);

    char response[BUFFER_SIZE];

    // Clear UART buffer
    while (uart_read_bytes(LORA_UART_NUM, (uint8_t *)response, sizeof(response), 100 / portTICK_PERIOD_MS) > 0)
        ;

    sendRawCommand("mac reset", response, sizeof(response));

    sendMacSet("nwkskey", _nwkskey);
    sendMacSet("appskey", _appskey);
    sendMacSet("devaddr", _devAddr);

    setAdaptiveDataRate(false);
    setAutomaticReply(false);
    setTXoutputPower(5);

    sendMacSet("dr", "5"); // 0= min, 7=max

    // Save configuration
    sendRawCommand("mac save", response, sizeof(response));

    // Attempt to join using ABP
    sendRawCommand("mac join abp", response, sizeof(response));
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second delay

    // Check if the join was successful
    if (strncmp(response, "accepted", 8) == 0)
    {
        ESP_LOGI(TAG, "LoRa ABP Join Successful");
        return true;
    }
    else
    {
        ESP_LOGW(TAG, "LoRa ABP Join Failed");
        return false;
    }
}

bool initOTAA(const char *AppEUI, const char *AppKey, const char *DevEUI)
{
    _otaa = true;
    strcpy(_nwkskey, "0");
    char response[BUFFER_SIZE];

    // Clear UART buffer
    while (uart_read_bytes(LORA_UART_NUM, (uint8_t *)response, sizeof(response), 100 / portTICK_PERIOD_MS) > 0)
        ;

    // Reset the module
    sendRawCommand("mac reset", response, sizeof(response));

    // Use given Device EUI or fetch from module
    if (strlen(DevEUI) == 16)
    {
        strcpy(_deveui, DevEUI);
    }
    else
    {
        sendRawCommand("sys get hweui", _deveui, sizeof(_deveui));
        if (strlen(_deveui) != 16)
            return false; // Invalid response
    }

    sendMacSet("deveui", _deveui);

    // Set Application EUI if provided
    if (strlen(AppEUI) == 16)
    {
        strcpy(_appeui, AppEUI);
        sendMacSet("appeui", _appeui);
    }

    // Set Application Key if provided
    if (strlen(AppKey) == 32)
    {
        strcpy(_appskey, AppKey);
        sendMacSet("appkey", _appskey);
    }

    // Configure transmit power
    setTXoutputPower(5);

    // Disable Adaptive Data Rate (ADR)
    setAdaptiveDataRate(false);

    // Disable automatic replies
    setAutomaticReply(false);

    // Save configuration
    sendRawCommand("mac save", response, sizeof(response));

    bool joined = false;

    ESP_LOGI(TAG, "MADE IT TO FOR LOOP");

    // Try joining twice
    for (int i = 0; i < 2 && !joined; i++)
    {
        ESP_LOGI(TAG, "IN FOR LOOP YAY");

        sendRawCommand("mac join otaa", response, sizeof(response));

        if (strncmp(response, "accepted", 8) == 0)
        {
            joined = true;
            vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
            ESP_LOGI(TAG, "OOTA CONNECTED :)");
        }
        else
        {
            ESP_LOGI(TAG, "OOTA FAILED ;(");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    return joined;
}

// Function to send a command over UART and read the response
bool sendRawCommand(const char *command, char *response, size_t response_size)
{
    ESP_LOGI(TAG, "SENDING COMMAND: %s", command);
    uart_write_bytes(LORA_UART_NUM, command, strlen(command));
    uart_write_bytes(LORA_UART_NUM, "\r\n", 2); // Send command termination

    int length = uart_read_bytes(LORA_UART_NUM, (uint8_t *)response, response_size - 1, pdMS_TO_TICKS(1000));
    if (length > 0)
    {
        response[length] = '\0'; // Null-terminate the response
        ESP_LOGI(TAG, "LoRa Response: %s", response);
        return true;
    }
    return false;
}

// Function to send a MAC command and get a response
bool sendMacSet(const char *key, const char *value)
{
    char command[64];
    snprintf(command, sizeof(command), "mac set %s %s", key, value);
    char response[BUFFER_SIZE];

    // Send command and read response
    uart_write_bytes(LORA_UART_NUM, command, strlen(command));
    uart_write_bytes(LORA_UART_NUM, "\r\n", 2);

    int length = uart_read_bytes(LORA_UART_NUM, (uint8_t *)response, sizeof(response) - 1, pdMS_TO_TICKS(1000));
    if (length > 0)
    {
        response[length] = '\0'; // Null-terminate response
        ESP_LOGI(TAG, "LoRa Response: %s", response);
        return (strncmp(response, LORA_OK_RESPONSE, 2) == 0); // Return true if response is "ok"
    }
    return false;
}

// Set Adaptive Data Rate (ADR)
bool setAdaptiveDataRate(bool enabled)
{
    return sendMacSet("adr", enabled ? "on" : "off");
}

// Set Automatic Reply (AR)
bool setAutomaticReply(bool enabled)
{
    return sendMacSet("ar", enabled ? "on" : "off");
}

// Set Transmit Output Power
bool setTXoutputPower(int pwridx)
{
    char pwrStr[4]; // Max value is a small integer
    snprintf(pwrStr, sizeof(pwrStr), "%d", pwridx);
    return sendMacSet("pwridx", pwrStr);
}

TX_RETURN_TYPE lora_tx(const char *data)
{
    return lora_txUncnf(data); // we are unsure which mode we're in. Better not to wait for acks.
}

TX_RETURN_TYPE lora_txCnf(const char *data)
{
    return lora_txCommand("mac tx cnf 1 ", data, true);
}

TX_RETURN_TYPE lora_txUncnf(const char *data)
{
    return lora_txCommand("mac tx uncnf 1 ", data, true);
}

LORA_RESPONSE_TYPE determineReceivedDataType(const char *receivedData)
{
    if (strlen(receivedData) != 0)
    {

        switch (receivedData[0])
        {
        case 'b':
            if (strncmp(receivedData, "busy", strlen("busy")) == 0)
            {
                return BUSY;
            }
            break;
        case 'f':
            if (strncmp(receivedData, "frame_counter_err_rejoin_needed", strlen("frame_counter_err_rejoin_needed")) == 0)
            {
                return FRAME_COUNTER_ERR_REJOIN_NEEDED;
            }
            break;
        case 'i':
            if (strncmp(receivedData, "invalid_data_len", strlen("invalid_data_len")) == 0)
            {
                return INVALID_DATA_LEN;
            }
            else if (strncmp(receivedData, "invalid_param", strlen("invalid_param")) == 0)
            {
                return INVALID_PARAM;
            }
            break;
        case 'm':
            if (strncmp(receivedData, "mac_err", strlen("mac_err")) == 0)
            {
                return MAC_ERR;
            }
            else if (strncmp(receivedData, "mac_paused", strlen("mac_paused")) == 0)
            {
                return MAC_PAUSED;
            }
            else if (strncmp(receivedData, "mac_rx", strlen("mac_rx")) == 0)
            {
                return MAC_RX;
            }
            else if (strncmp(receivedData, "mac_tx_ok", strlen("mac_tx_ok")) == 0)
            {
                return MAC_TX_OK;
            }
            break;
        case 'n':
            if (strncmp(receivedData, "no_free_ch", strlen("no_free_ch")) == 0)
            {
                return NO_FREE_CH;
            }
            else if (strncmp(receivedData, "not_joined", strlen("not_joined")) == 0)
            {
                return NOT_JOINED;
            }
            break;
        case 'o':
            if (strncmp(receivedData, "ok", strlen("ok")) == 0)
            {
                return OK;
            }
            break;
        case 'r':
            if (strncmp(receivedData, "radio_err", strlen("radio_err")) == 0)
            {
                return RADIO_ERR;
            }
            else if (strncmp(receivedData, "radio_tx_ok", strlen("radio_tx_ok")) == 0)
            {
                return RADIO_TX_OK;
            }
            break;
        case 's':
            if (strncmp(receivedData, "silent", strlen("silent")) == 0)
            {
                return SILENT;
            }
            break;
        default:
            return UNKNOWN;
        }
    }
    return UNKNOWN;
}

void readUntilNewline(char *receivedData, int max_len)
{
    uint8_t byte;
    int index = 0;

    while (index < max_len - 1) // Leave space for null terminator
    {
        int len = uart_read_bytes(LORA_UART_NUM, &byte, 1, pdMS_TO_TICKS(2000));
        if (len > 0)
        {
            if (byte == '\n')
            {
                break;
            }
            receivedData[index++] = byte;
        }
        else
        {
            // Timeout or error
            break;
        }
    }

    receivedData[index] = '\0'; // Null-terminate
}

void extract_rx_message(const char *receivedData, char *rxMessage, size_t rxMessageSize)
{
    // Start searching from index 7
    const char *start = receivedData + 7;

    // Find the first space after index 7
    const char *space = strchr(start, ' ');

    if (space != NULL && (space + 1 - receivedData) < strlen(receivedData))
    {
        const char *messageStart = space + 1;

        // Copy the message into rxMessage, safely
        strncpy(rxMessage, messageStart, rxMessageSize - 1);
        rxMessage[rxMessageSize - 1] = '\0'; // Null-terminate
    }
    else
    {
        // If no space found, set empty string
        rxMessage[0] = '\0';
    }
}

TX_RETURN_TYPE lora_txCommand(const char *command, const char *data, bool shouldEncode)
{
    bool send_success = false;
    uint8_t busy_count = 0;
    uint8_t retry_count = 0;
    char response[128];

    // clear serial buffer
    while (uart_read_bytes(LORA_UART_NUM, (uint8_t *)response, sizeof(response), 100 / portTICK_PERIOD_MS) > 0)
        ;

    while (!send_success)
    {
        // retransmit a maximum of 10 times
        retry_count++;
        if (retry_count > 10)
        {
            return TX_FAIL;
        }

        // Send command
        uart_write_bytes(LORA_UART_NUM, command, strlen(command));

        uart_write_bytes(LORA_UART_NUM, data, strlen(data));

        uart_write_bytes(LORA_UART_NUM, "\r\n", 2);

        // int len = uart_read_bytes(LORA_UART_NUM, receivedData, sizeof(receivedData) - 1, pdMS_TO_TICKS(UART_TIMEOUT_MS));

        char *receivedData = malloc(sizeof(char) * 128);
        readUntilNewline(receivedData, 128);

        switch (determineReceivedDataType(receivedData))
        {
        case OK:
        {
            uart_read_bytes(LORA_UART_NUM, receivedData, sizeof(receivedData) - 1, pdMS_TO_TICKS(30000));

            switch (determineReceivedDataType(receivedData))
            {
            case MAC_TX_OK:
            {
                // SUCCESS!!
                send_success = true;
                return TX_SUCCESS;
            }

            case MAC_RX:
            {
                // example: mac_rx 1 54657374696E6720313233
                extract_rx_message(receivedData, _rxMessage, sizeof(_rxMessage));
                send_success = true;
                return TX_WITH_RX;
            }

            case MAC_ERR:
            {
                lora_init();
                break;
            }

            case INVALID_DATA_LEN:
            {
                // this should never happen if the prototype worked
                send_success = true;
                return TX_FAIL;
            }

            case RADIO_TX_OK:
            {
                // SUCCESS!!
                send_success = true;
                return TX_SUCCESS;
            }

            case RADIO_ERR:
            {
                // This should never happen. If it does, something major is wrong.
                lora_init();
                break;
            }

            default:
            {
                // unknown response
                // init();
            }
            } // End while after "ok"
            break;
        }

        case INVALID_PARAM:
        {
            // should not happen if we typed the commands correctly
            send_success = true;
            return TX_FAIL;
        }

        case NOT_JOINED:
        {
            lora_init();
            break;
        }

        case NO_FREE_CH:
        {
            // retry
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        }

        case SILENT:
        {
            lora_init();
            break;
        }

        case FRAME_COUNTER_ERR_REJOIN_NEEDED:
        {
            lora_init();
            break;
        }

        case BUSY:
        {
            busy_count++;

            // Not sure if this is wise. At low data rates with large packets
            // this can perhaps cause transmissions at more than 1% duty cycle.
            // Need to calculate the correct constant value.
            // But it is wise to have this check and re-init in case the
            // lorawan stack in the RN2xx3 hangs.
            if (busy_count >= 10)
            {
                lora_init();
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            break;
        }

        case MAC_PAUSED:
        {
            lora_init();
            break;
        }

        case INVALID_DATA_LEN:
        {
            // should not happen if the prototype worked
            send_success = true;
            return TX_FAIL;
        }

        default:
        {
            // unknown response after mac tx command
            lora_init();
            break;
        }
        }
    }

    return TX_FAIL; // should never reach this
}