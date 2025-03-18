// #include "Lora.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_BUFFER_SIZE 1024
static const char *TAG = "LORA";

esp_err_t lora_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = LORA_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB};

    // Install UART driver
    uart_driver_install(LORA_UART_NUM, UART_BUFFER_SIZE, 0, 0, NULL, 0);

    // Configure UART0
    uart_param_config(LORA_UART_NUM, &uart_config);

    // Use default pins for UART0: GPIO1 (TX), GPIO3 (RX)
    uart_set_pin(LORA_UART_NUM, GPIO_NUM_1, GPIO_NUM_3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI("UART_INIT", "UART0 initialized for LORA");
}

esp_err_t lora_join_network(LORA_JOIN_TYPE joinType)
{
    switch (joinType):
    case OTAA
    {
        const char *appEui = "70B3D57ED00001A6";
        const char *appKey = "A23C96EE13804963F8C2BD6285448198";
        initOTAA(appEui, appKey);
        break;
    }
    case ABP
    { // ABP
        const char *devAddr = "02017201";
        const char *nwkSKey = "AE17E567AECC8787F749A62F5541D522";
        const char *appSKey = "8D7FFEF938589D95AAD928C2E2E7E48F";
        initABP(devAddr, appSKey, nwkSKey);
        break;
    }
}

void initABP(const char *devAddr, const char *appSKey, const char *nwkSKey)
{
    _otaa = false;
    strcpy(_devAddr, devAddr);
    strcpy(_appskey, AppSKey);
    strcpy(_nwkskey, NwkSKey);
    
    char response[BUFFER_SIZE];

    // Clear UART buffer
    while (uart_read_bytes(LORA_UART_NUM, (uint8_t *)response, sizeof(response), 100 / portTICK_PERIOD_MS) > 0);

    sendRawCommand("mac reset", response, sizeof(response));

    sendMacSet("nwkskey", _nwkskey);
    sendMacSet("appskey", _appskey);
    sendMacSet("devaddr", _devAddr);

    setAdaptiveDataRate(false);
    setAutomaticReply(false);
    setTXoutputPower(5);

    sendMacSet("dr", "5");  //0= min, 7=max

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

void initOTAA(const char *appEui, const char *appKey) {
    _otaa = true;
    char response[BUFFER_SIZE];

    // Clear UART buffer
    while (uart_read_bytes(LORA_UART_NUM, (uint8_t *)response, sizeof(response), 100 / portTICK_PERIOD_MS) > 0);

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
    
    // Try joining twice
    for (int i = 0; i < 2 && !joined; i++)
    {
        sendRawCommand("mac join otaa", response, sizeof(response));
        
        if (strncmp(response, "accepted", 8) == 0)
        {
            joined = true;
            vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    return joined;
}

// Function to send a command over UART and read the response
bool sendRawCommand(const char *command, char *response, size_t response_size)
{
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
