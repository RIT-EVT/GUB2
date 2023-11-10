#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "MCP251XFD.h"

#include "GUB2.h"
#include "Driver_MCP251863.h"

static const char *TAG = "GUB2";

MCP251XFD_BitTimeStats MCP251863_BTStats;
uint32_t MCP251863_SYSCLK1;

// void sendMessages(spi_device_handle_t spi, uint8_t data){
//     ESP_LOGD("SPI3", "Preparing to send byte");
//     spi_device_acquire_bus(spi, portMAX_DELAY);
//     spi_transaction_t t;
//     memset(&t, 0 , sizeof(t));
//     t.length = 8;
//     t.tx_buffer = &data;
//     t.user = (void*) 1;
//     ESP_LOGD("SPI3", "Transmitting");
//     assert(spi_device_polling_transmit(spi, &t) == ESP_OK);

//     spi_device_release_bus(spi);
// }

void printCANMessage(MCP251XFD_CANMessage *msg){
    printf(LOG_COLOR(LOG_COLOR_PURPLE)"Message #%"PRIu32" ID: 0x%08" PRIx32 " with %d bytes of data: ", msg->MessageSEQ, msg->MessageID, msg->DLC);
    for(int i=0; i<msg->DLC; i++){
        printf("0x%X ", msg->PayloadData[i]);
    }
    printf("\n"LOG_RESET_COLOR);
}

MCPError MCP251863DeviceSetup(MCP251XFD *dev, MCP251XFD_Config *conf, MCP251XFD_FIFO *fifoConf, uint8_t fifoCount){
    if (dev == NULL) return ERR__CONFIGURATION;
    if (conf == NULL) return ERR__CONFIGURATION;
    if (fifoConf == NULL) return ERR__CONFIGURATION;
    if (fifoCount == 0) return ERR__CONFIGURATION;

    eERRORRESULT ret;
    MCP251XFD_Filter MCP251863_NoFilter[1] = {
        { 
            .Filter = MCP251XFD_FILTER0, 
            .EnableFilter = true, 
            .Match = MCP251XFD_MATCH_ONLY_SID,
            .AcceptanceID = MCP251XFD_ACCEPT_ALL_MESSAGES, 
            .AcceptanceMask = MCP251XFD_ACCEPT_ALL_MESSAGES, 
            .PointTo = MCP251XFD_FIFO1, 
        }, // 0x000..0x1FF
    };
    ret = Init_MCP251XFD(dev, conf);
    if(ret != ERR_OK) return ret;
    // ESP_LOGI("MCP INIT","Initialization result: %d", ret);
    ret = MCP251XFD_ConfigureTimeStamp(dev, true, MCP251XFD_TS_CAN20_SOF_CANFD_SOF, TIMESTAMP_TICK(MCP251863_CRYCLK), false);
    if(ret != ERR_OK) return ret;
    // ESP_LOGI("MCP INIT","Timestamp setup result: %d", ret);
    ret = MCP251XFD_ConfigureFIFOList(dev, fifoConf, fifoCount);
    if(ret != ERR_OK) return ret;
    //ESP_LOGI("MCP INIT","FIFO Setup result: %d", ret);
    ret = MCP251XFD_ConfigureFilterList(dev, MCP251XFD_D_NET_FILTER_DISABLE, MCP251863_NoFilter, 1);
    if(ret != ERR_OK) return ret;
    //ESP_LOGI("MCP INIT","Filter Setup result: %d", ret);
    ret = MCP251XFD_StartCANFD(dev);
    return ret;
}

uint32_t messageSEQ = 0;
MCPError transmitCount(MCP251XFD *dev, uint8_t num){
    eERRORRESULT ErrorExt1 = ERR_OK;
    eMCP251XFD_FIFOstatus FIFOstatus = 0;
    ErrorExt1 = MCP251XFD_GetFIFOStatus(dev, MCP251XFD_FIFO2, &FIFOstatus); // First get FIFO2 status
    if (ErrorExt1 != ERR_OK){
        ESP_LOGE("CANTX","Error getting FIFO Status %d", ErrorExt1);
        return ErrorExt1;
    } 
    if ((FIFOstatus & MCP251XFD_TX_FIFO_NOT_FULL) > 0) // Second check FIFO not full
    {
        uint8_t payloadData[4];
        payloadData[0] = num;
        payloadData[1] = num+1;
        payloadData[2] = num+2;
        payloadData[3] = num+3;


        MCP251XFD_CANMessage TansmitMessage;
        //***** Fill the message as you want *****
        TansmitMessage.MessageID = 0x123;
        TansmitMessage.MessageSEQ = messageSEQ;
        TansmitMessage.ControlFlags = MCP251XFD_STANDARD_MESSAGE_ID;
        TansmitMessage.DLC = 4;
        TansmitMessage.PayloadData = &payloadData[0];

        if(++messageSEQ > MCP2518FD_SEQUENCE_MAX) messageSEQ = 0;

        printCANMessage(&TansmitMessage);

        //Send message and flush
        ErrorExt1 = MCP251XFD_TransmitMessageToFIFO(dev, &TansmitMessage, MCP251XFD_FIFO2, true);
    }else{
        ESP_LOGW("CANTX", "FIFO FULL!, Status: %d", FIFOstatus);
    }
    
    return ErrorExt1;
}

MCPError receiveMessage(MCP251XFD *dev){
    eERRORRESULT ErrorExt1 = ERR_OK;
    eMCP251XFD_FIFOstatus FIFOstatus = 0;
    ErrorExt1 = MCP251XFD_GetFIFOStatus(dev, MCP251XFD_FIFO1, &FIFOstatus); // First get FIFO1 status
    if (ErrorExt1 != ERR_OK){
        ESP_LOGE("CANRX","Error getting FIFO Status %d", ErrorExt1);
        return ErrorExt1;
    } 
    if ((FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY) > 0) // Second check FIFO not empty
    {
        uint8_t RxPayloadData[64]; // In this example, the FIFO1 have 64 bytes of payload
        MCP251XFD_CANMessage ReceivedMessage;
        ReceivedMessage.PayloadData = &RxPayloadData[0]; // Add receive payload data pointer to the message structure
        // that will be received
        ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO(dev, &ReceivedMessage, MCP251XFD_PAYLOAD_8BYTE,
        NULL, MCP251XFD_FIFO1);
        if (ErrorExt1 == ERR_OK)
        {
            printCANMessage(&ReceivedMessage);
            // ESP_LOGI("CANRX", "%"PRIu32", Received Message #%"PRIu32" from 0x%08" PRIx32 " with %d bytes. ", MessageTimeStamp, ReceivedMessage.MessageSEQ, ReceivedMessage.MessageID, ReceivedMessage.DLC);
        }
    }else if(FIFOstatus != 0){
        ESP_LOGI("CANRX","FIFO Status %d", FIFOstatus);
    }

    return ErrorExt1;
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

    // spi_device_interface_config_t devcfgCAN={
    //     .clock_speed_hz=1000000,                //Clock out at 1 MHz
    //     .mode=0,                                //SPI mode 0
    //     .spics_io_num=PIN_NUM_CAN_CS,           //CS pin
    //     .queue_size=7,                          //We want to be able to queue 7 transactions at a time
    //     .pre_cb=NULL,  //Specify pre-transfer callback to handle D/C line
    // };

    ESP_ERROR_CHECK(spi_bus_initialize(CANSPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    // ESP_ERROR_CHECK(spi_bus_add_device(CANSPI_HOST, &devcfgCAN, &spi));
    
    MCP251XFD MCP251863_CAN1 = DEFAULT_MCP251863_DRIVER_CONFIG(&spi);
    MCP251XFD_Config MCP251863_CAN1_conf = DEFAULT_MCP251863_CONTROLLER_CONFIG(&MCP251863_SYSCLK1, &MCP251863_BTStats);
    MCP251863_CAN1.DriverConfig |= MCP251XFD_DRIVER_INIT_SET_RAM_AT_0;
    #define MCP251863_FIFO_COUNT  2
    // #define MCP251863_FILTER_COUNT 1

    MCP251XFD_RAMInfos MCP251863_CAN1_FIFO_RAMInfos[MCP251863_FIFO_COUNT];
    MCP251XFD_FIFO MCP251863_CAN1_FIFO_Conf[MCP251863_FIFO_COUNT] = {
        { 
            .Name = MCP251XFD_FIFO1, 
            .Size = MCP251XFD_FIFO_8_MESSAGE_DEEP, 
            .Payload = MCP251XFD_PAYLOAD_8BYTE,
            .Direction = MCP251XFD_RECEIVE_FIFO, 
            .ControlFlags = MCP251XFD_FIFO_NO_TIMESTAMP_ON_RX,
            .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT + MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
            .RAMInfos = &MCP251863_CAN1_FIFO_RAMInfos[0], 
        }, // SID: 0x000..0x1FF ; No EID
        { 
            .Name = MCP251XFD_FIFO2, 
            .Size = MCP251XFD_FIFO_4_MESSAGE_DEEP, 
            .Payload = MCP251XFD_PAYLOAD_8BYTE,
            .Direction = MCP251XFD_TRANSMIT_FIFO, 
            .Attempts = MCP251XFD_THREE_ATTEMPTS,
            .Priority = MCP251XFD_MESSAGE_TX_PRIORITY16, 
            .ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
            .InterruptFlags = MCP251XFD_FIFO_TX_ATTEMPTS_EXHAUSTED_INT + MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT,
            .RAMInfos = &MCP251863_CAN1_FIFO_RAMInfos[1], 
        },
    };

    // MCP251XFD_Filter MCP251863_FilterList[MCP251863_FILTER_COUNT] = {
    //     { 
    //         .Filter = MCP251XFD_FILTER0, 
    //         .EnableFilter = true, 
    //         .Match = MCP251XFD_MATCH_ONLY_SID,
    //         .AcceptanceID = MCP251XFD_ACCEPT_ALL_MESSAGES, 
    //         .AcceptanceMask = MCP251XFD_ACCEPT_ALL_MESSAGES, 
    //         .PointTo = MCP251XFD_FIFO1, 
    //     }, // 0x000..0x1FF
    // };

    eERRORRESULT ret;// = Init_MCP251XFD(&MCP251863_Driver1_Conf, &MCP251863_can_if1);
    // ESP_LOGI("MCP INIT","Initialization result: %d", ret);
    // ret = MCP251XFD_ConfigureTimeStamp(&MCP251863_Driver1_Conf, true, MCP251XFD_TS_CAN20_SOF_CANFD_SOF, TIMESTAMP_TICK(MCP251863_CRYCLK), false);
    // ESP_LOGI("MCP INIT","Timestamp setup result: %d", ret);
    // ret = MCP251XFD_ConfigureFIFOList(&MCP251863_Driver1_Conf, MCP251863_FIFO_Conf, MCP251863_FIFO_COUNT);
    // ESP_LOGI("MCP INIT","FIFO Setup result: %d", ret);
    // ret = MCP251XFD_ConfigureFilterList(&MCP251863_Driver1_Conf, MCP251XFD_D_NET_FILTER_DISABLE, MCP251863_FilterList, MCP251863_FILTER_COUNT);
    // ESP_LOGI("MCP INIT","Filter Setup result: %d", ret);

    // ret = MCP251XFD_StartCANFD(&MCP251863_Driver1_Conf);
    // ESP_LOGI("MCP INIT","Started in FD mode: %d", ret);

    ret = MCP251863DeviceSetup(&MCP251863_CAN1, &MCP251863_CAN1_conf, MCP251863_CAN1_FIFO_Conf, MCP251863_FIFO_COUNT);

    eMCP251XFD_Devices device;
    uint8_t deviceId;
    uint8_t deviceRev;
    ret = MCP251XFD_GetDeviceID(&MCP251863_CAN1, &device, &deviceId, &deviceRev);
    ESP_LOGI("MCP INIT","Device ID result: %d", ret);
    ESP_LOGI("MCP INIT","Device %d, ID %x, REV %x", device, deviceId, deviceRev);

    // printf(LOG_COLOR(LOG_COLOR_CYAN)"Register states after start\n");

    // GetAndShowMCP251XFD_SFRreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_CANSFRreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_FIFOreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_FILTERreg(&MCP251863_Driver1_Conf);

    ESP_LOGI("Main Loop","transmit result: %d", transmitCount(&MCP251863_CAN1, 1));

    // printf(LOG_COLOR(LOG_COLOR_CYAN)"Register states after transmit\n");

    // GetAndShowMCP251XFD_SFRreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_CANSFRreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_FIFOreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_FILTERreg(&MCP251863_Driver1_Conf);

    uint8_t data = 0;
    uint64_t time = esp_timer_get_time()/1000;
    for(;;){
        if(esp_timer_get_time()/1000 - time > 1000){
            time = esp_timer_get_time()/1000;
            toggleLED();
            transmitCount(&MCP251863_CAN1, data);
            ESP_LOGI("Counter", "%d", data);
            data++;
        }
        // ESP_LOGD("Main Loop","transmit result: %d", transmitCount(&MCP251863_Driver1_Conf, data));
        // ESP_LOGD("Main Loop", "recive result: %d", receiveMessage(&MCP251863_Driver1_Conf));
        receiveMessage(&MCP251863_CAN1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
