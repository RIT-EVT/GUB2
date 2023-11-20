#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "MCP251XFD.h"

#include "GUB2.h"
#include "Driver_MCP251863.h"

#define rbALIGN_MASK (0x03)
#define rbALIGN_SIZE( xSize )       ( ( xSize + rbALIGN_MASK ) & ~rbALIGN_MASK )

#define BUFFER_SIZE 40

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

void reduceLogging(){
    esp_log_level_set("spi_master", ESP_LOG_INFO);
    
}

int listDir(const char *base_path){
    DIR *dir = opendir(base_path);
    struct dirent *dp;
    int count = 0;
    long totalBytes = 0;
    
    if(dir == NULL){
        ESP_LOGE("DIR", "Unable to open directory \"%s\", got %d", base_path, errno);
        return -1;
    }

    printf("Contents of %s: \n", base_path);
    printf("Mode\t\tSize\t\tdate\t\t\t\tname\n");
    printf("----------\t----------\t---------------------------\t----------\n");
    while((dp = readdir(dir)) != NULL){
        char path[500];
        struct stat info;
        sprintf(path, "%s/%s", base_path, dp->d_name);
        if(!stat(path, &info)){
            struct tm ts;
            char permbuf[11];
            char timebuf[80];
            
            permbuf[0] = S_ISDIR(info.st_mode)      ? 'd' : '-';
            permbuf[1] = (info.st_mode & S_IRUSR)   ? 'r' : '-';
            permbuf[2] = (info.st_mode & S_IWUSR)   ? 'w' : '-';
            permbuf[3] = (info.st_mode & S_IXUSR)   ? 'x' : '-';
            permbuf[4] = (info.st_mode & S_IRGRP)   ? 'r' : '-';
            permbuf[5] = (info.st_mode & S_IWGRP)   ? 'w' : '-';
            permbuf[6] = (info.st_mode & S_IXGRP)   ? 'x' : '-';
            permbuf[7] = (info.st_mode & S_IROTH)   ? 'r' : '-';
            permbuf[8] = (info.st_mode & S_IWOTH)   ? 'w' : '-';
            permbuf[9] = (info.st_mode & S_IXOTH)   ? 'x' : '-';
            permbuf[10] = '\0';

            ts = *localtime(&info.st_mtime);
            strftime(timebuf, sizeof(timebuf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);

            printf("%s\t%10ld\t%s\t%s\n",permbuf, info.st_size, timebuf, dp->d_name);
            totalBytes += info.st_size;
            count++;
        } else{
             ESP_LOGE("DIR", "could not stat file \"%s\", got %d", path, errno);
            return -1;
        }
    }

    printf("Total: %d; %ld bytes\n", count, totalBytes);

    closedir(dir);
    return 0;
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

MCPError receiveMessage(MCP251XFD *dev, MCP251XFD_CANMessage *receivedMessage){
    eERRORRESULT ErrorExt1 = ERR_OK;
    eMCP251XFD_FIFOstatus FIFOstatus = 0;
    ErrorExt1 = MCP251XFD_GetFIFOStatus(dev, MCP251XFD_FIFO1, &FIFOstatus); // First get FIFO1 status
    if (ErrorExt1 != ERR_OK){
        ESP_LOGE("CANRX","Error getting FIFO Status %d", ErrorExt1);
        return ErrorExt1;
    } 
    if ((FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY) > 0) // Second check FIFO not empty
    {
        // uint8_t RxPayloadData[64]; // In this example, the FIFO1 have 64 bytes of payload
        // MCP251XFD_CANMessage ReceivedMessage;
        // receivedMessage.PayloadData = &RxPayloadData[0]; // Add receive payload data pointer to the message structure
        // that will be received
        ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO(dev, receivedMessage, MCP251XFD_PAYLOAD_8BYTE,
        NULL, MCP251XFD_FIFO1);
        if (ErrorExt1 == ERR_OK)
        {   
            return ERR_OK;
            // printCANMessage(&receivedMessage);
            // ESP_LOGI("CANRX", "%"PRIu32", Received Message #%"PRIu32" from 0x%08" PRIx32 " with %d bytes. ", MessageTimeStamp, ReceivedMessage.MessageSEQ, ReceivedMessage.MessageID, ReceivedMessage.DLC);
        }
    }else if(FIFOstatus != 0){
        ESP_LOGI("CANRX","FIFO Status %d", FIFOstatus);
    }

    return ERR__EMPTY_DATA;
}

uint32_t getNumItemsInQueue(RingbufHandle_t ringbuffer){
    unsigned int count;
    vRingbufferGetInfo(ringbuffer, NULL,NULL,NULL,NULL, &count);
    // printf("free: %u, read: %u, write: %u, acquire: %u, count, %u\n", freebytes, readbytes, writebytes, acquirebytes, count);
    return count;
}

MCPError handleMessageReceive(MCP251XFD *dev, RingbufHandle_t ringbuffer){
    eERRORRESULT ErrorExt1 = ERR_OK;
    eMCP251XFD_FIFOstatus FIFOstatus = 0;

    ErrorExt1 = MCP251XFD_GetFIFOStatus(dev, MCP251XFD_FIFO1, &FIFOstatus); // First get FIFO1 status
    if (ErrorExt1 != ERR_OK){
        ESP_LOGE("CANRX","Error getting FIFO Status %d", ErrorExt1);
        return ErrorExt1;
    } 

    if(FIFOstatus & MCP251XFD_RX_FIFO_OVERFLOW){
        ESP_LOGW("CANRX", "RX FIFO Overflow!");
        MCP251XFD_ClearFIFOOverflowEvent(dev, MCP251XFD_FIFO1);
    }

    if (FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY) // Second check FIFO not empty
    {
        uint8_t RxPayloadData[64]; // In this example, the FIFO1 have 64 bytes of payload
        MCP251XFD_CANMessage receivedMessage;
        receivedMessage.PayloadData = &RxPayloadData[0]; 

        ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO(dev, &receivedMessage, MCP251XFD_PAYLOAD_8BYTE, NULL, MCP251XFD_FIFO1);
        if (ErrorExt1 == ERR_OK)
        {   
            uint8_t *payload = (uint8_t*) malloc(sizeof(uint8_t) * receivedMessage.DLC);
            if(payload == NULL){
                ESP_LOGE("CANRX", "Unable to allocate buffer");
                return ERR__NULL_BUFFER;
            }
            memcpy(payload, receivedMessage.PayloadData, sizeof(uint8_t) * receivedMessage.DLC);
            receivedMessage.PayloadData = payload;

            if(!xRingbufferSend(ringbuffer, &receivedMessage, sizeof(MCP251XFD_CANMessage), pdMS_TO_TICKS(100))){
                ESP_LOGW("CANRX", "Receive buffer full! Lost message!");
                printCANMessage(&receivedMessage);
                free(payload);
                xRingbufferPrintInfo(ringbuffer);
                ESP_LOGD("CANRX", "Available Heap: %lu", esp_get_free_heap_size());
                return ERR__BUFFER_FULL;
            }

            // xRingbufferPrintInfo(ringbuffer);
            // ESP_LOGD("CANRX", "Available Heap: %lu", esp_get_free_heap_size());

            return ERR_OK;
        }
    } else if(FIFOstatus != 0){
        ESP_LOGI("CANRX","FIFO Status %d", FIFOstatus);
    }

    return ERR__EMPTY_DATA;
}

int writeCanMessages(FILE *f, RingbufHandle_t buffer){

    if(getNumItemsInQueue(buffer) < 10) return 0;
    if(f == NULL) return -1;

    int count = 0;
    size_t itemSize;
    MCP251XFD_CANMessage *receivedMessage;
    while((receivedMessage = (MCP251XFD_CANMessage*)xRingbufferReceive(buffer, &itemSize, pdMS_TO_TICKS(100))) != NULL){
        fwrite(receivedMessage, sizeof(MCP251XFD_CANMessage), 1, f);
        fwrite(receivedMessage->PayloadData, sizeof(uint8_t), receivedMessage->DLC, f);
        fputs("\n", f); 
        count++;
        free(receivedMessage->PayloadData);
        vRingbufferReturnItem(buffer, (void*) receivedMessage);
    }

    ESP_LOGI("SDWrite", "Writing messages to SD card; %d wrote", count);

    fflush(f);
    fsync(fileno(f));

    return 1;
}

int writeMessage(FILE *f, MCP251XFD_CANMessage *receivedMessage){
    if(f != NULL){
        fwrite(receivedMessage, sizeof(MCP251XFD_CANMessage), 1, f);
        fwrite(receivedMessage->PayloadData, sizeof(uint8_t), receivedMessage->DLC, f);
        fputs("\n", f);
        return 0;
    }
    return -1;
}

#define MCP251863_FIFO_COUNT    2

void app_main(void)
{
    ESP_LOGI(TAG, "Starting GUB2");
    ESP_LOGD(TAG, "debug logging");
    initLED();
    reduceLogging();

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

    MCP251XFD_RAMInfos MCP251863_CAN1_FIFO_RAMInfos[MCP251863_FIFO_COUNT];
    MCP251XFD_FIFO MCP251863_CAN1_FIFO_Conf[MCP251863_FIFO_COUNT] = {
        { 
            .Name = MCP251XFD_FIFO1, 
            .Size = MCP251XFD_FIFO_16_MESSAGE_DEEP, 
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

    //setup MCP25863
    eERRORRESULT ret;
    ret = MCP251863DeviceSetup(&MCP251863_CAN1, &MCP251863_CAN1_conf, MCP251863_CAN1_FIFO_Conf, MCP251863_FIFO_COUNT);

    //Get MCP25863 device info 
    eMCP251XFD_Devices device;
    uint8_t deviceId;
    uint8_t deviceRev;
    ret = MCP251XFD_GetDeviceID(&MCP251863_CAN1, &device, &deviceId, &deviceRev);
    ESP_LOGI("MCP INIT","Device ID result: %d", ret);
    ESP_LOGI("MCP INIT","Device %d, ID %x, REV %x", device, deviceId, deviceRev);

    const char* base_path = "/data";
    const char* canLogPath = "/data/CANLogs";
    sdmmc_card_t card;

    int r = mountSDCard(base_path, &card);
    ESP_LOGI("SD_CARD", "SD card initialized with status %d", r);
    if(!r){
        sdmmc_card_print_info(stdout, &card);
    }
    // printf(LOG_COLOR(LOG_COLOR_CYAN)"Register states after start\n");

    // GetAndShowMCP251XFD_SFRreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_CANSFRreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_FIFOreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_FILTERreg(&MCP251863_Driver1_Conf);
    
    listDir(base_path);
    listDir(canLogPath);

    ESP_LOGI("Main Loop","transmit result: %d", transmitCount(&MCP251863_CAN1, 1));

    // printf(LOG_COLOR(LOG_COLOR_CYAN)"Register states after transmit\n");

    // GetAndShowMCP251XFD_SFRreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_CANSFRreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_FIFOreg(&MCP251863_Driver1_Conf);
    // GetAndShowMCP251XFD_FILTERreg(&MCP251863_Driver1_Conf);

    uint8_t RxPayloadData[64]; // In this example, the FIFO1 have 64 bytes of payload
    MCP251XFD_CANMessage receivedMessage;
    receivedMessage.PayloadData = &RxPayloadData[0]; // Add receive payload data pointer to the message structure

    RingbufHandle_t messageBuffer = xRingbufferCreateNoSplit(sizeof(MCP251XFD_CANMessage), BUFFER_SIZE);
    // messageBuffer = xRingbufferCreate((rbALIGN_SIZE(sizeof(MCP251XFD_CANMessage)) + 8) * 20, RINGBUF_TYPE_NOSPLIT);

    xRingbufferPrintInfo(messageBuffer);

    char pathBuf[500];
    struct stat st;
    if (stat(canLogPath, &st) == -1) {
        mkdir(canLogPath, 0777);
    }

    // sprintf(pathBuf, "%s/%s", canLogPath, "test.bin");
    // FILE *ftest = fopen(pathBuf, "a");
    // if(ftest != NULL){
    //     fprintf(ftest, "Test String!");
    //     fclose(ftest);
    // }else{
    //     ESP_LOGE("FILE","Unable to open file \"%s\", got %d (%s)", pathBuf, errno, strerror(errno));
    // }

    sprintf(pathBuf, "%s/%s", canLogPath, "CanLog.bin");
    FILE *f = fopen(pathBuf, "wb");
    if(f == NULL){
        ESP_LOGE("FILE","Unable to open file \"%s\", got %d (%s)", pathBuf, errno, strerror(errno));
    }else{
        // setvbuf(f, NULL, _IOFBF, 256);
    }

    uint8_t data = 0;
    uint64_t time = esp_timer_get_time()/1000;
    uint64_t time2 = esp_timer_get_time()/1000;
    for(;;){
        if(esp_timer_get_time()/1000 - time > 1000){
            time = esp_timer_get_time()/1000;
            toggleLED();
            // transmitCount(&MCP251863_CAN1, data);
            // ESP_LOGI("Counter", "%d", data);
            // listDir(canLogPath);
            // data++;
            // fflush(f);
            // fclose(f);
            // FILE *f = fopen(pathBuf, "ab");
            // if(f == NULL){
            //     ESP_LOGE("FILE","Unable to open file \"%s\", got %d (%s)", pathBuf, errno, strerror(errno));
            // }
        }

        if(esp_timer_get_time()/1000 - time2 > 5000){
            time2 = esp_timer_get_time()/1000;
            listDir(canLogPath);
        }
        // ESP_LOGD("Main Loop","transmit result: %d", transmitCount(&MCP251863_Driver1_Conf, data));
        // ESP_LOGD("Main Loop", "recive result: %d", receiveMessage(&MCP251863_Driver1_Conf));
        
        if((ret = handleMessageReceive(&MCP251863_CAN1, messageBuffer)) == ERR_OK){
            writeCanMessages(f, messageBuffer);
            // printf("RX: ");
            // printCANMessage(&receivedMessage);
            // if(writeMessage(f, &receivedMessage)){
            //     ESP_LOGW("FILE","Error while writing!");
            // }

        } else if(ret != ERR__EMPTY_DATA){
            ESP_LOGI("CANRX","FIFO Status %d", ret);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    fclose(f);
}
