#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <time.h>

//TODO Remove unneeded imports
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <lwip/inet.h>
#include <esp_ipc.h>

#include "drivers/WIFIDriver.h"
#include "drivers/CANDriver.h"
#include "GUB2.h"
// #include "drivers/Driver_MCP251863.h"
#include "Fileserver.h"

#include "GUB_Conf.h"

static const char *TAG = "MAIN";

MCP251XFD_BitTimeStats MCP251863_BTStats;
uint32_t MCP251863_SYSCLK1;

void reduceLogging(){
    esp_log_level_set("spi_master", ESP_LOG_INFO);
    esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
    esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
    esp_log_level_set("httpd_parse", ESP_LOG_ERROR);
    esp_log_level_set("event", ESP_LOG_INFO);
}

// int writeCanMessages(FILE *f, RingbufHandle_t buffer){

//     if(getNumItemsInQueue(buffer) < 10) return 0;
//     if(f == NULL) return -1;

//     int count = 0;
//     size_t itemSize;
//     MCP251XFD_CANMessage *receivedMessage;
//     while((receivedMessage = (MCP251XFD_CANMessage*)xRingbufferReceive(buffer, &itemSize, pdMS_TO_TICKS(100))) != NULL){
//         fwrite(receivedMessage, sizeof(MCP251XFD_CANMessage), 1, f);
//         fwrite(receivedMessage->PayloadData, sizeof(uint8_t), receivedMessage->DLC, f);
//         fputs("\n", f); 
//         count++;
//         free(receivedMessage->PayloadData);
//         vRingbufferReturnItem(buffer, (void*) receivedMessage);
//     }
//     ESP_LOGI("SDWrite", "Writing messages to SD card; %d wrote", count);
//     fflush(f);
//     fsync(fileno(f));
//     return 1;
// }

// int writeMessage(FILE *f, MCP251XFD_CANMessage *receivedMessage){
//     if(f != NULL){
//         fwrite(receivedMessage, sizeof(MCP251XFD_CANMessage), 1, f);
//         fwrite(receivedMessage->PayloadData, sizeof(uint8_t), receivedMessage->DLC, f);
//         fputs("\n", f);
//         return 0;
//     }
//     return -1;
// }

#define ARRAY_SIZE_OFFSET   2   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

int compareTaskNumber (const void * a, const void * b) {
    return ( ((TaskStatus_t*)a)->xTaskNumber - ((TaskStatus_t*)b)->xTaskNumber );
};

static esp_err_t print_real_time_stats(TickType_t xTicksToWait, bool showHeap) {
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    uint32_t start_run_time, end_run_time;
    esp_err_t ret;

    char *stateArray[7] = {
        "RUNNING  ",
        "READY    ",      
        "BLOCKED  ",    
        "SUSPENDED",  
        "DELETED  ",    
        "INVALID  ",
        "UNKNOWN  "
    };

    //Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    start_array = malloc(sizeof(TaskStatus_t) * start_array_size);
    if (start_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    vTaskDelay(xTicksToWait);

    //Allocate array to store tasks states post delay
    end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    end_array = malloc(sizeof(TaskStatus_t) * end_array_size);
    if (end_array == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get post delay task states
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    //Calculate total_elapsed_time in units of run time stats clock period.
    uint32_t total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto exit;
    }

    //sort start array
    qsort (start_array, start_array_size, sizeof(TaskStatus_t), compareTaskNumber);

    //print heap info
    // printf("\x1B[%dA\x1B[0J==============================================================\n", start_array_size+15);
    if(showHeap){
        printf("==============================================================\n");
        heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
    }
    printf("==============================================================\n");
    printf("| ## |    Task    | Core | Run Time | Percentage |   state   |\n");
    //Match each task in start_array to those in the end_array
    for (int i = 0; i < start_array_size; i++) {
        int k = -1;
        for (int j = 0; j < end_array_size; j++) {
            if (start_array[i].xHandle == end_array[j].xHandle) {
                k = j;
                //Mark that task have been matched by overwriting their handles
                start_array[i].xHandle = NULL;
                end_array[j].xHandle = NULL;
                break;
            }
        }

        eTaskState taskState = start_array[i].eCurrentState;
        int coreID = start_array[i].xCoreID == tskNO_AFFINITY ? -1 : start_array[i].xCoreID;

        //Check if matching task found
        if (k >= 0) {
            uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
            uint32_t percentage_time = (task_elapsed_time * 100UL) / (total_elapsed_time * portNUM_PROCESSORS);
            printf("| %2u | %10s | %4d | %8"PRIu32" | %9"PRIu32"%% | %s |\n", start_array[i].xTaskNumber,
                start_array[i].pcTaskName, coreID, task_elapsed_time, percentage_time, stateArray[taskState < 7 ? taskState : 6]);

        } else if (start_array[i].xHandle != NULL) { //Print unmatched tasks: Deleted
            printf("| %2u | %10s | %4d |   NONE   |    NONE    | %s |\n", start_array[i].xTaskNumber,
                start_array[i].pcTaskName, coreID, "DELETED  ");
        }
    }

    for (int i = 0; i < end_array_size; i++) {
        if (end_array[i].xHandle != NULL) {
            printf("| %s | Created\n", end_array[i].pcTaskName);
        }
    }
    printf("==============================================================\n");
    ret = ESP_OK;

exit:    //Common return path
    free(start_array);
    free(end_array);
    return ret;
}

void installGPIOISRService(void *arg){
    // Forces the isr_core_id in the gpio_context to core 1. This is a hack.
    gpio_intr_enable(PIN_NUM_CAN_RX_INT);
    gpio_intr_disable(PIN_NUM_CAN_RX_INT);
}

#define MCP251863_FIFO_COUNT    2

void app_main(void)
{
    // Start of main program, reduce logging info
    ESP_LOGI(TAG, "Starting GUB2");
    ESP_LOGD(TAG, "Debug logging Enabled");
    reduceLogging();

    // sys_delay_ms(10000);

    // Create default event loop needed by the  main app
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //setup CAN SPI bus
    ESP_LOGD(TAG, "Setting up SPI");
    // spi_device_handle_t spi;
    
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_CAN_MISO,
        .mosi_io_num=PIN_NUM_CAN_MOSI,
        .sclk_io_num=PIN_NUM_CAN_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
    };

    ESP_ERROR_CHECK(spi_bus_initialize(CAN_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    print_real_time_stats(pdMS_TO_TICKS(1000), false);
    
    // vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGD(TAG, "Starting CAN.");
    // CAN bus driver setup. Don't want to miss anything so do this first!
    CANDriverInit();

    CANDriverAddBus(0, PIN_NUM_CAN_CS, PIN_NUM_CAN_RX_INT);
    
    // ESP_ERROR_CHECK(spi_bus_add_device(CANSPI_HOST, &devcfgCAN, &spi));
    ESP_LOGD(TAG, "Starting GUB control.");
    
    GUBInit();

    // Install GPIO service on Core 1, all gpio isr handlers will be processed on core 1.
    esp_ipc_call_blocking(1, installGPIOISRService, 0);
    gpio_install_isr_service(0);

    // int r = mountSDCard(base_path, &card);
    // ESP_LOGI("SD_CARD", "SD card initialized with status %d", r);
    // if(!r){
    //     sdmmc_card_print_info(stdout, &card);
    // }
    
    // listDir(base_path);
    // listDir(canLogPath);

    // ESP_LOGI("Main Loop","transmit result: %d", transmitCount(&MCP251863_CAN1, 1));

    // char pathBuf[100];
    struct stat st;
    if (stat(canLogPath, &st) == -1) {
        mkdir(canLogPath, 0777);
    }

    //############################################
    //#####             WiFi                ######
    //############################################

    // Initialize NVS needed by Wi-Fi
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_softap();

    

    //Start File server
    // start_file_server(SDcardBasePath);

    GUBStart();

    //main loop, not much here since most stuff is handled through the GUB task that can leverage both cores
    while (1)
    {
        // printf("------------------------------\r\n");
        print_real_time_stats(pdMS_TO_TICKS(1000), false);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
