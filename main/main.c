// #include <stdio.h>

#include <freertos/FreeRTOS.h>
// #include <esp_system.h>
// #include <esp_timer.h>
#include <esp_event.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include "drivers/WIFIDriver.h"
#include "drivers/CANDriver.h"
#include "GUB2.h"
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
    esp_log_level_set("vfs_fat", ESP_LOG_INFO);
}

#define ARRAY_SIZE_OFFSET   2   //Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

// Simple comparison of task numbers for sorting
int compareTaskNumber (const void * a, const void * b) {
    return ( ((TaskStatus_t*)a)->xTaskNumber - ((TaskStatus_t*)b)->xTaskNumber );
};

/**
 * Prints out task statistics in a top type format
*/
static esp_err_t printTaskStats(TickType_t xTicksToWait, bool showHeap) {
    TaskStatus_t *startArray = NULL, *endArray = NULL;
    UBaseType_t startArraySize, endArraySize;
    uint32_t startRunTime, endRunTime;
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
    startArraySize = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    startArray = malloc(sizeof(TaskStatus_t) * startArraySize);
    if (startArray == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get current task states
    startArraySize = uxTaskGetSystemState(startArray, startArraySize, &startRunTime);
    if (startArraySize == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    vTaskDelay(xTicksToWait);

    //Allocate array to store tasks states post delay
    endArraySize = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
    endArray = malloc(sizeof(TaskStatus_t) * endArraySize);
    if (endArray == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }
    //Get post delay task states
    endArraySize = uxTaskGetSystemState(endArray, endArraySize, &endRunTime);
    if (endArraySize == 0) {
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    //Calculate totalElapsedTime in units of run time stats clock period.
    uint32_t totalElapsedTime = (endRunTime - startRunTime);
    if (totalElapsedTime == 0) {
        ret = ESP_ERR_INVALID_STATE;
        goto exit;
    }

    //sort start array
    qsort (startArray, startArraySize, sizeof(TaskStatus_t), compareTaskNumber);

    //print heap info
    if(showHeap){
        printf("==============================================================\n");
        heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
    }
    printf("==============================================================\n");
    printf("| ## |    Task    | Core | Run Time | Percentage |   state   |\n");
    //Match each task in startArray to those in the endArray
    for (int i = 0; i < startArraySize; i++) {
        int k = -1;
        for (int j = 0; j < endArraySize; j++) {
            if (startArray[i].xHandle == endArray[j].xHandle) {
                k = j;
                //Mark that task have been matched by overwriting their handles
                startArray[i].xHandle = NULL;
                endArray[j].xHandle = NULL;
                break;
            }
        }

        eTaskState taskState = startArray[i].eCurrentState;
        int coreID = startArray[i].xCoreID == tskNO_AFFINITY ? -1 : startArray[i].xCoreID;

        //Check if matching task found
        if (k >= 0) {
            uint32_t taskElapsedTime = endArray[k].ulRunTimeCounter - startArray[i].ulRunTimeCounter;
            uint32_t percentageTime = (taskElapsedTime * 100UL) / (totalElapsedTime * portNUM_PROCESSORS);
            printf("| %2u | %10s | %4d | %8"PRIu32" | %9"PRIu32"%% | %s |\n", startArray[i].xTaskNumber,
                startArray[i].pcTaskName, coreID, taskElapsedTime, percentageTime, stateArray[taskState < 7 ? taskState : 6]);

        } else if (startArray[i].xHandle != NULL) { //Print unmatched tasks: Deleted
            printf("| %2u | %10s | %4d |   NONE   |    NONE    | %s |\n", startArray[i].xTaskNumber,
                startArray[i].pcTaskName, coreID, "DELETED  ");
        }
    }

    for (int i = 0; i < endArraySize; i++) {
        if (endArray[i].xHandle != NULL) {
            printf("| %s | Created\n", endArray[i].pcTaskName);
        }
    }
    printf("==============================================================\n");
    ret = ESP_OK;

exit:    //Common return path
    free(startArray);
    free(endArray);
    return ret;
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
    
    // vTaskDelay(pdMS_TO_TICKS(1000)); //delay for a bit to give time for debugger to attach
    
    ESP_LOGD(TAG, "Initializing GUB");
    GUBInit();

    //############################################
    //#####             WiFi                ######
    //############################################

    // Initialize NVS needed by Wi-Fi
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_softap();

    //Start File server
    start_file_server(SD_CARD_BASE_PATH);

    // GUBStart();

    //main loop, not much here since most stuff is handled through the GUB task that can leverage both cores
    while (1)
    {
        // Print Task information if debugging
        if(LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG){
            printTaskStats(pdMS_TO_TICKS(1000), false);
            //print GUB Stats
            printGUBStatus();
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
