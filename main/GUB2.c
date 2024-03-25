#include "GUB2.h"

#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include <esp_log.h>
#include <driver/spi_master.h>
#include <esp_vfs_fat.h>
#include <esp_timer.h>

//temp
#include "drivers/CANDriver.h"
#include "CANLogger.h"

static const char *TAG = "GUB2";

//GUB State
GUBState_t gubState;

/**
 * A function for installing the GPIO ISR on a specific core.
*/
void installGPIOISRService(void *arg){
    // Forces the isr_core_id in the gpio_context to core 1. This is a hack.
    gpio_intr_enable(PIN_NUM_CAN1_RX_INT);
    gpio_intr_disable(PIN_NUM_CAN1_RX_INT);
}

/**
 * Setup the main GUB Control interface
*/
void GUBInit(){
    gubState.gubEvents = xEventGroupCreate();

    //setup CAN SPI bus
    ESP_LOGD(TAG, "Setting CAN up SPI bus");
    
    spi_bus_config_t canBuscfg={
        .miso_io_num=PIN_NUM_CAN_MISO,
        .mosi_io_num=PIN_NUM_CAN_MOSI,
        .sclk_io_num=PIN_NUM_CAN_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=0
    };

    ESP_ERROR_CHECK(spi_bus_initialize(CAN_SPI_HOST, &canBuscfg, SPI_DMA_CH_AUTO));

    // Install GPIO service on Core 1, all gpio isr handlers will be processed on core 1.
    esp_ipc_call_blocking(1, installGPIOISRService, 0);
    gpio_install_isr_service(0);
    
    ESP_LOGD(TAG, "Starting CAN.");
    // CAN bus driver setup. Don't want to miss anything so do this first!
    canDriverInit(gubState.gubEvents, CAN_EVENT);
    canDriverAddBus(0, PIN_NUM_CAN1_CS, PIN_NUM_CAN1_RX_INT, PIN_NUM_CAN1_STB);
    
    ESP_LOGI(TAG, "Initializing SD Card SPI bus");
    spi_bus_config_t sdBusCFG = {
        .mosi_io_num = PIN_NUM_SD_MOSI,
        .miso_io_num = PIN_NUM_SD_MISO,
        .sclk_io_num = PIN_NUM_SD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };
    
    //initialize SPI bus
    esp_err_t ret;
    ret = spi_bus_initialize(SD_SPI_HOST, &sdBusCFG, SDSPI_DEFAULT_DMA);
    if (ret == ESP_OK) {
        int r = GUBMountSDCard(SD_CARD_BASE_PATH);
        ESP_LOGI("SD_CARD", "SD card initialized with status %d", r);
        if(!r){
            sdmmc_card_print_info(stdout, gubState.SDcard);
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize bus.");
    }

    listDir(SD_CARD_BASE_PATH);

    //start up logger
    canLoggerInit();

    GUBInitLED();

    ESP_LOGI(TAG, "GUB Setup, starting main loop");
    xTaskCreate( GUBloop, "GUB", GUB_STACK_SIZE, NULL, 2, &gubState.mainTaskHandler);
}

/**
 * Main loop for GUB task
*/
void GUBloop(void *pvParam){

    while (true)
    {   
        GUBHeartbeatUpdate();
        checkSDCardStatus();
        if(gubState.sdCardState == SD_NOT_MOUNTED && esp_timer_get_time() - gubState.lastMountAttempt > 1000000){ // check every second
            gubState.lastMountAttempt = esp_timer_get_time();
            ESP_LOGW(TAG, "SD card not mounted! Attempting to remount...");
            GUBMountSDCard(SD_CARD_BASE_PATH);
        }

        canDriverUpdate();
        canLoggerUpdate();
        
        CANMessage_t message;
        if(canReceiveMessage(&message, pdMS_TO_TICKS(5))){
            canLoggerProcessMessage(&message);
        }
        
        //* needed if not relying to the timeout of the queue
        // vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * Setup the heartbeat led
*/
void GUBInitLED(){
    gubState.lastHeartbeat = 0;
    #if LED_IS_NEOPIXEL==1
    ESP_LOGI(TAG, "Heartbeat with RGB");
    led_strip_config_t strip_config = {
        .strip_gpio_num = PIN_NUM_HEARTBEAT,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &gubState.ledStrip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(gubState.ledStrip);

    #else
    ESP_LOGI(TAG, "Heartbeat with GPIO");
    gpio_reset_pin(PIN_NUM_HEARTBEAT);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(PIN_NUM_HEARTBEAT, GPIO_MODE_OUTPUT);
    #endif
}

/**
 * Toggle the state of the heartbeat LED
*/
void GUBToggleLED(){
    #if LED_IS_NEOPIXEL==1
    /* If the addressable LED is enabled */
    if (gubState.ledState) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(gubState.ledStrip, 0, 0, 16, 0);
        /* Refresh the strip to send data */
        led_strip_refresh(gubState.ledStrip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(gubState.ledStrip);
    }
    #else

    gpio_set_level(PIN_NUM_HEARTBEAT, gubState.ledState);
    #endif
    
    gubState.ledState = !gubState.ledState;
}

/**
 * Update heartbeat led
*/
void GUBHeartbeatUpdate(){
    if(esp_timer_get_time() - gubState.lastHeartbeat > HEARTBEAT_PERIOD){
        gubState.lastHeartbeat = esp_timer_get_time();
        GUBToggleLED();
    }
}

/**
 * print out the current status of the GUB
*/
void printGUBStatus(){
    printf("GUB status:\r\n\tUnused Stack %lu\r\n", uxTaskGetStackHighWaterMark2(gubState.mainTaskHandler));
    printCANDriverState();
}

/*****************************************
 *          SD card functions            *
 *****************************************/

/**
 * Mount the SD card
 * @param basePath the file base file path to mount the sd card to
*/
int GUBMountSDCard(const char* basePath){
    ESP_LOGI(TAG, "Initializing SD Card");

    if(gubState.sdCardState == SD_READY){
        return ESP_OK;
    }

    esp_err_t ret;
    esp_vfs_fat_mount_config_t mountConfig = {
        .format_if_mount_failed = false,
        .max_files = MAX_FILE_HANDLERS,
        .allocation_unit_size = 0
    };
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;

    gubState.sdCardState = SD_NOT_MOUNTED;

    //initialize the SDCard slot
    sdspi_device_config_t slotConfig = SDSPI_DEVICE_CONFIG_DEFAULT();
    slotConfig.gpio_cs = PIN_NUM_SD_CS;
    slotConfig.host_id = SD_SPI_HOST;
    slotConfig.gpio_cd = PIN_NUM_SD_CD;
    ret = esp_vfs_fat_sdspi_mount(basePath, &host, &slotConfig, &mountConfig, &gubState.SDcard);

    if (ret != ESP_OK){
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return ret;
    }

    ESP_LOGI(TAG, "SD card initialized");
    // sdmmc_card_print_info(stdout, card);
    gubState.sdCardState = SD_READY;

    return ESP_OK;
}

int checkSDCardStatus(){
    if(gubState.sdCardState != SD_READY)
        return ESP_FAIL;

    if(esp_timer_get_time() - gubState.lastSDCheck > SD_CARD_STATUS_CHECK_INTERVAL){
        gubState.lastSDCheck = esp_timer_get_time();
        esp_err_t ret = sdmmc_get_status(gubState.SDcard);
        if(ret != ESP_OK){
            gubState.sdCardState = SD_NOT_MOUNTED;
            esp_vfs_fat_sdcard_unmount(SD_CARD_BASE_PATH,gubState.SDcard);
        }

        return ret;
    }

    return 0;
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