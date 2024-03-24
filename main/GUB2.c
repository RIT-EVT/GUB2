#include "GUB2.h"

#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "esp_timer.h"

//temp
#include "drivers/CANDriver.h"
#include "CANLogger.h"

static const char *TAG = "GUB2";

//GUB State
GUBState_t gubState;

//GUB main loop task, staticly allocated on the stack 
// static StackType_t GUBThreadStack[GUB_STACK_SIZE + 1];
// static StaticTask_t xGUBTaskBuffer;
TaskHandle_t xGUBTask;


//SD card 
sdmmc_card_t SDcard;

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

    // Install GPIO service on Core 1, all gpio isr handlers will be processed on core 1.
    esp_ipc_call_blocking(1, installGPIOISRService, 0);
    gpio_install_isr_service(0);
    
    ESP_LOGD(TAG, "Starting CAN.");
    // CAN bus driver setup. Don't want to miss anything so do this first!
    CANDriverInit(gubState.gubEvents, CAN_EVENT);
    CANDriverAddBus(0, PIN_NUM_CAN1_CS, PIN_NUM_CAN1_RX_INT, PIN_NUM_CAN1_STB);

    GUBInitLED();
    // GUBInitSDCard();
    int r = GUBMountSDCard(SDcardBasePath, &SDcard);
    ESP_LOGI("SD_CARD", "SD card initialized with status %d", r);
    if(!r){
        sdmmc_card_print_info(stdout, &SDcard);
    }
    
    canLoggerInit();

    listDir(SDcardBasePath);
    listDir(canLogPath);

    ESP_LOGI(TAG, "GUB Setup, starting main loop");
}

/**
 * Start the main loop for the GUB
*/
void GUBStart(){
    xTaskCreate(
        GUBloop,
        "GUB",
        GUB_STACK_SIZE,
        NULL,
        2,
        &xGUBTask
    );

    if(xGUBTask == NULL){
        ESP_LOGE(TAG, "Unable to create main task!");
    }
}

/**
 * Main loop for GUB task
*/
void GUBloop(void *pvParam){
    // uint32_t lastPrint = esp_timer_get_time();
    while (true)
    {
        GUBHeartbeatUpdate();
        CANDriverUpdate();

        // if(esp_timer_get_time() - lastPrint > 1000000){
        //     ESP_LOGD(TAG, "Unused Stack: %lu", uxTaskGetStackHighWaterMark2(xGUBTask));
        //     lastPrint = esp_timer_get_time();
        // }
        
        canLoggerUpdate();
        vTaskDelay(pdMS_TO_TICKS(5));
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

void printGUBStatus(){
    printf("CAN Driver Status:\r\n\tUnused Stack %lu\r\n", uxTaskGetStackHighWaterMark2(driver.driverTaskHandler));
    printCANDriverState();
}

/*****************************************
 *          SD card functions            *
 *****************************************/

/**
 * Mount the SD card
*/
int GUBMountSDCard(const char* basePath, sdmmc_card_t *card){
    ESP_LOGI(TAG, "Initilizing SD Card");

    esp_err_t ret;
    esp_vfs_fat_mount_config_t mountConfig = {
        .format_if_mount_failed = false,
        .max_files = MAX_FILE_HANDLERS,
        .allocation_unit_size = 0
    };

    ESP_LOGI(TAG, "Initializing SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;
    spi_bus_config_t busCFG = {
        .mosi_io_num = PIN_NUM_SD_MOSI,
        .miso_io_num = PIN_NUM_SD_MISO,
        .sclk_io_num = PIN_NUM_SD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    ret = spi_bus_initialize(host.slot, &busCFG, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return ret;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slotConfig = SDSPI_DEVICE_CONFIG_DEFAULT();
    slotConfig.gpio_cs = PIN_NUM_SD_CS;
    slotConfig.host_id = host.slot;
    slotConfig.gpio_cd = PIN_NUM_SD_CD;
    ret = esp_vfs_fat_sdspi_mount(basePath, &host, &slotConfig, &mountConfig, &card);

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
    sdmmc_card_print_info(stdout, card);

    return ESP_OK;
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