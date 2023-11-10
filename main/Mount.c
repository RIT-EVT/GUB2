#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "GUB2.h"

int mountSDCard(const char* basePath){
    ESP_LOGI("SD", "Initilizing SD Card");

    esp_err_t ret;
    esp_vfs_fat_mount_config_t mountConfig = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 0
    };

    sdmmc_card_t *card;

    return 0;
}