#include "CANLogger.h"

#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <dirent.h> 
#include <errno.h>
#include <time.h>
#include <sys/stat.h>

#include <esp_timer.h>
#include <esp_log.h>

#include "esp_sleep.h"

// #include "GUB2.h"

static const char *TAG = "CANLogger";

const char *CAN_LOG_PATH = SD_CARD_CAN_LOG_PATH;

static struct CANFileStatus fileStatus;
static char fileName[30];
static char absPath[100];

/**
 * Sets the base log file name of the format '<BaseName><duplicate>_<timestamp>'
*/
int createBaseLogName(){
    time_t logtime = time(NULL);
    bool validName = true;
    
    // open directory to list existing files
    struct dirent *dir;
    DIR *d = opendir(CAN_LOG_PATH);
    if (!d) {
        ESP_LOGE(TAG, "Unable to open logging directory!");
        return LOGGER_ERR_BAD_PATH;
    }

    do {
        //create base file name
        snprintf(fileName, 30, "%s%d_%llu", CAN_LOG_BASE_NAME, fileStatus.duplicateNumber, logtime);

        //compare base name with files in directory to find a unique name;
        validName = true;
        rewinddir(d); 
        while ((dir = readdir(d)) != NULL) {
            if(strncmp(dir->d_name, fileName, strlen(fileName)) == 0){
                fileStatus.duplicateNumber++;
                validName = false;
                ESP_LOGD(TAG, "%s already exist for base %s\r\n", dir->d_name, fileName);
                break;
            }
        }
    } while (!validName);

    fileStatus.baseName = &fileName[0];
    closedir(d);
    return LOGGER_ERR_OK;
}

/**
 * Write the csv file header to the first file
*/
int writeLogHeader(){
    /* No Longer Used */
    return LOGGER_ERR_OK;
}

/**
 * Initialize the CAN logger
*/
int canLoggerInit(){
    fileStatus.fileMutex = xSemaphoreCreateMutex();
    fileStatus.duplicateNumber = 0;
    fileStatus.filePath = &absPath[0];

    // Make log directory if it doesn't exist
    struct stat st;
    if (stat(SD_CARD_CAN_LOG_PATH, &st) == -1) {
        mkdir(SD_CARD_CAN_LOG_PATH, 0777);
    }

    if(createBaseLogName() == LOGGER_ERR_OK){
        canLoggerOpenFile(false);
        writeLogHeader();
    }

    return 0;
}

/**
 * Process periodic logging tasks
*/
int canLoggerUpdate(){
    // Eh??
    // if(fileStatus.CANFile == NULL){
    //     if(fileStatus.baseName == NULL){
    //         createBaseLogName();
    //     } else {
    //         canLoggerOpenFile(false);
    //     }
    // }
    //
    // if(!fileStatus.headerWriten)
    //     writeLogHeader();
    //
    // if(fileStatus.totalBytesWritten > MAX_LOG_SIZE){
    //     canLoggerCloseFile();
    //     fileStatus.splitNumber++;
    //     canLoggerOpenFile(false);
    // }
    //
    // if(fileStatus.totalBytesWritten - fileStatus.bytesWrittenAtFlush > FLUSH_SIZE_THRESHOLD ||
    //         esp_timer_get_time() - fileStatus.lastFlushTime > FLUSH_LOG_INTERVAL){
    //     canLoggerFlushFile();
    // }
    //
    // return fileStatus.CANFile != NULL;
    return 0;
}

/**
 * Save the received can messages to the log
 * @param msg the message to log
*/
int canLoggerProcessMessage(CANMessage_t const *msg){
    if(fileStatus.CANFile == NULL) return LOGGER_ERR_NOT_OPEN;
    if(!xSemaphoreTake(fileStatus.fileMutex, pdMS_TO_TICKS(5))) return LOGGER_ERR_SEMAPHORE_TIMEOUT;
    if (msg->ID == 0x0FF) {
        // RIP GUB :(
        canLoggerFlushFile();
        canLoggerCloseFile();
        ESP_LOGI(TAG, "GUB STOPPED!", msg->ID);
        esp_deep_sleep_start();
        return LOGGER_ERR_OK;
    }

    // Allocate this frame
    mdf_can_frame_t* canFrame = calloc(1, sizeof(mdf_can_frame_t));
    // printf("%08lX\n", *((uint32_t*)(msg->payload)));
    memcpy(&canFrame->data[0], &msg->payload[0], 8);
    canFrame->id = msg->ID;
    canFrame->dlc = msg->DLC;

    mdf_logger_write(fileStatus.CANFile, canFrame, 0);

    free(canFrame);

    xSemaphoreGive(fileStatus.fileMutex);
    return LOGGER_ERR_OK;
}

/**
 * Open a log file for writing.
 * @param reopenFile specifies if the file should be reopened to continue writing or a new one should be made
*/
int canLoggerOpenFile(bool append){
    if(fileStatus.baseName == NULL) return LOGGER_ERR_ILLEGAL_NAME;
    if(fileStatus.CANFile != NULL){
        ESP_LOGW(TAG, "File already opened! Reopening");
        canLoggerCloseFile();
    }

    if(!xSemaphoreTake(fileStatus.fileMutex, pdMS_TO_TICKS(5))) return LOGGER_ERR_SEMAPHORE_TIMEOUT;
    snprintf(fileStatus.filePath, 100, "%s/%s-%d.mf4", SD_CARD_CAN_LOG_PATH, fileStatus.baseName, fileStatus.duplicateNumber);

    ESP_LOGI(TAG, "filename: %s", fileStatus.filePath);
    int status = mdf_logger_open(fileStatus.filePath, 0, 0, &fileStatus.CANFile);

    if (status == LOGGER_ERR_OK && fileStatus.CANFile != NULL) {
        ESP_LOGI(TAG, "File opened successfully");
    }
    else {
        xSemaphoreGive(fileStatus.fileMutex);
        ESP_LOGE(TAG, "Error opening CAN log file %d", status);
        return status;
    }
    // if(fileStatus.CANFile) {
    //     //get file size
    //     fseek(fileStatus.CANFile, 0L, SEEK_END);
    //     fileStatus.totalBytesWritten = ftell(fileStatus.CANFile);
    //     fseek(fileStatus.CANFile, 0, SEEK_SET);
    //
    //     //reset stats
    //     fileStatus.bytesWrittenAtFlush = fileStatus.totalBytesWritten;
    //     fileStatus.lastFlushTime = esp_timer_get_time();
    // }

    xSemaphoreGive(fileStatus.fileMutex);
    return LOGGER_ERR_OK;
}

/**
 * Flush the internal buffers to the file.
*/
int canLoggerFlushFile(){
    return mdf_logger_flush(fileStatus.CANFile);
    // if(fileStatus.CANFile == NULL) return LOGGER_ERR_NOT_OPEN;
    // if(!xSemaphoreTake(fileStatus.fileMutex, pdMS_TO_TICKS(5))) return LOGGER_ERR_SEMAPHORE_TIMEOUT;
    //
    // // Force file writing, both sync and flush are necessary to force buffers to disk.
    // fflush(fileStatus.CANFile);
    // fsync(fileno(fileStatus.CANFile));
    // fileStatus.lastFlushTime = esp_timer_get_time();
    // fileStatus.bytesWrittenAtFlush = fileStatus.totalBytesWritten;
    //
    // xSemaphoreGive(fileStatus.fileMutex);
    // return LOGGER_ERR_OK;
}

/**
 * Close the log file.
*/
int canLoggerCloseFile(){
    if(!xSemaphoreTake(fileStatus.fileMutex, pdMS_TO_TICKS(5))) return LOGGER_ERR_NOT_OPEN;

    if(fileStatus.CANFile != NULL){
        mdf_logger_flush(fileStatus.CANFile);
        mdf_logger_close(fileStatus.CANFile);
    }

    fileStatus.CANFile = NULL;
    xSemaphoreGive(fileStatus.fileMutex);
    return LOGGER_ERR_OK;
}