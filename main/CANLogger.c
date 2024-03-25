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
        return -3;
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
    return 0;
}

/**
 * Write the csv file header to the first file
*/
int writeLogHeader(){
    if(fileStatus.CANFile == NULL) return -2;
    if(!xSemaphoreTake(fileStatus.fileMutex, pdMS_TO_TICKS(5))) return -1;

    fprintf(fileStatus.CANFile, "TS,BUS,ID,SEQ,DLC,DATAn\r\n");
    fileStatus.headerWriten = true;

    xSemaphoreGive(fileStatus.fileMutex);
    return 0;
}

/**
 * Initialize the CAN logger
*/
int canLoggerInit(){
    fileStatus.fileMutex = xSemaphoreCreateMutex();
    fileStatus.splitNumber = 0;
    fileStatus.duplicateNumber = 0;
    fileStatus.filePath = &absPath[0];

    // Make log directory if it doesn't exist
    struct stat st;
    if (stat(SD_CARD_CAN_LOG_PATH, &st) == -1) {
        mkdir(SD_CARD_CAN_LOG_PATH, 0777);
    }

    if(!createBaseLogName()){
        canLoggerOpenFile(false);
        writeLogHeader();
    }

    return 0;
}

/**
 * Process periodic logging tasks
*/
int canLoggerUpdate(){
    if(fileStatus.CANFile == NULL){
        if(fileStatus.baseName == NULL){
            createBaseLogName();
        }
        canLoggerOpenFile(false);
    }

    if(!fileStatus.headerWriten)
        writeLogHeader();

    if(fileStatus.totalBytesWritten > MAX_LOG_SIZE){
        canLoggerCloseFile();
        fileStatus.splitNumber++;
        canLoggerOpenFile(false);
    }

    if(fileStatus.totalBytesWritten - fileStatus.bytesWrittenAtFlush > FLUSH_SIZE_THRESHOLD || 
            esp_timer_get_time() - fileStatus.lastFlushTime > FLUSH_LOG_INTERVAL){
        canLoggerFlushFile();
    }

    return fileStatus.CANFile != NULL;
}

/**
 * Save the received can messages to the log
 * @param msg the message to log
*/
int canLoggerProcessMessage(CANMessage_t const *msg){
    if(fileStatus.CANFile == NULL) return -2;
    if(!xSemaphoreTake(fileStatus.fileMutex, pdMS_TO_TICKS(5))) return -1;

    fprintf(fileStatus.CANFile, "%lld,%u,%lx,%lu,%u,",
        msg->timestamp,
        msg->bus,
        msg->ID,
        msg->SEQ,
        msg->DLC
    );

    for(int i=0; i<msg->DLC; i++){
        fprintf(fileStatus.CANFile,"%x,", msg->payload[i]);
    }

    fprintf(fileStatus.CANFile,"\r\n");

    fileStatus.totalBytesWritten = ftell(fileStatus.CANFile);

    xSemaphoreGive(fileStatus.fileMutex);
    return 0;
}

/**
 * Open a log file for writing.
 * @param reopenFile specifies if the file should be reopened to continue writing or a new one should be made
*/
int canLoggerOpenFile(bool reopenFile){
    if(fileStatus.baseName == NULL) return -3;
    if(fileStatus.CANFile != NULL){
        ESP_LOGW(TAG, "File already opened! Reopening");
        canLoggerCloseFile();
    }

    if(!xSemaphoreTake(fileStatus.fileMutex, pdMS_TO_TICKS(5))) return -1;
    snprintf(fileStatus.filePath, 100, "%s/%s-%d.csv", SD_CARD_CAN_LOG_PATH, fileStatus.baseName, fileStatus.splitNumber);

    if(reopenFile){
        fileStatus.CANFile = fopen(fileStatus.filePath, "a");
    } else {
        fileStatus.CANFile = fopen(fileStatus.filePath, "w");
    }

    if(fileStatus.CANFile) {
        //get file size
        fseek(fileStatus.CANFile, 0L, SEEK_END);
        fileStatus.totalBytesWritten = ftell(fileStatus.CANFile);
        fseek(fileStatus.CANFile, 0, SEEK_SET);

        //reset stats
        fileStatus.bytesWrittenAtFlush = fileStatus.totalBytesWritten;
        fileStatus.lastFlushTime = esp_timer_get_time();
    }

    xSemaphoreGive(fileStatus.fileMutex);
    return 0;
}

/**
 * Flush the internal buffers to the file.
*/
int canLoggerFlushFile(){
    if(fileStatus.CANFile == NULL) return -2;
    if(!xSemaphoreTake(fileStatus.fileMutex, pdMS_TO_TICKS(5))) return -1;

    fflush(fileStatus.CANFile);
    fsync(fileno(fileStatus.CANFile));
    fileStatus.lastFlushTime = esp_timer_get_time();
    fileStatus.bytesWrittenAtFlush = fileStatus.totalBytesWritten;

    xSemaphoreGive(fileStatus.fileMutex);
    return 0;
}

/**
 * Close the log file.
*/
int canLoggerCloseFile(){
    if(!xSemaphoreTake(fileStatus.fileMutex, pdMS_TO_TICKS(5))) return -1;

    if(fileStatus.CANFile != NULL){
        fclose(fileStatus.CANFile);
    }

    fileStatus.CANFile = NULL;
    xSemaphoreGive(fileStatus.fileMutex);
    return 0;
}