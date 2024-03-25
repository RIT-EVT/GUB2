#ifndef CAN_LOGGER_H
#define CAN_LOGGER_H

#include <stdio.h>
#include <stdbool.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "GUB_Conf.h"
#include "drivers/CANDriver.h"

struct CANFileStatus
{
    FILE* CANFile;
    SemaphoreHandle_t fileMutex;

    // file name information
    char* baseName;
    char* filePath;
    int splitNumber;
    int duplicateNumber; 

    // stats for file control
    uint32_t totalBytesWritten;
    uint32_t bytesWrittenAtFlush;
    int64_t lastFlushTime;
    bool headerWriten;
};

static const char* CAN_LOG_PATH = SD_CARD_CAN_LOG_PATH;


int canLoggerInit();
int canLoggerUpdate();

int canLoggerProcessMessage(CANMessage_t const *msg);

int canLoggerOpenFile(bool reopenFile);
int canLoggerFlushFile();
int canLoggerCloseFile();

#endif //CAN_LOGGER_H