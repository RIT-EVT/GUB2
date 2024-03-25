#ifndef CAN_LOGGER_H
#define CAN_LOGGER_H

#include <stdbool.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "GUB_Conf.h"
#include "drivers/CANDriver.h"

struct CANFileStatus {
    FILE *CANFile;
    SemaphoreHandle_t fileMutex;

    // file name information
    char *baseName;
    char *filePath;
    int splitNumber;
    int duplicateNumber;

    // stats for file control
    uint32_t totalBytesWritten;
    uint32_t bytesWrittenAtFlush;
    int64_t lastFlushTime;
    bool headerWriten;
};

extern const char *CAN_LOG_PATH;

// Logger setup
int canLoggerInit();
int canLoggerUpdate();

// Message processing
int canLoggerProcessMessage(CANMessage_t const *msg);

// Log file methods
int canLoggerOpenFile(bool reopenFile);
int canLoggerFlushFile();
int canLoggerCloseFile();

#endif // CAN_LOGGER_H