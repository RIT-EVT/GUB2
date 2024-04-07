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

enum LoggerError{
    LOGGER_ERR_OK = 0,
    LOGGER_ERR_NOT_OPEN = -1,
    LOGGER_ERR_ILLEGAL_NAME = -2,
    LOGGER_ERR_SEMAPHORE_TIMEOUT = -3,
    LOGGER_ERR_BAD_PATH = -4,
};

extern const char *CAN_LOG_PATH;

// Logger setup
int canLoggerInit();
int canLoggerUpdate();

// Message processing
int canLoggerProcessMessage(CANMessage_t const *msg);

// Log file methods
int canLoggerOpenFile(bool append);
int canLoggerFlushFile();
int canLoggerCloseFile();

#endif // CAN_LOGGER_H