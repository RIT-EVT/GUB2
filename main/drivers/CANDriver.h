/******************************************************************************
 * CANDriver.h
 *
 * The CANDriver serves as a high level interface for receiving CAN messages.
 * The driver will setup multiple CAN chips on a single SPI bus and handle
 * automatically receiving messages in a separate task. All received messages
 * are held in a single queue with each message designating the bus (receiver
 * chip) of origin.
 *****************************************************************************/
#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

// Imports
#include <driver/gpio.h>
#include <driver/spi_master.h>

#include <freertos/event_groups.h>
#include <freertos/message_buffer.h>
#include <freertos/semphr.h>

#include "GUB_Conf.h"
#include "drivers/Driver_MCP251863.h"

// Global Defines
#define CAN_FIFO_COUNT 2

#ifndef CAN_MAX_MESSAGE_SIZE
#define CAN_MAX_MESSAGE_SIZE 8
#endif

#if CAN_MAX_MESSAGE_SIZE < 8
#undef CAN_MAX_MESSAGE_SIZE
#define CAN_MAX_MESSAGE_SIZE 8
#endif

// set the payload sizes based off of the max message size
#if CAN_MAX_MESSAGE_SIZE <= 8
#define CAN_RX_PAYLOAD MCP251XFD_PAYLOAD_8BYTE
#define CAN_RX_PAYLOAD_SIZE 8
#elif CAN_MAX_MESSAGE_SIZE <= 12
#define CAN_RX_PAYLOAD_SIZE MCP251XFD_PAYLOAD_12BYTE
#define CAN_RX_PAYLOAD_SIZE 12
#elif CAN_MAX_MESSAGE_SIZE <= 16
#define CAN_RX_PAYLOAD_SIZE MCP251XFD_PAYLOAD_16BYTE
#define CAN_RX_PAYLOAD_SIZE 16
#elif CAN_MAX_MESSAGE_SIZE <= 20
#define CAN_RX_PAYLOAD_SIZE MCP251XFD_PAYLOAD_20BYTE
#define CAN_RX_PAYLOAD_SIZE 20
#elif CAN_MAX_MESSAGE_SIZE <= 24
#define CAN_RX_PAYLOAD_SIZE MCP251XFD_PAYLOAD_24BYTE
#define CAN_RX_PAYLOAD_SIZE 24
#elif CAN_MAX_MESSAGE_SIZE <= 32
#define CAN_RX_PAYLOAD_SIZE MCP251XFD_PAYLOAD_32BYTE
#define CAN_RX_PAYLOAD_SIZE 32
#elif CAN_MAX_MESSAGE_SIZE <= 48
#define CAN_RX_PAYLOAD_SIZE MCP251XFD_PAYLOAD_48BYTE
#define CAN_RX_PAYLOAD_SIZE 48
#elif CAN_MAX_MESSAGE_SIZE <= 64
#define CAN_RX_PAYLOAD_SIZE MCP251XFD_PAYLOAD_64BYTE
#define CAN_RX_PAYLOAD_SIZE 64
#endif

// CAN message struct for received messages
typedef struct {
    int64_t timestamp;
    uint32_t ID;
    uint32_t SEQ;
    uint8_t DLC;
    uint8_t bus;
    uint8_t payload[CAN_RX_PAYLOAD_SIZE];
} CANMessage_t;

// Device message statistics
typedef struct {
    uint32_t messageReceiveCount;
    uint32_t fifoErrorCount;
    uint32_t communicationErrorCount;
    uint32_t receiveBufferFullCount;
} CANDeviceStatistic_t;

// Device struct to represent a CAN Device
typedef struct {
    // MCP251XFD config
    MCP251XFD mcp251863;
    MCP251XFD_BitTimeStats MCP251863_BTStats;
    uint32_t MCP251863_SYSCLK1;
    // generic IO vars
    spi_device_handle_t spi;
    gpio_num_t CSPin;
    gpio_num_t InteruptPin;
    // Device States

    // Device Statistics
    CANDeviceStatistic_t stats;
    SemaphoreHandle_t statsMutex;
} CANDevice_t;

// Overall CAN driver struct to hold system state
typedef struct {
    CANDevice_t devices[CAN_BUS_COUNT]; // Device states
    QueueHandle_t messageBuffer;        // Combined messageBuffer
    EventGroupHandle_t messageEvents;   // Event handler for interrupts to trigger receive task
    EventGroupHandle_t globalEvents;    // Global GUB events to signal new messages are available;
    TaskHandle_t driverTaskHandler;
    uint16_t messageFlag;
    uint8_t deviceEventMask;
} CANDriver_t;

//Main driver setup methods
void canDriverInit(EventGroupHandle_t globalEvents, uint16_t messageFlag);
int canDriverAddBus(uint8_t bus, int csPin, int interruptPin, int standbyPin);

int canReceiveMessage(CANMessage_t *message, uint32_t timeoutTicks);
CANDeviceStatistic_t canGetStatistics(int bus, bool clear);

//! obsolete?
void canDriverUpdate();

// debugging methods
void printCANMessage(CANMessage_t const *msg);
void printCANDriverState();

#endif