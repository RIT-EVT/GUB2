#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <freertos/ringbuf.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "drivers/Driver_MCP251863.h"
#include "datatypes/CircularBuffer.h"
#include "GUB_Conf.h"

// #define CAN_BUS_COUNT  1
#define CAN_FIFO_COUNT  2

#ifndef CAN_MAX_MESSAGE_SIZE
    #define CAN_MAX_MESSAGE_SIZE 8
#endif

#if CAN_MAX_MESSAGE_SIZE < 8
    #undef CAN_MAX_MESSAGE_SIZE
    #define CAN_MAX_MESSAGE_SIZE 8
#endif

//set the payload sizes based off of the max message size
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

// abstracting away the specifics of the MCP251863 Library
typedef enum
{
  NO_MESSAGE_CTRL_FLAGS       = 0x00, //!< No Message Control Flags
  CAN20_FRAME                 = 0x00, //!< Indicate that the frame is a CAN2.0A/B
  CANFD_FRAME                 = 0x01, //!< Indicate that the frame is a CAN-FD
  NO_SWITCH_BITRATE           = 0x00, //!< The data bitrate is not switched (only CAN-FD frame)
  SWITCH_BITRATE              = 0x02, //!< The data bitrate is switched (only CAN-FD frame)
  REMOTE_TRANSMISSION_REQUEST = 0x04, //!< The frame is a Remote Transmission Request; not used in CAN FD
  STANDARD_MESSAGE_ID         = 0x00, //!< Clear the Identifier Extension Flag that set the standard ID format
  EXTENDED_MESSAGE_ID         = 0x08, //!< Set the Identifier Extension Flag that set the extended ID format
  TRANSMIT_ERROR_PASSIVE      = 0x10, //!< Error Status Indicator: In CAN to CAN gateway mode (CiCON.ESIGM=1), the transmitted ESI flag is a "logical OR" of T1.ESI and error passive state of the CAN controller; In normal mode ESI indicates the error status
} CANMessageCtrlFlags;

typedef struct
{
    uint32_t ID;                        
    uint32_t SEQ;                       
    CANMessageCtrlFlags flags;
    uint8_t DLC;   
    uint8_t bus;                
    uint8_t payload[CAN_RX_PAYLOAD_SIZE];      
}CANMessage;

// typedef struct{
//     uint32_t mess
// } CANDeviceState_t;

typedef struct
{
    //MCP251XFD config
    MCP251XFD device;
    MCP251XFD_BitTimeStats MCP251863_BTStats;
    uint32_t MCP251863_SYSCLK1;
    //generic IO vars
    spi_device_handle_t spi;
    gpio_num_t CSPin;
    gpio_num_t InteruptPin;
    //CAN state vars
    uint32_t fifoErrors;
    uint32_t receiveBufferFullErrors;
} CANDevice_t;

typedef struct
{
    CANDevice_t devices[CAN_BUS_COUNT];
    RingbufHandle_t messageBuffer;
    // CircularBuffer_t payloadBuffer;
} CANDriver_t;

static CANDriver_t driver;

void CANDriverInit();
int CANDriverAddBus(int bus, int csPin, int interruptPin, int standbyPin);

void CANDriverUpdate();

uint32_t CANDriverGetQueueLen();
void printCANMessage(CANMessage *msg);
void sendMessage(spi_device_handle_t spi, uint8_t data);

// void taskRegisterInterrupt();

// void IRAM_ATTR CANDriverISRHandler(void* arg);



#endif //CAN_DRIVER_H