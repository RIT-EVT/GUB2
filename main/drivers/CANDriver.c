#include "drivers/CANDriver.h"

#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "Events.h"

static const char *TAG = "CANDriver";

/**
 * ! These are copied from ringbuf.c to fix the buffer getting stuck, if the tool chain is updated from 5.1.1
 * ! the GUB should be tested without this hack and this will hopefully be removed!
*/
typedef BaseType_t (*ringbufFuncPointer)(int args);
typedef struct RingbufferDefinition {
    size_t xSize;                               //Size of the data storage
    size_t xMaxItemSize;                        //Maximum item size
    UBaseType_t uxRingbufferFlags;              //Flags to indicate the type and status of ring buffer

    ringbufFuncPointer xCheckItemFits;     //Function to check if item can currently fit in ring buffer
    ringbufFuncPointer vCopyItem;               //Function to copy item to ring buffer
    ringbufFuncPointer pvGetItem;                //Function to get item from ring buffer
    ringbufFuncPointer vReturnItem;           //Function to return item to ring buffer
    ringbufFuncPointer xGetCurMaxSize;     //Function to get current free size

    uint8_t *pucAcquire;                        //Acquire Pointer. Points to where the next item should be acquired.
    uint8_t *pucWrite;                          //Write Pointer. Points to where the next item should be written
    uint8_t *pucRead;                           //Read Pointer. Points to where the next item should be read from
    uint8_t *pucFree;                           //Free Pointer. Points to the last item that has yet to be returned to the ring buffer
    uint8_t *pucHead;                           //Pointer to the start of the ring buffer storage area
    uint8_t *pucTail;                           //Pointer to the end of the ring buffer storage area

    BaseType_t xItemsWaiting;                   //Number of items/bytes(for byte buffers) currently in ring buffer that have not yet been read
    List_t xTasksWaitingToSend;                 //List of tasks that are blocked waiting to send/acquire onto this ring buffer. Stored in priority order.
    List_t xTasksWaitingToReceive;              //List of tasks that are blocked waiting to receive from this ring buffer. Stored in priority order.
    QueueSetHandle_t xQueueSet;                 //Ring buffer's read queue set handle.

    portMUX_TYPE mux;                           //Spinlock required for SMP
} RingBufferHack_t;

// ! END HACK

/**
 * The ISR handler for the INT1 pin
*/
void IRAM_ATTR CANDriverISRHandler(void *arg) { 
    CANDevice_t *dev = (CANDevice_t*) arg;
    RingbufHandle_t ringbuffer = driver.messageBuffer;

    eERRORRESULT ErrorExt1 = ERR_OK;

    uint8_t RxPayloadData[CAN_RX_PAYLOAD_SIZE];
    CANMessage receivedMessage;
    MCP251XFD_CANMessage tempMessage;
    tempMessage.PayloadData = &RxPayloadData[0]; 

    ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO(&dev->device, &tempMessage, CAN_RX_PAYLOAD, NULL, MCP251XFD_FIFO1);
    if (ErrorExt1 != ERR_OK)
    {   
        // ESP_LOGE(TAG, "Unable to receive message from CAN, %d", ErrorExt1);
        dev->fifoErrors++;
        return;
    }

    memcpy(receivedMessage.payload, tempMessage.PayloadData, sizeof(uint8_t) * tempMessage.DLC);
    // receivedMessage.payload = payload;
    receivedMessage.bus=0;
    receivedMessage.DLC = MCP251XFD_DLCToByte(tempMessage.DLC, tempMessage.ControlFlags & MCP251XFD_CANFD_FRAME);
    receivedMessage.ID = tempMessage.MessageID;
    receivedMessage.SEQ = tempMessage.MessageSEQ;

    if(!xRingbufferSendFromISR(ringbuffer, &receivedMessage, sizeof(CANMessage), NULL)){
        dev->receiveBufferFullErrors++;
        // free(payload);
        return;
    }
}

/**
 * Initialize the CANbus driver
*/
void CANDriverInit(){
    driver.messageBuffer = xRingbufferCreateNoSplit(sizeof(CANMessage), CAN_BUFFER_SIZE);

    // uint8_t *buffer = malloc(sizeof(uint8_t) * CAN_BUFFER_SIZE * CAN_MAX_MESSAGE_SIZE);

    // CircularBufferInit(&driver.payloadBuffer, buffer, CAN_BUFFER_SIZE * CAN_MAX_MESSAGE_SIZE);
}

/**
 * Adds a new CAN bus chip
 * @param bus The bus number of the chip, used to identify where a message came from
 * @param csPin The chip select pin 
 * @param interuptPin the INT1 pin from the chip
*/
int CANDriverAddBus(int bus, int csPin, int interruptPin, int standbyPin) {
    if(bus >= CAN_BUS_COUNT || bus < 0) return -1;

    //setup device info
    CANDevice_t *dev = &driver.devices[bus];
    dev->CSPin = csPin;
    dev->InteruptPin = interruptPin;
    dev->receiveBufferFullErrors=0;
    dev->fifoErrors=0;

    MCP251XFD mcp2517fd = DEFAULT_MCP251863_DRIVER_CONFIG(&dev->spi, csPin);
    mcp2517fd.DriverConfig |= MCP251XFD_DRIVER_INIT_SET_RAM_AT_0;
    memcpy(&dev->device, &mcp2517fd, sizeof(MCP251XFD));

    //configure MCP251XFD
    MCP251XFD_Config MCP251863_CAN_conf = DEFAULT_MCP251863_CONTROLLER_CONFIG(&dev->MCP251863_SYSCLK1, &dev->MCP251863_BTStats);
    MCP251863_CAN_conf.SysInterruptFlags = (MCP251XFD_INT_RX_EVENT | MCP251XFD_INT_RX_OVERFLOW_EVENT | MCP251XFD_INT_CLEARABLE_FLAGS_MASK);

    MCP251XFD_FIFO MCP251863_CAN_FIFO_Conf[CAN_FIFO_COUNT] = {
        { 
            .Name = MCP251XFD_FIFO1, 
            .Size = MCP251XFD_FIFO_16_MESSAGE_DEEP, 
            .Payload = CAN_RX_PAYLOAD,
            .Direction = MCP251XFD_RECEIVE_FIFO, 
            .ControlFlags = MCP251XFD_FIFO_NO_TIMESTAMP_ON_RX,
            .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT | MCP251XFD_FIFO_RECEIVE_FIFO_HALF_FULL_INT | MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
            .RAMInfos = NULL, 
        }, // SID: 0x000..0x1FF ; No EID
        { 
            .Name = MCP251XFD_FIFO2, 
            .Size = MCP251XFD_FIFO_4_MESSAGE_DEEP, 
            .Payload = MCP251XFD_PAYLOAD_8BYTE,
            .Direction = MCP251XFD_TRANSMIT_FIFO, 
            .Attempts = MCP251XFD_THREE_ATTEMPTS,
            .Priority = MCP251XFD_MESSAGE_TX_PRIORITY16, 
            .ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
            .InterruptFlags = MCP251XFD_FIFO_TX_ATTEMPTS_EXHAUSTED_INT | MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT,
            .RAMInfos = NULL, 
        },
    };

    eERRORRESULT ret;
    ret = MCP251863DeviceSetup(&dev->device, &MCP251863_CAN_conf, MCP251863_CAN_FIFO_Conf, CAN_FIFO_COUNT);

    // configure the interupt pin
    gpio_config_t pinConf = {};
    pinConf.mode = GPIO_MODE_INPUT;
    pinConf.intr_type = GPIO_INTR_LOW_LEVEL;
    pinConf.pull_down_en = 0;
    pinConf.pull_up_en = 0;
    pinConf.pin_bit_mask = (1ULL << interruptPin);
    gpio_config(&pinConf);

    //Configure standby pin
    pinConf.mode = GPIO_MODE_OUTPUT;
    pinConf.intr_type = GPIO_INTR_DISABLE;
    pinConf.pin_bit_mask = (1ULL << standbyPin);
    gpio_config(&pinConf);

    gpio_set_level(standbyPin, 0);

    gpio_isr_handler_add(interruptPin, CANDriverISRHandler, (void*) dev);

    return ret; 
}



/**
 * Gets the number of messages in the queue
*/
uint32_t CANDriverGetQueueLen(){
    unsigned int count;
    vRingbufferGetInfo(driver.messageBuffer, NULL,NULL,NULL,NULL, &count);
    // printf("free: %u, read: %u, write: %u, acquire: %u, count, %u\n", freebytes, readbytes, writebytes, acquirebytes, count);
    return count;
}

void printCANMessage(CANMessage *msg){
    printf(LOG_COLOR(LOG_COLOR_PURPLE)"Message #%"PRIu32" ID: 0x%08" PRIx32 " with %d bytes of data: ", msg->SEQ, msg->ID, msg->DLC);
    for(int i=0; i<msg->DLC; i++){
        printf("0x%X ", msg->payload[i]);
    }
    printf("\n"LOG_RESET_COLOR);
}

void CANDriverUpdate(){
    static int receiveCount = 0;
    static uint64_t lastUpdate = 0;
    CANDevice_t *dev = &driver.devices[0];

    //! TEMP Debug printing periodic message statistics
    if(esp_timer_get_time() - lastUpdate > 1000000){
        lastUpdate = esp_timer_get_time();
        ESP_LOGI(TAG, "%sReceived %d messages with %lu FIFO Errors and %lu Buffer Errors", dev->fifoErrors | dev->receiveBufferFullErrors ? LOG_COLOR(LOG_COLOR_RED) : "",
            receiveCount, dev->fifoErrors, dev->receiveBufferFullErrors);
        receiveCount = 0;
        dev->fifoErrors = 0;
        dev->receiveBufferFullErrors = 0;
    }
     
    // If no message return immediately 
    // if(!CANDriverGetQueueLen()) return;

    size_t itemSize;
    CANMessage *receivedMessage;
    int count = 0;
    while((receivedMessage = (CANMessage*)xRingbufferReceive(driver.messageBuffer, &itemSize, 0)) != NULL && count < 5){
        receiveCount++;
        count++;
        esp_event_post(CANBUS_EVENT, CAN_MESSAGE_RECIEVED, receivedMessage, sizeof(CANMessage), 2);
        vRingbufferReturnItem(driver.messageBuffer, (void*) receivedMessage);
    }

    // Fix the ring buffer if it gets stuck. I should not have to do this, but here we are. 
    // The ESP-IDF is the dumbest tool chain I have seen. The ring buffer will sometimes fail
    // to clear the rbBUFFER_FULL_FLAG resulting in the buffer permanently dropping items despite
    // being empty, so we fix that here. Dumb thing should just work.
    RingBufferHack_t *ringbuffer = (RingBufferHack_t *)driver.messageBuffer;
    if(ringbuffer->xItemsWaiting == 0 && (ringbuffer->uxRingbufferFlags & 0x4)){
        ESP_LOGW(TAG, "Buffer dumb, attempting fix!");
        ringbuffer->uxRingbufferFlags &= ~0x4;
    }

}

void sendMessage(spi_device_handle_t spi, uint8_t data){
    ESP_LOGD("SPI3", "Preparing to send byte");
    spi_device_acquire_bus(spi, portMAX_DELAY);
    spi_transaction_t t;
    memset(&t, 0 , sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    t.user = (void*) 1;
    ESP_LOGD("SPI3", "Transmitting");
    assert(spi_device_polling_transmit(spi, &t) == ESP_OK);

    spi_device_release_bus(spi);
}

// MCPError receiveMessage(MCP251XFD *dev, MCP251XFD_CANMessage *receivedMessage){
//     eERRORRESULT ErrorExt1 = ERR_OK;
//     eMCP251XFD_FIFOstatus FIFOstatus = 0;
//     ErrorExt1 = MCP251XFD_GetFIFOStatus(dev, MCP251XFD_FIFO1, &FIFOstatus); // First get FIFO1 status
//     if (ErrorExt1 != ERR_OK){
//         ESP_LOGE("CANRX","Error getting FIFO Status %d", ErrorExt1);
//         return ErrorExt1;
//     } 
//     if ((FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY) > 0) // Second check FIFO not empty
//     {
//         // uint8_t RxPayloadData[64]; // In this example, the FIFO1 have 64 bytes of payload
//         // MCP251XFD_CANMessage ReceivedMessage;
//         // receivedMessage.PayloadData = &RxPayloadData[0]; // Add receive payload data pointer to the message structure
//         // that will be received
//         ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO(dev, receivedMessage, MCP251XFD_PAYLOAD_8BYTE,
//         NULL, MCP251XFD_FIFO1);
//         if (ErrorExt1 == ERR_OK)
//         {   
//             return ERR_OK;
//             // printCANMessage(&receivedMessage);
//             // ESP_LOGI("CANRX", "%"PRIu32", Received Message #%"PRIu32" from 0x%08" PRIx32 " with %d bytes. ", MessageTimeStamp, ReceivedMessage.MessageSEQ, ReceivedMessage.MessageID, ReceivedMessage.DLC);
//         }
//     }else if(FIFOstatus != 0){
//         ESP_LOGI("CANRX","FIFO Status %d", FIFOstatus);
//     }

//     return ERR__EMPTY_DATA;
// }

