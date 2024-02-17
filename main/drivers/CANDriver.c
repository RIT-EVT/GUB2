#include "drivers/CANDriver.h"

#include <string.h>
#include <esp_log.h>

#include "Events.h"

static const char *TAG = "CANDriver";

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
    //TODO Replace with staticly allocated memory
    // uint8_t *payload = (uint8_t*) malloc(sizeof(uint8_t) * tempMessage.DLC);
    // if(payload == NULL){
    //     // ESP_LOGE(TAG, "Unable to allocate buffer");
    //     return;
    // }

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
int CANDriverAddBus(int bus, int csPin, int interruptPin) {
    if(bus >= CAN_BUS_COUNT || bus < 0) return -1;

    //setup device info
    CANDevice_t *dev = &driver.devices[bus];
    dev->CSPin = csPin;
    dev->InteruptPin = interruptPin;
    dev->receiveBufferFullErrors=0;
    dev->fifoErrors=0;

    MCP251XFD mcp2517fd = DEFAULT_MCP251863_DRIVER_CONFIG(&dev->spi, PIN_NUM_CAN_CS);
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
    gpio_config_t int1Conf = {};
    int1Conf.mode = GPIO_MODE_INPUT;
    int1Conf.intr_type = GPIO_INTR_LOW_LEVEL;
    int1Conf.pull_down_en = 0;
    int1Conf.pull_up_en = 0;
    int1Conf.pin_bit_mask = (1ULL << interruptPin);

    gpio_config(&int1Conf);

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
    if(!CANDriverGetQueueLen()) return;

    size_t itemSize;
    CANMessage *receivedMessage;
    while((receivedMessage = (CANMessage*)xRingbufferReceive(driver.messageBuffer, &itemSize, pdMS_TO_TICKS(10))) != NULL && receiveCount < 5){
        receiveCount++;
        esp_event_post(CANBUS_EVENT, CAN_MESSAGE_RECIEVED, receivedMessage, sizeof(CANMessage), 200);
        vRingbufferReturnItem(driver.messageBuffer, (void*) receivedMessage);
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

