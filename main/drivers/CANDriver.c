

#include "CANDriver.h"

#include <string.h>
#include <esp_log.h>

#define CAN_STACK_SIZE  2048 //Minimum


static const char *TAG = "CANDriver";

/**
 * The ISR handler for the INT1 pin of device triggered on rising edge
*/
void IRAM_ATTR CANDriverISRHandler(void *arg) { 
    uint32_t dev = (uint32_t) arg;

    // set the corrsponding event to chip with data (this basically defers 
    // the SPI reading to a later time that is not in an interrupt). 
    xEventGroupSetBitsFromISR(driver.messageEvents, 1 << dev, pdTRUE);
}

/**
 * The FreeRTOS task for reading 
*/
static void CANDriverTask(void * arg){
    EventBits_t activeEvents = 0;

    //wait until the mask has atleast on bit set
    while(!driver.deviceEventMask)
        vTaskDelay(1);

    for (;;)
    {
        // determine which bus has just got data
        if(activeEvents){
             //get any additional events while there are still messages in the FIFO
            activeEvents |= xEventGroupClearBits(driver.messageEvents, driver.deviceEventMask);
        }else{
           //get any evens, wait until an even happens or 10ms without event
            activeEvents |= xEventGroupWaitBits(driver.messageEvents, driver.deviceEventMask, true, false, portMAX_DELAY);
        }

        //read messages from the CAN buses that currently have messages
        for(int i = 0; activeEvents >> i && i<=(0x1<<CAN_BUS_COUNT); i++){
            //skip chips without any messages
            if(((activeEvents >> i) & 0x1) == 0) {
                continue;
            }
            CANDevice_t *dev = &driver.devices[i];
            eERRORRESULT ErrorExt1 = ERR_OK;
            
            //* Device Status Checking
            //get FIFO status to determine if there is still a message
            eMCP251XFD_FIFOstatus FIFOstatus = 0;
            ErrorExt1 = MCP251XFD_GetFIFOStatus(&dev->mcp251863, MCP251XFD_FIFO1, &FIFOstatus); // First get FIFO1 status
            if (ErrorExt1 != ERR_OK){
                ESP_LOGE("CANRX","Error getting FIFO Status %d", ErrorExt1);
                xSemaphoreTake(dev->statsMutex, portMAX_DELAY);
                dev->stats.communicationErrorCount++;
                xSemaphoreGive(dev->statsMutex);
                continue;
            } 

            //Stop processing messages from a chip if there are no messages
            if(!(FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY)){
                activeEvents &= ~(0x1 << i);
                continue;
            }

            //* Message Processing
            //There is atleast one message in the FIFO so read it
            // uint8_t RxPayloadData[CAN_RX_PAYLOAD_SIZE];
            CANMessage_t receivedMessage;
            MCP251XFD_CANMessage tempMessage;
            //set the payload of the MCP251XFD struct to our CANMessage struct to avoid a memcpy
            tempMessage.PayloadData = &receivedMessage.payload[0]; 

            // Receive the message from the FIFO
            ErrorExt1 = MCP251XFD_ReceiveMessageFromFIFO(&dev->mcp251863, &tempMessage, CAN_RX_PAYLOAD, NULL, MCP251XFD_FIFO1);
            if (ErrorExt1 != ERR_OK)
            {   
                xSemaphoreTake(dev->statsMutex, portMAX_DELAY);
                dev->stats.fifoErrorCount++;
                xSemaphoreGive(dev->statsMutex);
                continue;
            }

            receivedMessage.bus = i;
            receivedMessage.DLC = MCP251XFD_DLCToByte(tempMessage.DLC, tempMessage.ControlFlags & MCP251XFD_CANFD_FRAME);
            receivedMessage.ID = tempMessage.MessageID;
            receivedMessage.SEQ = tempMessage.MessageSEQ;
            receivedMessage.timestamp = esp_timer_get_time();
            
            //? Is there a slim possibility of deadlock? Possibly need to be very careful when stats and messages are read.
            xSemaphoreTake(dev->statsMutex, portMAX_DELAY);
            if(!xQueueSend(driver.messageBuffer, &receivedMessage, 1)) {
                
                dev->stats.receiveBufferFullCount++;
                xSemaphoreGive(dev->statsMutex);
                continue;
            }
            dev->stats.messageReceiveCount++;
            xSemaphoreGive(dev->statsMutex);
        }
    }
}

void CANDriverInit(EventGroupHandle_t globalEvents, uint16_t eventFlag){
    //setup message queue
    driver.messageBuffer = xQueueCreate(CAN_BUFFER_SIZE, sizeof(CANMessage_t));
    driver.messageEvents = xEventGroupCreate();
    driver.globalEvents = globalEvents;
    driver.eventFlag = eventFlag;

    CANDevice_t *dev;
    for(int i = 0; i < CAN_BUS_COUNT; i++){
        dev = &driver.devices[i];
        dev->statsMutex = xSemaphoreCreateMutex();
    }

    //Create driver task on core 1
    xTaskCreatePinnedToCore(CANDriverTask, "CANDriver", CAN_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY, &driver.driverTaskHandler, 1);
}

int CANDriverAddBus(uint8_t bus, int csPin, int interruptPin, int standbyPin) {
    if(bus >= CAN_BUS_COUNT) return -1;

    //setup device info
    CANDevice_t *dev = &driver.devices[bus];
    dev->CSPin = csPin;
    dev->InteruptPin = interruptPin;
    memset(&dev->stats, 0, sizeof(CANDeviceStatistic_t));

    MCP251XFD mcp2517fd = DEFAULT_MCP251863_DRIVER_CONFIG(&dev->spi, csPin);
    mcp2517fd.DriverConfig |= MCP251XFD_DRIVER_INIT_SET_RAM_AT_0;
    memcpy(&dev->mcp251863, &mcp2517fd, sizeof(MCP251XFD));

    //configure MCP251XFD
    MCP251XFD_Config MCP251863_CAN_conf = DEFAULT_MCP251863_CONTROLLER_CONFIG(&dev->MCP251863_SYSCLK1, &dev->MCP251863_BTStats);
    MCP251863_CAN_conf.SysInterruptFlags = (MCP251XFD_INT_RX_EVENT | MCP251XFD_INT_RX_OVERFLOW_EVENT | MCP251XFD_INT_CLEARABLE_FLAGS_MASK);
    
    //Setup FIFO1 for receiving and FIFO2 fro transmitting
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
    ret = MCP251863DeviceSetup(&dev->mcp251863, &MCP251863_CAN_conf, MCP251863_CAN_FIFO_Conf, CAN_FIFO_COUNT);

    // configure the interupt pin
    gpio_config_t pinConf = {};
    pinConf.mode = GPIO_MODE_INPUT;
    pinConf.intr_type = GPIO_INTR_NEGEDGE;
    pinConf.pull_down_en = 0;
    pinConf.pull_up_en = 0;
    pinConf.pin_bit_mask = (1ULL << interruptPin);
    gpio_config(&pinConf);

     //Configure standby pin
    if(standbyPin >= 0){
        pinConf.mode = GPIO_MODE_OUTPUT;
        pinConf.intr_type = GPIO_INTR_DISABLE;
        pinConf.pin_bit_mask = (1ULL << standbyPin);
        gpio_config(&pinConf);
        gpio_set_level(standbyPin, 0); // pull low
    }

    gpio_isr_handler_add(interruptPin, CANDriverISRHandler, bus);
    gpio_intr_enable(interruptPin);

    driver.deviceEventMask |= (0x1 << bus);

    return ret; 
}

void CANDriverUpdate(){
    static uint64_t lastUpdate = 0;
    CANDevice_t *dev = &driver.devices[0];

    //! TEMP Debug printing periodic message statistics
    if(esp_timer_get_time() - lastUpdate > 1000000){
        lastUpdate = esp_timer_get_time();
        CANDeviceStatistic_t *stats = &dev->stats;
        ESP_LOGI(TAG, "%sReceived %lu messages with %lu FIFO Errors and %lu Buffer Errors", stats->fifoErrorCount | stats->receiveBufferFullCount ? LOG_COLOR(LOG_COLOR_RED) : "",
            stats->messageReceiveCount, stats->fifoErrorCount, stats->receiveBufferFullCount);
        ESP_LOGD(TAG, "Unused Stack: %lu", uxTaskGetStackHighWaterMark2(driver.driverTaskHandler));
        stats->messageReceiveCount = 0;
        stats->fifoErrorCount = 0;
        stats->receiveBufferFullCount = 0;
    }
     
    // If no message return immediately 
    // if(!CANDriverGetQueueLen()) return;

    size_t itemSize;
    CANMessage_t receivedMessage;
    int count = 0;
    while(xQueueReceive(driver.messageBuffer, &receivedMessage, pdMS_TO_TICKS(5)) == pdPASS && count < 5){
        count++;
        // esp_event_post(CANBUS_EVENT, CAN_MESSAGE_RECIEVED, &receivedMessage, sizeof(CANMessage_t), 2);
    }
}

void printCANMessage(CANMessage_t *msg){
    printf(LOG_COLOR(LOG_COLOR_PURPLE)"Message on bus #%u ID: 0x%08" PRIx32 " with %d bytes of data: ", msg->bus, msg->ID, msg->DLC);
    for(int i=0; i<msg->DLC; i++){
        printf("0x%X ", msg->payload[i]);
    }
    printf("\n"LOG_RESET_COLOR);
}

//unused
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