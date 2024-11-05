#include "CANDriver.h"

#include <esp_log.h>
#include <string.h>

#define CAN_STACK_SIZE 2048 // Minimum

static const char *TAG = "CANDriver";

static CANDriver_t driver;

/**
 * The ISR handler for the INT1 pin of device triggered on rising edge
 * @param arg The can bus number
 */
void IRAM_ATTR CANDriverISRHandler(void *arg) {
    uint32_t bus = (uint32_t)arg;

    // set the corrsponding event to chip with data (this basically defers
    // the SPI reading to a later time that is not in an interrupt).
    xEventGroupSetBitsFromISR(driver.messageEvents, 1 << bus, pdTRUE);
}

/**
 * The FreeRTOS task for reading
 * @param arg UNUSED (part to FreeRTOS spec)
 */
static void CANDriverTask(void *arg) {
    EventBits_t activeEvents = 0;

    // wait until the mask has at least on bit set
    while (!driver.deviceEventMask)
        vTaskDelay(1);

    for (;;) {
        // determine which bus has just got data
        if (activeEvents) {
            // get any additional events while there are still messages in the FIFO
            activeEvents |= xEventGroupClearBits(driver.messageEvents, driver.deviceEventMask);
        } else {
            // get any events, wait until an even happens or 10ms without event
            activeEvents |=
                xEventGroupWaitBits(driver.messageEvents, driver.deviceEventMask, true, false, portMAX_DELAY);
        }

        // read a message from each CAN bus that currently have messages
        for (int i = 0; activeEvents >> i && i < CAN_BUS_COUNT; i++) {
            // skip chips without any messages
            if (((activeEvents >> i) & 0x1) == 0) {
                continue;
            }

            CANDevice_t *dev = &driver.devices[i];
            eERRORRESULT ErrorExt1 = ERR_OK;

            //* Device Status Checking
            // get FIFO status to determine if there is still a message
            eMCP251XFD_FIFOstatus FIFOstatus = 0;
            ErrorExt1 =
                MCP251XFD_GetFIFOStatus(&dev->mcp251863, MCP251XFD_FIFO1, &FIFOstatus); // First get FIFO1 status
            if (ErrorExt1 != ERR_OK) {
                ESP_LOGE(TAG, "Error getting FIFO Status %d", ErrorExt1);
                xSemaphoreTake(dev->statsMutex, portMAX_DELAY);
                dev->stats.communicationErrorCount++;
                xSemaphoreGive(dev->statsMutex);
                continue;
            }

            // Stop processing messages from a chip if there are no messages
            if (!(FIFOstatus & MCP251XFD_RX_FIFO_NOT_EMPTY)) {
                activeEvents &= ~(0x1 << i);
                continue;
            }

            //* Message Processing
            // There is atleast one message in the FIFO so read it
            // uint8_t RxPayloadData[CAN_RX_PAYLOAD_SIZE];
            CANMessage_t receivedMessage;
            MCP251XFD_CANMessage tempMessage;
            // set the payload of the MCP251XFD struct to our CANMessage struct to avoid a memcpy
            tempMessage.PayloadData = &receivedMessage.payload[0];

            // Receive the message from the FIFO
            ErrorExt1 =
                MCP251XFD_ReceiveMessageFromFIFO(&dev->mcp251863, &tempMessage, CAN_RX_PAYLOAD, NULL, MCP251XFD_FIFO1);
            if (ErrorExt1 != ERR_OK) {
                xSemaphoreTake(dev->statsMutex, portMAX_DELAY);
                dev->stats.communicationErrorCount++;
                xSemaphoreGive(dev->statsMutex);
                continue;
            }

            receivedMessage.bus = i;
            receivedMessage.DLC =
                MCP251XFD_DLCToByte(tempMessage.DLC, tempMessage.ControlFlags & MCP251XFD_CANFD_FRAME);
            receivedMessage.ID = tempMessage.MessageID;
            receivedMessage.SEQ = tempMessage.MessageSEQ;
            receivedMessage.timestamp = esp_timer_get_time();

            //? Is there a slim possibility of deadlock? Possibly need to be very careful with taking the 
            //? Semaphore elsewhere then waiting to receive a message
            xSemaphoreTake(dev->statsMutex, portMAX_DELAY);
            if (FIFOstatus & MCP251XFD_RX_FIFO_OVERFLOW) {
                dev->stats.fifoErrorCount++;
                MCP251863ClearFIFOOverflowFlag(&dev->mcp251863, MCP251XFD_FIFO1);
            }

            if (!xQueueSend(driver.messageBuffer, &receivedMessage, pdMS_TO_TICKS(1))) {

                dev->stats.receiveBufferFullCount++;
                xSemaphoreGive(dev->statsMutex);
                continue;
            }
            dev->stats.messageReceiveCount++;
            xSemaphoreGive(dev->statsMutex);

            xEventGroupSetBits(driver.globalEvents, driver.messageFlag);
        }
    }
}

/**
 * Driver initialization.
 * @param globalEvents EventGroupHandle for main GUB component for notifying CAN events
 * @param eventFlag the flags to set on a new message
 */
void setupCANDriver(EventGroupHandle_t globalEvents, uint16_t messageFlag) {
    // setup message queue
    driver.messageBuffer = xQueueCreate(CAN_BUFFER_SIZE, sizeof(CANMessage_t));
    driver.messageEvents = xEventGroupCreate();
    driver.globalEvents = globalEvents;
    driver.messageFlag = messageFlag;

    // Initialize device structs
    CANDevice_t *dev;
    for (int i = 0; i < CAN_BUS_COUNT; i++) {
        dev = &driver.devices[i];
        dev->statsMutex = xSemaphoreCreateMutex();
    }

    // Create driver task on core 1
    xTaskCreatePinnedToCore(CANDriverTask, "CANDriver", CAN_STACK_SIZE, NULL, DEFAULT_TASK_PRIORITY,
                            &driver.driverTaskHandler, 1);
}

/**
 * Add a new CAN bus chip to the driver
 * @param bus the bus number, must be unique to each chip and at max CAN_BUS_COUNT
 * @param csPin The pin nCS of the MCP251863
 * @param interruptPin The nINT1 pin of the MCP251863
 * @param standbyPin The STBY pin of the MCP251863 (optional, set -1 to ignore)
 */
int addCANBus(uint8_t bus, int csPin, int interruptPin, int standbyPin) {
    if (bus >= CAN_BUS_COUNT)
        return -1;

    // setup device info
    CANDevice_t *dev = &driver.devices[bus];
    dev->CSPin = csPin;
    dev->InteruptPin = interruptPin;
    memset(&dev->stats, 0, sizeof(CANDeviceStatistic_t));

    MCP251XFD mcp2517fd = DEFAULT_MCP251863_DRIVER_CONFIG(&dev->spi, csPin);
    mcp2517fd.DriverConfig |= MCP251XFD_DRIVER_INIT_SET_RAM_AT_0;
    memcpy(&dev->mcp251863, &mcp2517fd, sizeof(MCP251XFD));

    // configure MCP251XFD
    MCP251XFD_Config MCP251863_CAN_conf =
        DEFAULT_MCP251863_CONTROLLER_CONFIG(&dev->MCP251863_SYSCLK1, &dev->MCP251863_BTStats);
    MCP251863_CAN_conf.SysInterruptFlags =
        (MCP251XFD_INT_RX_EVENT | MCP251XFD_INT_RX_OVERFLOW_EVENT | MCP251XFD_INT_CLEARABLE_FLAGS_MASK);

    // Setup FIFO1 for receiving and FIFO2 for transmitting
    MCP251XFD_FIFO MCP251863_CAN_FIFO_Conf[CAN_FIFO_COUNT] = {
        {
            .Name = MCP251XFD_FIFO1,
            .Size = MCP251XFD_FIFO_32_MESSAGE_DEEP,
            .Payload = CAN_RX_PAYLOAD,
            .Direction = MCP251XFD_RECEIVE_FIFO,
            .ControlFlags = MCP251XFD_FIFO_NO_TIMESTAMP_ON_RX,
            .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT | MCP251XFD_FIFO_RECEIVE_FIFO_HALF_FULL_INT |
                              MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
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

    // configure the interrupt pin
    gpio_config_t pinConf = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .pin_bit_mask = (1ULL << interruptPin),
    };
    gpio_config(&pinConf);

    // Configure standby pin
    if (standbyPin >= 0) {
        pinConf.mode = GPIO_MODE_OUTPUT;
        pinConf.intr_type = GPIO_INTR_DISABLE;
        pinConf.pin_bit_mask = (1ULL << standbyPin);
        gpio_config(&pinConf);
        gpio_set_level(standbyPin, 0); // pull low
    }

    // Set interrupt handler for the pin. Done before setup to ensure a falling edge is not missed
    gpio_isr_handler_add(interruptPin, CANDriverISRHandler, (void *)bus);
    gpio_intr_enable(interruptPin);

    // Configure MCP251863
    eERRORRESULT ret;
    ret = MCP251863DeviceSetup(&dev->mcp251863, &MCP251863_CAN_conf, MCP251863_CAN_FIFO_Conf, CAN_FIFO_COUNT);

    // Unmask event
    driver.deviceEventMask |= (0x1 << bus);
    xEventGroupSetBits(driver.messageEvents, 1 << bus);

    return ret;
}

/**
 * CAN driver update method for periodic tasks, may become obsolete
 */
void canDriverUpdate() {
    //! Unused!
}

/**
 * Retrieve a CAN message from the driver.
 * @param message the CAN message received from the driver
 * @param timeoutTicks the number of ticks to wait to retrieve a message.
 * @returns 0 for success or error code
 */
int receiveCANMessage(CANMessage_t *message, uint32_t timeoutTicks) {
    return xQueueReceive(driver.messageBuffer, message, timeoutTicks);
}

/**
 * Get the message statistics for a bus from the driver.
 * @param bus the bus to retrieve the statistics for.
 * @param clear should the stats be cleared on read
 */
CANDeviceStatistic_t getCANStatistics(int bus, bool clear) {
    CANDeviceStatistic_t busStat;
    xSemaphoreTake(driver.devices[bus].statsMutex, portMAX_DELAY);
    memcpy(&busStat, &driver.devices[bus].stats, sizeof(CANDeviceStatistic_t));
    if (clear)
        memset(&driver.devices[bus].stats, 0, sizeof(CANDeviceStatistic_t));
    xSemaphoreGive(driver.devices[bus].statsMutex);
    return busStat;
}

/**
 * Prints out a CAN message.
 * @param msg the message to print
 */
void printCANMessage(CANMessage_t const *msg) {
    printf(LOG_COLOR(LOG_COLOR_PURPLE) "Message on bus #%u ID: 0x%08" PRIx32 " with %d bytes of data: ", msg->bus,
           msg->ID, msg->DLC);
    for (int i = 0; i < msg->DLC; i++) {
        printf("0x%X ", msg->payload[i]);
    }
    printf("\n" LOG_RESET_COLOR);
}

/**
 * Debug printing of CAN Driver state
 */
void printCANDriverState() {
    printf("CAN Driver Status:\r\n\tUnused Stack %lu\r\n\tBus Status:\r\n",
           uxTaskGetStackHighWaterMark2(driver.driverTaskHandler));
           
    printf("\t| bus | MSG Cnt | FIFO ERR | BUFF ERR | COM ERR |\r\n");
    for (int i = 0; i < CAN_BUS_COUNT; i++) {
        CANDeviceStatistic_t busStats = getCANStatistics(i, true);
        printf("\t| %3d | %7lu | %s%8lu" LOG_RESET_COLOR " | %s%8lu" LOG_RESET_COLOR " | %s%7lu" LOG_RESET_COLOR
               " |\r\n",
               i, busStats.messageReceiveCount, busStats.fifoErrorCount ? LOG_BOLD(LOG_COLOR_RED) : "",
               busStats.fifoErrorCount, busStats.receiveBufferFullCount ? LOG_BOLD(LOG_COLOR_RED) : "",
               busStats.receiveBufferFullCount, busStats.communicationErrorCount ? LOG_BOLD(LOG_COLOR_RED) : "",
               busStats.communicationErrorCount);
    }
}