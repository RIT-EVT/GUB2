/**
 * CAN pin defines
*/

// Real GUB pins
// CAN SPI
#define CAN_SPI_HOST        SPI3_HOST
#define PIN_NUM_CAN_MISO    13
#define PIN_NUM_CAN_MOSI    11
#define PIN_NUM_CAN_CLK     12

//CAN A
#define PIN_NUM_CAN_A_CS      10
#define PIN_NUM_CAN_A_RX_INT  47    // 50/50 this and MC_RX_INT need to be swapped
#define PIN_NUM_CAN_A_STB     16

// CAN MC
#define PIN_NUM_CAN_MC_CS      14
#define PIN_NUM_CAN_MC_RX_INT  48   // 50/50 this and A_RX_INT need to be swapped
#define PIN_NUM_CAN_MC_STB     4

/**
 * CAN configuration defines
*/
#define CAN_SPI_CLOCK_SPEED 5000000     /*5MHz*/

#define MCP251863_CRY_CLK   40000000    /*40MHz*/
#define CAN_NOMINAL_BITRATE 500000      /* 500 Kbps*/
#define CAN_DATA_BITRATE    1000000     /* 1   Mbps*/

#define CAN_BUS_COUNT           2 
#define CAN_BUFFER_SIZE         100      // ESP message buffer size
#define CAN_MAX_MESSAGE_SIZE    8       // max can message payload size of 8 bytes used to 
                                        // preallocate memory for the payload data

/**
 * SD card pin defines
*/
#define SD_USING_SDIO

// Real GUB pins
// SPI pins
#define SD_SPI_HOST         SPI2_HOST
#define PIN_NUM_SD_MISO     37
#define PIN_NUM_SD_MOSI     35
#define PIN_NUM_SD_CLK      36
#define PIN_NUM_SD_CS       21
#define PIN_NUM_SD_CD       SDSPI_SLOT_NO_CD

// SDIO pins
#define SD_PIN_CLK      35
#define SD_PIN_CMD      36
#define SD_PIN_D0       6
#define SD_PIN_D1       21
#define SD_PIN_D2       38
#define SD_PIN_D3       37

/**
 * SD card configuration defines
*/

#define MAX_FILE_HANDLERS               5
#define FLUSH_LOG_INTERVAL              10000 * 1000 //10 seconds
#define FLUSH_SIZE_THRESHOLD            4 << 10  //4 KiB
#define MAX_LOG_SIZE                    1 << 20  //1 MiB    
#define SD_CARD_STATUS_CHECK_INTERVAL   1000 * 1000 //1 Second

#define SD_CARD_MOUNT_PATH      "/data"
#define SD_CARD_CAN_LOG_PATH    SD_CARD_MOUNT_PATH "/CANLogs"

// File base name. Will have the format <BaseName><duplicate>_<timestamp>-<split>
#define CAN_LOG_BASE_NAME   "CANLOG"

/**
 * Generic IO pin defines
*/

// Real GUB pins
#define PIN_NUM_HEARTBEAT   2

#define HEARTBEAT_LED_PERIOD    100 * 1000 //100ms

/**
 * GUB Configuration defines
*/


/**
 * WIFI Configuration defines
*/

#define ESP_WIFI_SSID "GUB2AP" //CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS "test1234"// CONFIG_ESP_WIFI_PASSWORD
#define MAX_STA_CONN  4 //CONFIG_ESP_MAX_STA_CONN

/**
 * Task Priorites
*/
#define LOW_TASK_PRIORITY 3
#define DEFAULT_TASK_PRIORITY 5
#define HIGH_TASK_PRIORITY 7