/**
 * CAN pin defines
*/
#define CAN_SPI_HOST        SPI3_HOST
#define PIN_NUM_CAN_MISO    37
#define PIN_NUM_CAN_MOSI    35
#define PIN_NUM_CAN_CLK     36
#define PIN_NUM_CAN_CS      45
#define PIN_NUM_CAN_RX_INT  42

/**
 * CAN configuration defines
*/
#define CAN_SPI_CLOCK_SPEED 5000000     /*5MHz*/

#define MCP251863_CRY_CLK   40000000    /*40MHz*/
#define CAN_NOMINAL_BITRATE 500000      /* 500 Kbps*/
#define CAN_DATA_BITRATE    1000000     /* 1   Mbps*/

#define CAN_BUS_COUNT           1
#define CAN_BUFFER_SIZE         30
#define CAN_MAX_MESSAGE_SIZE    8      //max can message payload size of 12 bytes used to preallocate memory for the payload data

/**
 * SD card pin defines
*/
#define SD_SPI_HOST         SPI2_HOST
#define PIN_NUM_SD_MISO     9
#define PIN_NUM_SD_MOSI     10
#define PIN_NUM_SD_CLK      11
#define PIN_NUM_SD_CS       12
#define PIN_NUM_SD_CD       13

/**
 * SD card configuration defines
*/

#define MAX_FILE_HANDLERS   5

/**
 * Generic IO pin defines
*/
#define PIN_NUM_HEARTBEAT   48
#define LED_IS_NEOPIXEL     1
#define HEARTBEAT_PERIOD    500 * 1000 //500ms

/**
 * GUB Configuration defines
*/

#define NUMBER_DRIVERS      3


/**
 * WIFI Configuration defines
*/

#define ESP_WIFI_SSID "GUB2AP" //CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS "test1234"// CONFIG_ESP_WIFI_PASSWORD
#define MAX_STA_CONN  4 //CONFIG_ESP_MAX_STA_CONN
