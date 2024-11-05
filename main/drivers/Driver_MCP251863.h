#ifndef DRIVER_MCP251863
#define DRIVER_MCP251863

#include <stdint.h>
#include "MCP251XFD.h"

// default initialization structs
#define DEFAULT_MCP251863_DRIVER_CONFIG(MCP251863_DEV_HANDLE, SPI_CS_PIN)     { \
  .UserDriverData = NULL,                                           \
  /*--- Driver configuration ---*/                                  \
  .DriverConfig   = MCP251XFD_DRIVER_NORMAL_USE                     \
                  | MCP251XFD_DRIVER_SAFE_RESET,                    \
  /*--- IO configuration ---*/                                      \
  .GPIOsOutLevel   = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_HIGH,    \
  /*--- Interface driver call functions ---*/                       \
  .SPI_ChipSelect  = SPI_CS_PIN,                                    \
  .InterfaceDevice = MCP251863_DEV_HANDLE,                          \
  .fnSPI_Init      = MCP251863DeviceInit,                           \
  .fnSPI_Transfer  = MCP251863DeviceTransfer,                       \
  /*--- Time call function ---*/                                    \
  .fnGetCurrentms  = MCP251863DeviceGetCurrentms,                   \
  /*--- CRC16-USB call function ---*/                               \
  .fnComputeCRC16  = MCP251863DeviceComputeCRC16,                   \
  /*--- Interface clocks ---*/                                      \
  .SPIClockSpeed   = CAN_SPI_CLOCK_SPEED,                           \
}

#define DEFAULT_MCP251863_CONTROLLER_CONFIG(SYSCLK, BTSTATS)   {                    \
    /*--- Controller clocks ---*/                                                   \
    .XtalFreq       = MCP251863_CRY_CLK,     /* CLKIN is a crystal*/                \
    .OscFreq        = 0,     /* CLKIN is an oscillator*/                            \
    .SysclkConfig   = MCP251XFD_SYSCLK_IS_CLKIN,                                    \
    .ClkoPinConfig  = MCP251XFD_CLKO_SOF,                                           \
    .SYSCLK_Result  = SYSCLK,                                                       \
    /*--- CAN configuration ---*/                                                   \
    .NominalBitrate = CAN_NOMINAL_BITRATE,          /* Nominal Bitrate*/            \
    .DataBitrate    = CAN_DATA_BITRATE,             /* Data Bitrate*/               \
    .BitTimeStats   = BTSTATS,                                                      \
    .Bandwidth      = MCP251XFD_NO_DELAY,                                           \
    .ControlFlags   = MCP251XFD_CAN_RESTRICTED_RETRANS_ATTEMPTS,   /* Restricted retransmission attempts, */                \
                                                                   /* MCP251XFD_FIFO.Attempts (CiFIFOCONm.TXAT) is used */  \
    /*--- GPIOs and Interrupts pins ---*/                                           \
    .GPIO0PinMode  = MCP251XFD_PIN_AS_GPIO0_OUT,                                    \
    .GPIO1PinMode  = MCP251XFD_PIN_AS_INT1_RX,                                      \
    .INTsOutMode   = MCP251XFD_PINS_PUSHPULL_OUT,                                   \
    .TXCANOutMode  = MCP251XFD_PINS_PUSHPULL_OUT,                                   \
    /*--- Interrupts ---*/                                                          \
    .SysInterruptFlags = MCP251XFD_INT_ENABLE_ALL_EVENTS - MCP251XFD_INT_TX_EVENT,  \
}

#define TIMESTAMP_TICK_us (25) // TimeStamp tick is 25s
#define TIMESTAMP_TICK(sysclk) (((sysclk) / 1000000) * TIMESTAMP_TICK_us)

typedef eERRORRESULT MCPError;

// helper methods
MCPError MCP251863DeviceSetup(MCP251XFD *dev, MCP251XFD_Config *conf, MCP251XFD_FIFO *fifoConf, uint8_t fifoCount);

// Library methods
eERRORRESULT MCP251863DeviceInit(void *pIntDev, uint8_t chipSelect, const uint32_t sckFreq);
eERRORRESULT MCP251863DeviceTransfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size);
uint32_t MCP251863DeviceGetCurrentms(void);
uint16_t MCP251863DeviceComputeCRC16(const uint8_t *data, size_t size);

// Additional Methods
eERRORRESULT MCP251863SetFIFOStatus(MCP251XFD *pComp, eMCP251XFD_FIFO name, setMCP251XFD_FIFOstatus statusFlags);
eERRORRESULT MCP251863ClearFIFOOverflowFlag(MCP251XFD *pComp, eMCP251XFD_FIFO name);

// Debug Methods
void ShowDeviceError(eERRORRESULT error);

#endif