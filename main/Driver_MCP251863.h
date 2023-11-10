#ifndef DRIVER_MCP251863
#define DRIVER_MCP251863

#include <stdint.h>
#include "MCP251XFD.h"



#define DEFAULT_MCP251863_DRIVER_CONFIG(MCP251863_DEV_HANDLE)     { \
  .UserDriverData = NULL,                                           \
  /*--- Driver configuration ---*/                                  \
  .DriverConfig   = MCP251XFD_DRIVER_NORMAL_USE                     \
                  | MCP251XFD_DRIVER_SAFE_RESET,                    \
  /*--- IO configuration ---*/                                      \
  .GPIOsOutState   = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_HIGH,    \
  /*--- Interface driver call functions ---*/                       \
  .SPI_ChipSelect  = PIN_NUM_CAN_CS,                                \
  .InterfaceDevice = MCP251863_DEV_HANDLE,                          \
  .fnSPI_Init      = MCP251863DeviceInit,                           \
  .fnSPI_Transfer  = MCP251863DeviceTransfer,                       \
  /*--- Time call function ---*/                                    \
  .fnGetCurrentms  = MCP251863DeviceGetCurrentms,                   \
  /*--- CRC16-USB call function ---*/                               \
  .fnComputeCRC16  = MCP251863DeviceComputeCRC16,                   \
  /*--- Interface clocks ---*/                                      \
  .SPIClockSpeed   = 5000000, /*5MHz*/                            \
};

#define DEFAULT_MCP251863_CONTROLLER_CONFIG(SYSCLK, BTSTATS)   {                    \
    /*--- Controller clocks ---*/                                                   \
    .XtalFreq       = MCP251863_CRYCLK,     /* CLKIN is not a crystal*/             \
    .OscFreq        = 0,     /* CLKIN is an oscillator*/                            \
    .SysclkConfig   = MCP251XFD_SYSCLK_IS_CLKIN,                                    \
    .ClkoPinConfig  = MCP251XFD_CLKO_SOF,                                           \
    .SYSCLK_Result  = SYSCLK,                                           \
    /*--- CAN configuration ---*/                                                   \
    .NominalBitrate = 500000,                   /* Nominal Bitrate to 0.5Mbps*/     \
    .DataBitrate    = 1000000,                   /* Data Bitrate to 1Mbps*/         \
    .BitTimeStats   = BTSTATS,                                           \
    .Bandwidth      = MCP251XFD_NO_DELAY,                                           \
    .ControlFlags   = MCP251XFD_CAN_RESTRICTED_RETRANS_ATTEMPTS,   /* Restricted retransmission attempts, MCP251XFD_FIFO.Attempts (CiFIFOCONm.TXAT) is used*/   \
    /*--- GPIOs and Interrupts pins ---*/                                               \
    .GPIO0PinMode  = MCP251XFD_PIN_AS_GPIO0_OUT,                                        \
    .GPIO1PinMode  = MCP251XFD_PIN_AS_INT1_RX,                                          \
    .INTsOutMode   = MCP251XFD_PINS_PUSHPULL_OUT,                                       \
    .TXCANOutMode  = MCP251XFD_PINS_PUSHPULL_OUT,                                       \
    /*--- Interrupts ---*/                                                              \
    .SysInterruptFlags = MCP251XFD_INT_ENABLE_ALL_EVENTS - MCP251XFD_INT_TX_EVENT,      \
};

#define TIMESTAMP_TICK_us       ( 25 )      // TimeStamp tick is 25s
#define TIMESTAMP_TICK(sysclk)  ( ((sysclk) / 1000000) * TIMESTAMP_TICK_us )

typedef eERRORRESULT MCPError;

eERRORRESULT MCP251863DeviceInit(void *pIntDev, uint8_t chipSelect, const uint32_t sckFreq);

eERRORRESULT MCP251863DeviceTransfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size);

uint32_t MCP251863DeviceGetCurrentms(void);

uint16_t MCP251863DeviceComputeCRC16(const uint8_t* data, size_t size);

void GetAndShowMCP251XFD_SFRreg(MCP251XFD *pComp);
void GetAndShowMCP251XFD_CANSFRreg(MCP251XFD *pComp);
void GetAndShowMCP251XFD_FIFOreg(MCP251XFD *pComp);
void GetAndShowMCP251XFD_FILTERreg(MCP251XFD *pComp);

void ShowDeviceError(MCP251XFD *pComp, eERRORRESULT error);

#endif