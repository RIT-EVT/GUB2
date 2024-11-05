#include "drivers/Driver_MCP251863.h"

#include <string.h>

#include <driver/spi_master.h>
#include <esp_log.h>
#include <esp_timer.h>

//Enable library error strings if debugging
#if LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG
#define USE_ERRORS_STRING 
#endif

#include "CRC/CRC16_CMS.h"
#include "ErrorsDef.h"
#include "MCP251XFD.h"

#include "GUB_Conf.h"

/**
 * Setup a MCP251863 chip
 * @param dev pointer to the device struct
 * @param conf pointer to the device configuration struct
 * @param fifoConf pointer to the FIFO configuration array
 * @param fifoCount the number of configured FIFOs
*/
MCPError MCP251863DeviceSetup(MCP251XFD *dev, MCP251XFD_Config *conf, MCP251XFD_FIFO *fifoConf, uint8_t fifoCount){
    if (dev == NULL) return ERR__CONFIGURATION;
    if (conf == NULL) return ERR__CONFIGURATION;
    if (fifoConf == NULL) return ERR__CONFIGURATION;
    if (fifoCount == 0) return ERR__CONFIGURATION;

    eERRORRESULT ret;
    MCP251XFD_Filter MCP251863_NoFilter[1] = {
        { 
            .Filter = MCP251XFD_FILTER0, 
            .EnableFilter = true, 
            .Match = MCP251XFD_MATCH_ONLY_SID,
            .AcceptanceID = MCP251XFD_ACCEPT_ALL_MESSAGES, 
            .AcceptanceMask = MCP251XFD_ACCEPT_ALL_MESSAGES, 
            .PointTo = MCP251XFD_FIFO1, 
        }, // 0x000..0x1FF
    };
    ret = Init_MCP251XFD(dev, conf);
    if(ret != ERR_OK) return ret;
    
    ret = MCP251XFD_ConfigureTimeStamp(dev, true, MCP251XFD_TS_CAN20_SOF_CANFD_SOF, TIMESTAMP_TICK(MCP251863_CRY_CLK), false);
    if(ret != ERR_OK) return ret;
    
    ret = MCP251XFD_ConfigureFIFOList(dev, fifoConf, fifoCount);
    if(ret != ERR_OK) return ret;
    
    ret = MCP251XFD_ConfigureFilterList(dev, MCP251XFD_D_NET_FILTER_DISABLE, MCP251863_NoFilter, 1);
    if(ret != ERR_OK) return ret;
    
    ret = MCP251XFD_StartCANFD(dev);
    return ret;
}

//***********************************
//***** MCP251863 Library Hooks *****
//***********************************

eERRORRESULT MCP251863DeviceInit(void *pIntDev, uint8_t chipSelect, const uint32_t sckFreq){
    if (pIntDev == NULL) return ERR__SPI_PARAMETER_ERROR;
    spi_device_handle_t *spiDevice = (spi_device_handle_t *) pIntDev;
    esp_err_t ret;
    spi_device_interface_config_t devCFG={
        .clock_speed_hz=sckFreq,    //set clock out frequency
        .mode=0,                    //SPI mode 0
        .spics_io_num=chipSelect,   //CS pin
        .queue_size=7,              //We want to be able to queue 7 transactions at a time
        .pre_cb=NULL,               //Specify pre-transfer callback to handle D/C line
    };

    ret = spi_bus_add_device(CAN_SPI_HOST, &devCFG, spiDevice);
    ESP_ERROR_CHECK(ret);

    return ERR_OK;
}

eERRORRESULT MCP251863DeviceTransfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size){
    if (pIntDev == NULL) return ERR__SPI_PARAMETER_ERROR;
    if (txData == NULL) return ERR__SPI_PARAMETER_ERROR;

    spi_device_handle_t *spiDevice = (spi_device_handle_t *) pIntDev;
    spi_transaction_t t;
    esp_err_t ret;

    spi_device_acquire_bus(*spiDevice, portMAX_DELAY);

    memset(&t, 0 , sizeof(t));
    t.length = 8 * size;
    t.tx_buffer = txData;
    t.rx_buffer = rxData;
    t.user = (void*) 1;

    ret = spi_device_polling_transmit(*spiDevice, &t);
    spi_device_release_bus(*spiDevice);

    return ret == ESP_OK ? ERR_OK : ERR__SPI_PARAMETER_ERROR;
}

uint32_t MCP251863DeviceGetCurrentms(void){
    return esp_timer_get_time()/1000;
}

uint16_t MCP251863DeviceComputeCRC16(const uint8_t* data, size_t size){
    return ComputeCRC16CMS(data, size);
}

//****************************************
//***** MCP251863 Additional Methods *****
//****************************************

/**
 * Set status of a FIFO
 * @param pComp the device struct
 * @param name the FIFO to set the state of
 * @param statusFlags the status flags to write
*/
eERRORRESULT MCP251863SetFIFOStatus(MCP251XFD *pComp, eMCP251XFD_FIFO name, setMCP251XFD_FIFOstatus statusFlags){
  if (name >= MCP251XFD_FIFO_COUNT) return ERR__PARAMETER_ERROR;

  //--- Set address of the FIFO ---
  uint16_t Address = RegMCP251XFD_CiFIFOSTAm_FLAGS + (MCP251XFD_FIFO_REG_SIZE * ((uint16_t)name - 1u)); // Select the address of the FIFO
  if (name == MCP251XFD_TEF) Address = RegMCP251XFD_CiTEFSTA_FLAGS;                                     // If it's the TEF FIFO then select its address
  if (name == MCP251XFD_TXQ) Address = RegMCP251XFD_CiTXQSTA_FLAGS;                                     // If it's the TXQ FIFO then select its address

  //--- Set FIFO status ---
  return MCP251XFD_WriteSFR8(pComp, Address, (uint8_t)statusFlags);                                     // Write FIFO status (First byte only)
}

/**
 * Clears a FIFO's Overflow flag that needs to be cleared manually. 
 * @param pComp the device struct
 * @param name the FIFO to clear the overflow flag on
*/
eERRORRESULT MCP251863ClearFIFOOverflowFlag(MCP251XFD *pComp, eMCP251XFD_FIFO name){
    eERRORRESULT error;
    setMCP251XFD_FIFOstatus statusFlags;
    error = MCP251XFD_GetFIFOStatus(pComp, name, &statusFlags);
    if (error != ERR_OK) return error;
    
    return MCP251863SetFIFOStatus(pComp, name, statusFlags & ~(MCP251XFD_RX_FIFO_OVERFLOW));
}

//***********************************
//*****     MCP251863 Debug     *****
//***********************************

/**
 * Prints out the friendly name for MCP251863 device errors
 * @param error the error to print out
*/
void ShowDeviceError(eERRORRESULT error) {
  char* pStr = NULL;
  switch (error)
  {
      #define X(a, b, c) case a: pStr = (char*)c; break;
      ERRORS_TABLE
      #undef X
      default:
          pStr = NULL; 
          break;
  }

  if (pStr != NULL)
       ESP_LOGE("MCP_INFO","Device error: %s", pStr);
  else ESP_LOGE("MCP_INFO","Device error: Unknown error (%u)", (unsigned int)error);
}