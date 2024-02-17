#include "drivers/Driver_MCP251863.h"

#include <string.h>

#include <esp_timer.h>
#include <driver/spi_master.h>
#include <esp_log.h>

#include "MCP251XFD.h"
#include "CRC/CRC16_CMS.h"
#define USE_ERRORS_STRING
#include "ErrorsDef.h"

#include "GUB_Conf.h"

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
    // ESP_LOGI("MCP INIT","Initialization result: %d", ret);
    ret = MCP251XFD_ConfigureTimeStamp(dev, true, MCP251XFD_TS_CAN20_SOF_CANFD_SOF, TIMESTAMP_TICK(MCP251863_CRY_CLK), false);
    if(ret != ERR_OK) return ret;
    // ESP_LOGI("MCP INIT","Timestamp setup result: %d", ret);
    ret = MCP251XFD_ConfigureFIFOList(dev, fifoConf, fifoCount);
    if(ret != ERR_OK) return ret;
    //ESP_LOGI("MCP INIT","FIFO Setup result: %d", ret);
    ret = MCP251XFD_ConfigureFilterList(dev, MCP251XFD_D_NET_FILTER_DISABLE, MCP251863_NoFilter, 1);
    if(ret != ERR_OK) return ret;
    //ESP_LOGI("MCP INIT","Filter Setup result: %d", ret);
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
        .clock_speed_hz=sckFreq,                //Clock out at 1 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=chipSelect,           //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=NULL,  //Specify pre-transfer callback to handle D/C line
    };

    ret = spi_bus_add_device(CAN_SPI_HOST, &devCFG, spiDevice);
    ESP_ERROR_CHECK(ret);

    return ERR_OK;
}

eERRORRESULT MCP251863DeviceTransfer(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size){
    if (pIntDev == NULL) return ERR__SPI_PARAMETER_ERROR;
    if (txData == NULL) return ERR__SPI_PARAMETER_ERROR;

    // ESP_LOGD("MCP251863", "Transfering %d bytes of data; First byte: %x", size, *txData);

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


//***********************************
//*****     MCP251863 Debug     *****
//***********************************

// typedef union
// {
//   unsigned int Uint32[MCP251XFD_CONTROLLER_SFR_SIZE / sizeof(unsigned int)];
//   uint8_t      Bytes[MCP251XFD_CONTROLLER_SFR_SIZE / sizeof(uint8_t)];
// } ControllerSFRData;


#define LOGINFO(format, ... ) ESP_LOGI("MCP_INFO", format, ##__VA_ARGS__)
#define LOGERROR(format, ... ) ESP_LOGE("MCP_INFO", format, ##__VA_ARGS__)

// void GetAndShowMCP251XFD_SFRreg(MCP251XFD *pComp)
// {
//   ControllerSFRData CSD;
//   eERRORRESULT Error = MCP251XFD_ReadData(pComp, MCP251XFD_CONTROLLER_SFR_ADDR, &CSD.Bytes[0], MCP251XFD_CONTROLLER_SFR_SIZE);
//   if (Error == ERR_OK)
//   {
//     LOGINFO("MCP251XFD Special Function Registers:");
//     LOGINFO("   OSC     = 0x%08X", CSD.Uint32[0]);
//     LOGINFO("   IOCON   = 0x%08X", CSD.Uint32[1]);
//     LOGINFO("   CRC     = 0x%08X", CSD.Uint32[2]);
//     LOGINFO("   ECCCON  = 0x%08X", CSD.Uint32[3]);
//     LOGINFO("   ECCSTAT = 0x%08X", CSD.Uint32[4]);
//     LOGINFO("   DEVID   = 0x%08X", CSD.Uint32[5]);
//   }
//   else ShowDeviceError(pComp, Error);
// }



// #define CAN_SFR_SIZE  ( 16 * sizeof(unsigned int) )

// typedef union
// {
//   unsigned int Uint32[CAN_SFR_SIZE / sizeof(unsigned int)];
//   uint8_t      Bytes[CAN_SFR_SIZE / sizeof(uint8_t)];
// } CANControllerSFRData;

// void GetAndShowMCP251XFD_CANSFRreg(MCP251XFD *pComp)
// {
//   CANControllerSFRData CCSD;
//   eERRORRESULT Error = MCP251XFD_ReadData(pComp, MCP251XFD_CAN_CONTROLLER_ADDR, &CCSD.Bytes[0], CAN_SFR_SIZE);
//   if (Error == ERR_OK)
//   {
//     LOGINFO(" CAN Controller Special Function Registers:");
//     LOGINFO("   C1CON    = 0x%08X", CCSD.Uint32[ 0]);
//     LOGINFO("   C1NBTCFG = 0x%08X", CCSD.Uint32[ 1]);
//     LOGINFO("   C1DBTCFG = 0x%08X", CCSD.Uint32[ 2]);
//     LOGINFO("   C1TDC    = 0x%08X", CCSD.Uint32[ 3]);
//     LOGINFO("   C1TBC    = 0x%08X", CCSD.Uint32[ 4]);
//     LOGINFO("   C1TSCON  = 0x%08X", CCSD.Uint32[ 5]);
//     LOGINFO("   C1VEC    = 0x%08X", CCSD.Uint32[ 6]);
//     LOGINFO("   C1INT    = 0x%08X", CCSD.Uint32[ 7]);
//     LOGINFO("   C1RXIF   = 0x%08X", CCSD.Uint32[ 8]);
//     LOGINFO("   C1TXIF   = 0x%08X", CCSD.Uint32[ 9]);
//     LOGINFO("   C1RXOVIF = 0x%08X", CCSD.Uint32[10]);
//     LOGINFO("   C1TXATIF = 0x%08X", CCSD.Uint32[11]);
//     LOGINFO("   C1TXREQ  = 0x%08X", CCSD.Uint32[12]);
//     LOGINFO("   C1TREC   = 0x%08X", CCSD.Uint32[13]);
//     LOGINFO("   C1BDIAG0 = 0x%08X", CCSD.Uint32[14]);
//     LOGINFO("   C1BDIAG1 = 0x%08X", CCSD.Uint32[15]);
//   }
//   else ShowDeviceError(pComp, Error);
// }



// #define CAN_SFR_FIFO_SIZE  ( (3+1+3+3*31) * sizeof(unsigned int) )

// typedef union
// {
//   unsigned int Uint32[CAN_SFR_FIFO_SIZE / sizeof(unsigned int)];
//   uint8_t      Bytes[CAN_SFR_FIFO_SIZE / sizeof(uint8_t)];
// } FIFOSFRData;

// void GetAndShowMCP251XFD_FIFOreg(MCP251XFD *pComp)
// {
//   FIFOSFRData FSD;
//   eERRORRESULT Error = MCP251XFD_ReadData(pComp, RegMCP251XFD_CiTEFCON, &FSD.Bytes[0], CAN_SFR_FIFO_SIZE);
//   if (Error == ERR_OK)
//   {
//     LOGINFO(" CAN Controller FIFOs:");
//     LOGINFO("   TEF   : C1TEFCON    = 0x%08X ; C1TEFSTA    = 0x%08X ; C1TEFUA    = 0x%08X", FSD.Uint32[ 0], FSD.Uint32[ 1], FSD.Uint32[ 2]);
//     LOGINFO("   TXQ   : C1TXQCON    = 0x%08X ; C1TXQSTA    = 0x%08X ; C1TXQUA    = 0x%08X", FSD.Uint32[ 4], FSD.Uint32[ 5], FSD.Uint32[ 6]);
//     LOGINFO("   FIFO1 : C1FIFOCON1  = 0x%08X ; C1FIFOSTA1  = 0x%08X ; C1FIFOUA1  = 0x%08X", FSD.Uint32[ 7], FSD.Uint32[ 8], FSD.Uint32[ 9]);
//     LOGINFO("   FIFO2 : C1FIFOCON2  = 0x%08X ; C1FIFOSTA2  = 0x%08X ; C1FIFOUA2  = 0x%08X", FSD.Uint32[10], FSD.Uint32[11], FSD.Uint32[12]);
//     LOGINFO("   FIFO3 : C1FIFOCON3  = 0x%08X ; C1FIFOSTA3  = 0x%08X ; C1FIFOUA3  = 0x%08X", FSD.Uint32[13], FSD.Uint32[14], FSD.Uint32[15]);
//     LOGINFO("   FIFO4 : C1FIFOCON4  = 0x%08X ; C1FIFOSTA4  = 0x%08X ; C1FIFOUA4  = 0x%08X", FSD.Uint32[16], FSD.Uint32[17], FSD.Uint32[18]);
//     LOGINFO("   FIFO5 : C1FIFOCON5  = 0x%08X ; C1FIFOSTA5  = 0x%08X ; C1FIFOUA5  = 0x%08X", FSD.Uint32[19], FSD.Uint32[20], FSD.Uint32[21]);
//     LOGINFO("   FIFO6 : C1FIFOCON6  = 0x%08X ; C1FIFOSTA6  = 0x%08X ; C1FIFOUA6  = 0x%08X", FSD.Uint32[22], FSD.Uint32[23], FSD.Uint32[24]);
//     LOGINFO("   FIFO7 : C1FIFOCON7  = 0x%08X ; C1FIFOSTA7  = 0x%08X ; C1FIFOUA7  = 0x%08X", FSD.Uint32[25], FSD.Uint32[26], FSD.Uint32[27]);
//     LOGINFO("   FIFO8 : C1FIFOCON8  = 0x%08X ; C1FIFOSTA8  = 0x%08X ; C1FIFOUA8  = 0x%08X", FSD.Uint32[28], FSD.Uint32[29], FSD.Uint32[30]);
//     LOGINFO("   FIFO9 : C1FIFOCON9  = 0x%08X ; C1FIFOSTA9  = 0x%08X ; C1FIFOUA9  = 0x%08X", FSD.Uint32[31], FSD.Uint32[32], FSD.Uint32[33]);
//     LOGINFO("   FIFO10: C1FIFOCON10 = 0x%08X ; C1FIFOSTA10 = 0x%08X ; C1FIFOUA10 = 0x%08X", FSD.Uint32[34], FSD.Uint32[35], FSD.Uint32[36]);
//     LOGINFO("   FIFO11: C1FIFOCON11 = 0x%08X ; C1FIFOSTA11 = 0x%08X ; C1FIFOUA11 = 0x%08X", FSD.Uint32[37], FSD.Uint32[38], FSD.Uint32[39]);
//     LOGINFO("   FIFO12: C1FIFOCON12 = 0x%08X ; C1FIFOSTA12 = 0x%08X ; C1FIFOUA12 = 0x%08X", FSD.Uint32[40], FSD.Uint32[41], FSD.Uint32[42]);
//     LOGINFO("   FIFO13: C1FIFOCON13 = 0x%08X ; C1FIFOSTA13 = 0x%08X ; C1FIFOUA13 = 0x%08X", FSD.Uint32[43], FSD.Uint32[44], FSD.Uint32[45]);
//     LOGINFO("   FIFO14: C1FIFOCON14 = 0x%08X ; C1FIFOSTA14 = 0x%08X ; C1FIFOUA14 = 0x%08X", FSD.Uint32[46], FSD.Uint32[47], FSD.Uint32[48]);
//     LOGINFO("   FIFO15: C1FIFOCON15 = 0x%08X ; C1FIFOSTA15 = 0x%08X ; C1FIFOUA15 = 0x%08X", FSD.Uint32[49], FSD.Uint32[50], FSD.Uint32[51]);
//     LOGINFO("   FIFO16: C1FIFOCON16 = 0x%08X ; C1FIFOSTA16 = 0x%08X ; C1FIFOUA16 = 0x%08X", FSD.Uint32[52], FSD.Uint32[53], FSD.Uint32[54]);
//     LOGINFO("   FIFO17: C1FIFOCON17 = 0x%08X ; C1FIFOSTA17 = 0x%08X ; C1FIFOUA17 = 0x%08X", FSD.Uint32[55], FSD.Uint32[56], FSD.Uint32[57]);
//     LOGINFO("   FIFO18: C1FIFOCON18 = 0x%08X ; C1FIFOSTA18 = 0x%08X ; C1FIFOUA18 = 0x%08X", FSD.Uint32[58], FSD.Uint32[59], FSD.Uint32[60]);
//     LOGINFO("   FIFO19: C1FIFOCON19 = 0x%08X ; C1FIFOSTA19 = 0x%08X ; C1FIFOUA19 = 0x%08X", FSD.Uint32[61], FSD.Uint32[62], FSD.Uint32[63]);
//     LOGINFO("   FIFO20: C1FIFOCON20 = 0x%08X ; C1FIFOSTA20 = 0x%08X ; C1FIFOUA20 = 0x%08X", FSD.Uint32[64], FSD.Uint32[65], FSD.Uint32[66]);
//     LOGINFO("   FIFO21: C1FIFOCON21 = 0x%08X ; C1FIFOSTA21 = 0x%08X ; C1FIFOUA21 = 0x%08X", FSD.Uint32[67], FSD.Uint32[68], FSD.Uint32[69]);
//     LOGINFO("   FIFO22: C1FIFOCON22 = 0x%08X ; C1FIFOSTA22 = 0x%08X ; C1FIFOUA22 = 0x%08X", FSD.Uint32[70], FSD.Uint32[71], FSD.Uint32[72]);
//     LOGINFO("   FIFO23: C1FIFOCON23 = 0x%08X ; C1FIFOSTA23 = 0x%08X ; C1FIFOUA23 = 0x%08X", FSD.Uint32[73], FSD.Uint32[74], FSD.Uint32[75]);
//     LOGINFO("   FIFO24: C1FIFOCON24 = 0x%08X ; C1FIFOSTA24 = 0x%08X ; C1FIFOUA24 = 0x%08X", FSD.Uint32[76], FSD.Uint32[77], FSD.Uint32[78]);
//     LOGINFO("   FIFO25: C1FIFOCON25 = 0x%08X ; C1FIFOSTA25 = 0x%08X ; C1FIFOUA25 = 0x%08X", FSD.Uint32[79], FSD.Uint32[80], FSD.Uint32[81]);
//     LOGINFO("   FIFO26: C1FIFOCON26 = 0x%08X ; C1FIFOSTA26 = 0x%08X ; C1FIFOUA26 = 0x%08X", FSD.Uint32[82], FSD.Uint32[83], FSD.Uint32[84]);
//     LOGINFO("   FIFO27: C1FIFOCON27 = 0x%08X ; C1FIFOSTA27 = 0x%08X ; C1FIFOUA27 = 0x%08X", FSD.Uint32[85], FSD.Uint32[86], FSD.Uint32[87]);
//     LOGINFO("   FIFO28: C1FIFOCON28 = 0x%08X ; C1FIFOSTA28 = 0x%08X ; C1FIFOUA28 = 0x%08X", FSD.Uint32[88], FSD.Uint32[89], FSD.Uint32[90]);
//     LOGINFO("   FIFO29: C1FIFOCON29 = 0x%08X ; C1FIFOSTA29 = 0x%08X ; C1FIFOUA29 = 0x%08X", FSD.Uint32[91], FSD.Uint32[92], FSD.Uint32[93]);
//     LOGINFO("   FIFO30: C1FIFOCON30 = 0x%08X ; C1FIFOSTA30 = 0x%08X ; C1FIFOUA30 = 0x%08X", FSD.Uint32[94], FSD.Uint32[95], FSD.Uint32[96]);
//     LOGINFO("   FIFO31: C1FIFOCON31 = 0x%08X ; C1FIFOSTA31 = 0x%08X ; C1FIFOUA31 = 0x%08X", FSD.Uint32[97], FSD.Uint32[98], FSD.Uint32[99]);
//   }
//   else ShowDeviceError(pComp, Error);
// }



// #define CAN_SFR_FILTER_SIZE  ( (8+2*32) * sizeof(unsigned int) )

// typedef union
// {
//   unsigned int Uint32[CAN_SFR_FILTER_SIZE / sizeof(unsigned int)];
//   uint8_t      Bytes[CAN_SFR_FILTER_SIZE / sizeof(uint8_t)];
// } FilterSFRData;

// void GetAndShowMCP251XFD_FILTERreg(MCP251XFD *pComp)
// {
//   FIFOSFRData FSD;
//   eERRORRESULT Error = MCP251XFD_ReadData(pComp, RegMCP251XFD_CiFLTCON0, &FSD.Bytes[0], CAN_SFR_FILTER_SIZE);
//   if (Error == ERR_OK)
//   {
//     LOGINFO(" CAN Controller Filters:");
//     LOGINFO("   Filter0 : C1FLTCON0 = 0x%02X ; C1FLTOBJ0  = 0x%08X ; C1MASK0  = 0x%08X", FSD.Bytes[ 0], FSD.Uint32[ 8], FSD.Uint32[ 9]);
//     LOGINFO("   Filter1 : C1FLTCON0 = 0x%02X ; C1FLTOBJ1  = 0x%08X ; C1MASK1  = 0x%08X", FSD.Bytes[ 1], FSD.Uint32[10], FSD.Uint32[11]);
//     LOGINFO("   Filter2 : C1FLTCON0 = 0x%02X ; C1FLTOBJ2  = 0x%08X ; C1MASK2  = 0x%08X", FSD.Bytes[ 2], FSD.Uint32[12], FSD.Uint32[13]);
//     LOGINFO("   Filter3 : C1FLTCON0 = 0x%02X ; C1FLTOBJ3  = 0x%08X ; C1MASK3  = 0x%08X", FSD.Bytes[ 3], FSD.Uint32[14], FSD.Uint32[15]);
//     LOGINFO("   Filter4 : C1FLTCON1 = 0x%02X ; C1FLTOBJ4  = 0x%08X ; C1MASK4  = 0x%08X", FSD.Bytes[ 4], FSD.Uint32[16], FSD.Uint32[17]);
//     LOGINFO("   Filter5 : C1FLTCON1 = 0x%02X ; C1FLTOBJ5  = 0x%08X ; C1MASK5  = 0x%08X", FSD.Bytes[ 5], FSD.Uint32[18], FSD.Uint32[19]);
//     LOGINFO("   Filter6 : C1FLTCON1 = 0x%02X ; C1FLTOBJ6  = 0x%08X ; C1MASK6  = 0x%08X", FSD.Bytes[ 6], FSD.Uint32[20], FSD.Uint32[21]);
//     LOGINFO("   Filter7 : C1FLTCON1 = 0x%02X ; C1FLTOBJ7  = 0x%08X ; C1MASK7  = 0x%08X", FSD.Bytes[ 7], FSD.Uint32[22], FSD.Uint32[23]);
//     LOGINFO("   Filter8 : C1FLTCON2 = 0x%02X ; C1FLTOBJ8  = 0x%08X ; C1MASK8  = 0x%08X", FSD.Bytes[ 8], FSD.Uint32[24], FSD.Uint32[25]);
//     LOGINFO("   Filter9 : C1FLTCON2 = 0x%02X ; C1FLTOBJ9  = 0x%08X ; C1MASK9  = 0x%08X", FSD.Bytes[ 9], FSD.Uint32[26], FSD.Uint32[27]);
//     LOGINFO("   Filter10: C1FLTCON2 = 0x%02X ; C1FLTOBJ10 = 0x%08X ; C1MASK10 = 0x%08X", FSD.Bytes[10], FSD.Uint32[28], FSD.Uint32[29]);
//     LOGINFO("   Filter11: C1FLTCON2 = 0x%02X ; C1FLTOBJ11 = 0x%08X ; C1MASK11 = 0x%08X", FSD.Bytes[11], FSD.Uint32[30], FSD.Uint32[31]);
//     LOGINFO("   Filter12: C1FLTCON3 = 0x%02X ; C1FLTOBJ12 = 0x%08X ; C1MASK12 = 0x%08X", FSD.Bytes[12], FSD.Uint32[32], FSD.Uint32[33]);
//     LOGINFO("   Filter13: C1FLTCON3 = 0x%02X ; C1FLTOBJ13 = 0x%08X ; C1MASK13 = 0x%08X", FSD.Bytes[13], FSD.Uint32[34], FSD.Uint32[35]);
//     LOGINFO("   Filter14: C1FLTCON3 = 0x%02X ; C1FLTOBJ14 = 0x%08X ; C1MASK14 = 0x%08X", FSD.Bytes[14], FSD.Uint32[36], FSD.Uint32[37]);
//     LOGINFO("   Filter15: C1FLTCON3 = 0x%02X ; C1FLTOBJ15 = 0x%08X ; C1MASK15 = 0x%08X", FSD.Bytes[15], FSD.Uint32[38], FSD.Uint32[39]);
//     LOGINFO("   Filter16: C1FLTCON4 = 0x%02X ; C1FLTOBJ16 = 0x%08X ; C1MASK16 = 0x%08X", FSD.Bytes[16], FSD.Uint32[40], FSD.Uint32[41]);
//     LOGINFO("   Filter17: C1FLTCON4 = 0x%02X ; C1FLTOBJ17 = 0x%08X ; C1MASK17 = 0x%08X", FSD.Bytes[17], FSD.Uint32[42], FSD.Uint32[43]);
//     LOGINFO("   Filter18: C1FLTCON4 = 0x%02X ; C1FLTOBJ18 = 0x%08X ; C1MASK18 = 0x%08X", FSD.Bytes[18], FSD.Uint32[44], FSD.Uint32[45]);
//     LOGINFO("   Filter19: C1FLTCON4 = 0x%02X ; C1FLTOBJ19 = 0x%08X ; C1MASK19 = 0x%08X", FSD.Bytes[19], FSD.Uint32[46], FSD.Uint32[47]);
//     LOGINFO("   Filter20: C1FLTCON5 = 0x%02X ; C1FLTOBJ20 = 0x%08X ; C1MASK20 = 0x%08X", FSD.Bytes[20], FSD.Uint32[48], FSD.Uint32[49]);
//     LOGINFO("   Filter21: C1FLTCON5 = 0x%02X ; C1FLTOBJ21 = 0x%08X ; C1MASK21 = 0x%08X", FSD.Bytes[21], FSD.Uint32[50], FSD.Uint32[51]);
//     LOGINFO("   Filter22: C1FLTCON5 = 0x%02X ; C1FLTOBJ22 = 0x%08X ; C1MASK22 = 0x%08X", FSD.Bytes[22], FSD.Uint32[52], FSD.Uint32[53]);
//     LOGINFO("   Filter23: C1FLTCON5 = 0x%02X ; C1FLTOBJ23 = 0x%08X ; C1MASK23 = 0x%08X", FSD.Bytes[23], FSD.Uint32[54], FSD.Uint32[55]);
//     LOGINFO("   Filter24: C1FLTCON6 = 0x%02X ; C1FLTOBJ24 = 0x%08X ; C1MASK24 = 0x%08X", FSD.Bytes[24], FSD.Uint32[56], FSD.Uint32[57]);
//     LOGINFO("   Filter25: C1FLTCON6 = 0x%02X ; C1FLTOBJ25 = 0x%08X ; C1MASK25 = 0x%08X", FSD.Bytes[25], FSD.Uint32[58], FSD.Uint32[59]);
//     LOGINFO("   Filter26: C1FLTCON6 = 0x%02X ; C1FLTOBJ26 = 0x%08X ; C1MASK26 = 0x%08X", FSD.Bytes[26], FSD.Uint32[60], FSD.Uint32[61]);
//     LOGINFO("   Filter27: C1FLTCON6 = 0x%02X ; C1FLTOBJ27 = 0x%08X ; C1MASK27 = 0x%08X", FSD.Bytes[27], FSD.Uint32[62], FSD.Uint32[63]);
//     LOGINFO("   Filter28: C1FLTCON7 = 0x%02X ; C1FLTOBJ28 = 0x%08X ; C1MASK28 = 0x%08X", FSD.Bytes[28], FSD.Uint32[64], FSD.Uint32[65]);
//     LOGINFO("   Filter29: C1FLTCON7 = 0x%02X ; C1FLTOBJ29 = 0x%08X ; C1MASK29 = 0x%08X", FSD.Bytes[29], FSD.Uint32[66], FSD.Uint32[67]);
//     LOGINFO("   Filter30: C1FLTCON7 = 0x%02X ; C1FLTOBJ30 = 0x%08X ; C1MASK30 = 0x%08X", FSD.Bytes[30], FSD.Uint32[68], FSD.Uint32[69]);
//     LOGINFO("   Filter31: C1FLTCON7 = 0x%02X ; C1FLTOBJ31 = 0x%08X ; C1MASK31 = 0x%08X", FSD.Bytes[31], FSD.Uint32[70], FSD.Uint32[71]);
//   }
//   else ShowDeviceError(pComp, Error);
// }

void ShowDeviceError(MCP251XFD *pComp, eERRORRESULT error) {
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
       LOGERROR("Device error: %s", pStr);
  else LOGERROR("Device error: Unknown error (%u)", (unsigned int)error);
}