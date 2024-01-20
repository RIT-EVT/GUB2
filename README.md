# Gateway Utility Board Rev. 2 (GUB2)

> [!WARNING]
> **This is currently a proof of concept project to verify that the ESP32 will work for our needs!** Future commits will clean up the project and follow better codding standards.

## Introduction

The Gateway Utility Board Rev. 2 (GUB2) is responsible for recording all the CAN messages sent on the bike and logging them for future debugging. The board utilizes an esp32s3 for creating a wifi access point to remotely retrieve logs. 

### Current Progress
- [x] setup simple project and install toolchain
- [x] get UART logging working
- [x] get CANbus through MCP251863 to UART working
- [x] get SD card file writing with raw CAN message dumps
- [x] get wifi ap working - maybe wifi station + AP
- [x] simple http server for listing and reading sd card contents
- [ ] generate vectorblf files for CAN logs
- [ ] clean up code

## Build Tools Setup Steps
> [!IMPORTANT]
> This project does not use the standard EVT-core abstraction layer and corresponding built tools! 

Before building this project, the ESP-IDF tool chains needs to be installed. Follow the [install guide](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md) from Espressif. 

Once the build tools are setup, clone this repository with:
```
git clone --recurse-submodules https://github.com/RIT-EVT/GUB2.git
```
Once the repository is cloned, the project can be built in VSCode using the ESP-IDF extension and pressing the build button on the bottom status bar. The project can also be built with `cmake build` from the ESP-IDF Terminal.



## Future Features
- GPS Logging 
- LoRa data broadcasts.
