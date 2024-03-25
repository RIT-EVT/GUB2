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

Before building this project, the ESP-IDF tool chains needs to be installed. There is an [install guide](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md) provided by Espressif that should be referenced if any issue arise. First, install Visual Studio Code and the "ESP-IDF" extension. Once installed click on the "ESP-IDF: Explorer" icon on the left side bar. Then the "Configure ESP-IDF Extension" option under the commands. In this menu, click on "Express" and under the ESP-IDF version, select v5.1.1 (release). Once installed, the rest of the project can be built and flashed to the board. 

> If you encounter `esp_lcd_panel_rgb.c:747:1: internal compiler error: Segmentation fault`, pray it goes away.

Once the build tools are setup, clone this repository with:
```
git clone --recurse-submodules https://github.com/RIT-EVT/GUB2.git
```
Once the repository is cloned, the project can be built in VSCode using the ESP-IDF extension and pressing the build button on the bottom status bar. The project can also be built with `cmake build` from the ESP-IDF Terminal.

## Debugging
For debugging the board you will likely have to launch openocd manually as it will often times fail. To do so open the ESP-IDF terminal and enter one of the two following commands depending on the board. 

For the ESP32-S3-DevKitC-1 board or built in usb debugger: 
`openocd -f "board/esp32s3-builtin.cfg"` \
For the GUB:
`openocd -s share/openocd/scripts -f interface/ftdi/esp32_devkitj_v1.cfg -f board/esp32s3-ftdi.cfg`

<!-- ## Project Structure -->

## Additional notes
The Freertos tick rate has been set to 1000 Hz (1ms). This is the smallest unit of delay a task can wait for! Current project has about a 1-2sec startup time.

## Future Features
- Log CAN errors
- Better logging file format
- GPS Logging 
- LoRa data broadcasts.

## Current Issues
Many have been fixed, but SD card handling will likely cause issues. The code has been tested up to ~1800 messages per second while logging without loading the file list and can be pushed to ~2100 messages per second with some loss. Another issue is the SD card getting stuck into a persistent error state of 0x107. A full power cycle may fix this issue or it may just be an issue with my breadboard setup. 

A future addition should be a better logging format than a csv since a CAN message with 4 bytes of data takes up 32 bytes of file space at say 1800 messages per second is 57.6 KB/s of space. 



<!-- The Board, must start up in an environment of less than 76 messages per second or the board will be stuck in a boot loop until the number of messages reduces below this limit. Once booted, the board can currently handle ~1700 messages per second before file writing. -->

<!-- ## Example folder contents -->

<!-- The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py. -->
