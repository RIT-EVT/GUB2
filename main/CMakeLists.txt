
set(SOURCES 
    "main.c" 
    "GUB2.c" 
   
    "Fileserver.c" 
    "Filelogger.c"
    "drivers/WIFIDriver.c" 
    "drivers/Driver_MCP251863.c" 
    "drivers/CANDriver.c"
    "datatypes/CircularBuffer.c"
    "Events.c"
)


idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS ".")