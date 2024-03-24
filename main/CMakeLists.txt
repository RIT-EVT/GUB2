
set(SOURCES 
    "main.c" 
    "GUB2.c" 
    "Fileserver.c" 
    "CANLogger.c"
    "drivers/WIFIDriver.c" 
    "drivers/Driver_MCP251863.c" 
    "drivers/CANDriver.c"
    "datatypes/CircularBuffer.c"
)


idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS ".")