#include "FileLogger.h"

#include "Events.h"

void fileOnCANMessage(void *arg, esp_event_base_t base, int32_t eventId, void* eventData){
    // printf("Got CAN message");
}

void fileLoggerInit(){
    esp_event_handler_register(CANBUS_EVENT, CAN_MESSAGE_RECIEVED, fileOnCANMessage, NULL);
}

void fileLoggerUpdate(){

}