#include "GatewayCommands.h"
#include <string.h>

const char* GatewayCommand_Str[NUM_GATEWAY_COMMANDS]={
    "AVG_FORCE_DATA",
    "BLE_CONNECTION_EVENT",
    "BLE_RSSI_DATA",
    "LTE_RSSI_DATA",
    "NODE_BATTERY",
    "GATEWAY_BATTERY",
    "NODE_FREE_SPACE",
    "GATEWAY_FREE_SPACE",
    "TIME_UPDATE",
    "GATEWAY_BATTERY_REQ",
    "SET_GPS",
    "SET_START_TIME", 
    "SET_END_TIME",   
    "SET_THRESHOLD", 
};

uint8_t PreparePacket(uint8_t* buffer, GatewayUartPacket* packet){

    uint8_t totalLen = packet->len+2;

    buffer[0] = packet->command;
    buffer[1] =totalLen;
    
    memcpy(&buffer[2],&(packet->data),packet->len);

    return totalLen;
}

uint8_t GetPacket(uint8_t* buffer,GatewayUartPacket* packet){


    packet->command = (GatewayCommand_t)buffer[0];

    if(packet->command>NUM_GATEWAY_COMMANDS) return 1;  //Invalid command byte.

    packet->len = buffer[1]-2;  //Subract 2, for the command and len fields.
    
    memcpy(&packet->data,&buffer[2],packet->len);

    return 0;
}