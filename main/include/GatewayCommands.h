#ifndef GATEWAY_COMMANDS_H_
#define GATEWAY_COMMANDS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*  GatewayCommand_t: Defines the commands /data sent between ESP32 and Boron.
 *  
 *  AVG_FORCE_DATA: 
 */
typedef enum{

    AVG_FORCE_DATA,
    BLE_CONNECTION_EVENT,
    BLE_RSSI_DATA,
    LTE_RSSI_DATA,
    NODE_BATTERY,
    GATEWAY_BATTERY,
    NODE_FREE_SPACE,
    GATEWAY_FREE_SPACE,
    TIME_UPDATE,
    GATEWAY_BATTERY_REQ,
    SET_GPS,
    SET_START_TIME, 
    SET_END_TIME,
    SET_THRESHOLD,
    SET_MODE,
    TX_STD_Y,
    NUM_GATEWAY_COMMANDS

} GatewayCommand_t;

extern const char* GatewayCommand_Str[NUM_GATEWAY_COMMANDS];

typedef union {
    uint8_t uint8[252];
    uint16_t uint16[126];
    int16_t int16[126];
    float float32[63];
    double double64[31];
} packetData;

// GatewayUartPacket: struct for formating the uart communications.
typedef struct{

    GatewayCommand_t command;   
    uint8_t len;                //Size of the data, in bytes, ignoring the command and len field. Must be less than 253!

    packetData data;

} GatewayUartPacket;

//Copies the packet into a buffer, returns the number of bytes that should be sent.
uint8_t PreparePacket(uint8_t* buffer, GatewayUartPacket* packet);

//Copies data from a buffer into a packet. returns 0 if no error.
uint8_t GetPacket(uint8_t* buffer,GatewayUartPacket* packet);


#ifdef __cplusplus
}
#endif
#endif