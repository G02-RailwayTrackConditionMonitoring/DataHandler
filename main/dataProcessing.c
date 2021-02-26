#include "dataProcessing.h"
#include "modem.h"
#include "GatewayCommands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/uart.h"
#include <string.h>

static const char* TAG = "PROCESSING";

void unPackSamples(uint8_t* inBuff, int16_t* x, int16_t* y, int16_t* z,int numSamples);

void processingTask(void * pvParams){

    QueueHandle_t dataQueue = (QueueHandle_t)pvParams;
    uint8_t* dataBuff;

    int16_t xData[40];
    int16_t yData[40];
    int16_t zData[40];


    while(1){

    //Wait until we have data.
    xQueueReceive(dataQueue,&dataBuff,portMAX_DELAY);

    //Unpack the data.
    uint8_t node_id = dataBuff[244];
    uint32_t frameNum = *((uint32_t*)&dataBuff[240]);

    unPackSamples(dataBuff,xData,yData,zData,40);

    //Do processing...
    ESP_LOGI(TAG,"Processing frame %d from node %d",frameNum,node_id);
    ESP_LOGI(TAG,"x: %d, y: %d, z:%d",xData[0],yData[0],zData[0]);

    // if(frameNum%500 ==0){
    //     char buf[255];
    //     sprintf(buf,"%d: %d,%d,%d\n",AVG_FORCE_DATA,xData[0],yData[0],zData[0]);
    //     uart_write_bytes(ECHO_UART_PORT_NUM, buf, strlen(buf));
    // }
    }

}


void unPackSamples(uint8_t* inBuff, int16_t* x, int16_t* y, int16_t* z,int numSamples){


    for(int i=0; i<numSamples; i++){

        x[i] = *((int16_t*)&inBuff[i*6]);
        y[i] = *((int16_t*)&inBuff[i*6+2]);
        z[i] = *((int16_t*)&inBuff[i*6+4]);
    }
}
