#include "dataProcessing.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <math.h>

//DATA PROCESSING CONSTANTS
#define STDDEV_THRESHOLD 0.01
#define STATION_TIME 5
#define RUN_TIME 600

#define FS 2000
#define BYTES_PER_SAMPLE 6
#define BYTES_PER_SECOND 12000
#define SCALE_FACTOR 0.00048828
static const char *TAG = "PROCESSING";

void unPackSamples(uint8_t *inBuff, int16_t *x, int16_t *y, int16_t *z, int numSamples, int bufIndex);

int getSignedInt(uint16_t rawValue);

void processingTask(void *pvParams)
{

    QueueHandle_t dataQueue = (QueueHandle_t)pvParams;
    uint8_t *dataBuff;

    int16_t xData[FS];
    int16_t yData[FS];
    int16_t zData[FS];
    int bufIndex = 0;

    //stdDev Buffer for station detection
    float yA[FS];

    //store squares values buffer for rms
    float x2[FS];
    float y2[FS];
    float z2[FS];
    

    //rms force each second buffer. Holds up to one hour of data. Circular Pointer
    float xSec[3200], ySec[3200], zSec[3200];
    int startIndex = 0; 
    int nowIndex = 0; 


    //vars for station detection and rms calculation
    float xSum, ySum, ySumStd, zSum, yAvg, yStd = 0;

    //counter increments by 1 for every second yStd(standard deviation in Y) is less than STDDEV_THRESHOLD
    int stoppedCounter = 0;
    int motionCounter = 0; //do we need MOTION_THRESHOLD ?
    int atStation = 1;  //train is stationary in the beginning
    int inMotion = 0;  //train is stationary in the beginning    

    while (1)
    {
        

        if (inMotion)
        {
            
            if (stoppedCounter > 5)
            {
                atStation = 1;
                inMotion = 0;
                ESP_LOGI(TAG, "************ ARRIVED AT A STOP 5s AGO ***");      
                //get percentile and TX data
                //ON STATION DETECTED
            }
        }
        else if (atStation)
        {
            if (motionCounter > 5)
            {
                ESP_LOGI(TAG, "************ STARTED MOVING 5s AGO ***");      
                inMotion = 1;
                atStation = 0;
                //ON MOTION DETECTED
                //processAndTx()
            }
        }          
        //Wait until we have data.
        xQueueReceive(dataQueue, &dataBuff, portMAX_DELAY);

        //Unpack the data.
        uint8_t node_id = dataBuff[244];
        uint32_t frameNum = *((uint32_t *)&dataBuff[240]);

        unPackSamples(dataBuff, xData, yData, zData, 40, bufIndex);

        bufIndex = bufIndex + 40;
        // collect 1 second worth of samples before processing
        // Should processsing be a separeate task from buffering ? Yeah probably..
        if (bufIndex == FS)
        {
            xSum = 0;
            ySum = 0;
            ySumStd = 0;
            zSum = 0;
            yAvg = 0;
            yStd = 0;
            bufIndex = 0;   
            //ESP_LOGI(TAG, " x: %d, y: %d, z:%d", xData[10], yData[10], zData[10]);
            //ESP_LOGI(TAG, " x: %f, y: %f, z:%f", (float)xData[10] * SCALE_FACTOR, yData[10]*0.00048828, (float)zData[10]*0.00048828);
            for (int i = 0; i < FS; i++)
            {
                
                
                x2[i] = pow((xData[i]) * SCALE_FACTOR, 2); 
                y2[i] = pow((yData[i]) * SCALE_FACTOR, 2); 
                z2[i] = pow((zData[i]) * SCALE_FACTOR, 2); 
                yA[i] = yData[i]* SCALE_FACTOR;

                xSum = xSum + x2[i];
                ySum = ySum + y2[i];
                zSum = zSum + z2[i];
                ySumStd = ySumStd + yA[i];
            }
            yAvg = (ySumStd / FS);
            xSec[nowIndex] = sqrtf(xSum / FS);
            ySec[nowIndex] = sqrtf(ySum / FS);
            zSec[nowIndex] = sqrtf(zSum / FS);
            
            for (int yi = 0; yi < FS; yi++)
            {
                yStd += pow(yAvg - yA[yi], 2);
            }
            yStd = sqrtf(yStd / 1000); 

            if (yStd < STDDEV_THRESHOLD)
            {
                stoppedCounter = stoppedCounter + 1;
                motionCounter = 0;
            }
            else
            {
                stoppedCounter = 0;
                motionCounter = motionCounter + 1;
            }


                   

            //LOG
            ESP_LOGI(TAG, "Processing frame %d from node %d", frameNum, node_id);
            ESP_LOGI(TAG, "x: %f, y: %f, z:%f, yStd:%f", xSec[second], ySec[second], zSec[second], yStd);
            ESP_LOGI(TAG, "STOPPED: %d  MOTION: %d",stoppedCounter, motionCounter);      
            nowIndex= nowIndex+1;  
            
        }
    }
}

void unPackSamples(uint8_t *inBuff, int16_t *x, int16_t *y, int16_t *z, int numSamples, int startIndex)
{
    //ESP_LOGI(TAG, "START INDEX %d",startIndex);     

    for (int i = 0; i < (numSamples); i++)
    {
        x[i+startIndex] = *((int16_t *)&inBuff[i * 6]);
        y[i+startIndex] = *((int16_t *)&inBuff[i * 6 + 2]);
        z[i+startIndex] = *((int16_t *)&inBuff[i * 6 + 4]);
        //ESP_LOGI(TAG, "%d | x: %d, y: %d, z:%d",i+startIndex, x[i+startIndex], y[i+startIndex], z[i+startIndex]);
    }
    
}

int getSignedInt(uint16_t rawValue)
{
    if ((rawValue & 0x8000) == 0)
    {
        return rawValue;
    }
    return (uint16_t)(~(rawValue - 0x01)) * -1;
}