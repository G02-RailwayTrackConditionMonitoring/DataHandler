#include "dataProcessing.h"
#include "modem.h"
#include "GatewayCommands.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <math.h>
#include "driver/uart.h"
#include "GatewayCommands.h"
#include <string.h>

//DATA PROCESSING CONSTANTS
//#define STDDEV_THRESHOLD 0.01 //unused
#define STATION_TIME 5
#define RUN_TIME 600

#define FS 2000
#define BYTES_PER_SAMPLE 6
#define BYTES_PER_SECOND 12000
#define SCALE_FACTOR 0.00048828
static const char *TAG = "PROCESSING";

void unPackSamples(uint8_t *inBuff, int16_t *x, int16_t *y, int16_t *z, int numSamples, int bufIndex);
int compare(const void *a, const void *b);

void processingTask(void *pvParams)
{
    char uartSendBuf[255] = "";

    int64_t m1;
    int64_t m2;
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

    //rms force each second buffer. Holds up to one hour of data.
    float xSec[3600], ySec[3600], zSec[3600], yStdSec[3600];
    int startIndex = 0;
    int nowIndex = 0;
    int interval;

    //vars for station detection and rms calculation
    float xSum, ySum, ySumStd, zSum, yAvg, yStd = 0;

    //counter increments by 1 for every second yStd(standard deviation in Y) is less than STDDEV_THRESHOLD
    int stoppedCounter = 0;
    int motionCounter = 0; 
    int atStation = 1;     //train is stationary in the beginning
    int inMotion = 0;      //train is stationary in the beginning
    char buf[255];

    int getLoc3 = 1; // flag to get location at 3 seconds only once

    while (1)
    {
        if (inMotion)
        {
            if ((stoppedCounter == 3) && (getLoc3 == 1))
            {
                sprintf(buf, "%d: %d\n", SET_GPS, 0);
                uart_write_bytes(2, buf, strlen(buf));
                getLoc3 = 0;
            }
            if (stoppedCounter > 5)
            {
                getLoc3 = 1;
                atStation = 1;
                inMotion = 0;
                ESP_LOGI(TAG, "************ ARRIVED AT A STOP 5s AGO ***");
                //get percentile and TX data

                //ON STATION DETECTED
                interval = (nowIndex - 5);
                // m1 = esp_timer_get_time();
                qsort(xSec, interval, sizeof(int), compare);
                qsort(ySec, interval, sizeof(int), compare);
                qsort(zSec, interval, sizeof(int), compare);
                //ESP_LOGI(TAG, "Q SORT %d items:  %lld", interval, esp_timer_get_time()-m1);
                ESP_LOGI(TAG, "PREVIOUS TRIP DATA: %d %d", startIndex, nowIndex - 4);
                ESP_LOGI(TAG, "60th Percentile X: %f \n", xSec[(int)(0.6 * interval)]);
                ESP_LOGI(TAG, "60th Percentile Y: %f \n", ySec[(int)(0.6 * interval)]);
                ESP_LOGI(TAG, "60th Percentile Z: %f \n", zSec[(int)(0.6 * interval)]);

                //UART Command send

                sprintf(buf, "%d: %d\n", SET_END_TIME, 0);
                uart_write_bytes(2, buf, strlen(buf));

                //char buf[255];
                sprintf(buf, "%d: %f,%f,%f\n", AVG_FORCE_DATA, xSec[(int)(0.6 * interval)], ySec[(int)(0.6 * interval)], zSec[(int)(0.6 * interval)]);
                uart_write_bytes(2, buf, strlen(buf));

                //reset buffer
                
                for (int i = 0; i < 5; i++)
                {
                    xSec[i] = xSec[nowIndex - (5 - i)];
                    ySec[i] = ySec[nowIndex - (5 - i)];
                    zSec[i] = zSec[nowIndex - (5 - i)];
                }
                nowIndex = 5;
                startIndex = 0;
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

                //char buf[255];
                sprintf(buf, "%d: %d\n", SET_GPS, 0);
                uart_write_bytes(2, buf, strlen(buf));

                sprintf(buf, "%d: %d\n", SET_START_TIME, 0);
                uart_write_bytes(2, buf, strlen(buf));

                //reset buffer
                for (int i = 0; i < 5; i++)
                {
                    xSec[i] = xSec[nowIndex - (5 - i)];
                    ySec[i] = ySec[nowIndex - (5 - i)];
                    zSec[i] = zSec[nowIndex - (5 - i)];
                }
                nowIndex = 5;
            }
        }

        //Wait until we have data.(every 40 samples = 20 ms)
        xQueueReceive(dataQueue, &dataBuff, portMAX_DELAY);
        //Unpack the data.
        uint8_t node_id = dataBuff[244];
        uint32_t frameNum = *((uint32_t *)&dataBuff[240]);
        unPackSamples(dataBuff, xData, yData, zData, 40, bufIndex);

        bufIndex = bufIndex + 40;
        // collect 1 second worth of samples before processing
        if (bufIndex == FS)
        {

            // m1 = esp_timer_get_time();
            xSum = 0;
            ySum = 0;
            ySumStd = 0;
            zSum = 0;
            yAvg = 0;
            yStd = 0;
            bufIndex = 0;
            // m2 = esp_timer_get_time();
            ESP_LOGI(TAG, "x: %d, y: %d, z:%d",xData[1000], yData[1000], zData[1000]);
            ESP_LOGI(TAG, "INITIAL | xSum: %f, ySum: %f, zSum:%f, yStdSum:%f",  xSum, ySum, zSum, ySumStd);
            for (int i = 0; i < FS; i++)
            {
                //ESP_LOGI(TAG, " DURING | xSum: %f, ySum: %f, zSum:%f, yStdSum:%f",  pow((xData[i]) * SCALE_FACTOR, 2), pow((xData[i]) * SCALE_FACTOR, 2), pow((xData[i]) * SCALE_FACTOR, 2), ySumStd);
                x2[i] = pow((xData[i]) * SCALE_FACTOR, 2);
                y2[i] = pow((yData[i]) * SCALE_FACTOR, 2);
                z2[i] = pow((zData[i]) * SCALE_FACTOR, 2);
                yA[i] = yData[i] * SCALE_FACTOR;

                xSum = xSum + x2[i];
                ySum = ySum + y2[i];
                zSum = zSum + z2[i];
                ySumStd = ySumStd + yA[i];
        
            }
            // ESP_LOGI(TAG, "LOOP 1 SECOND: %lld",esp_timer_get_time()-m2);
            yAvg = (ySumStd / FS);

            xSec[nowIndex] = sqrtf(xSum / FS);
            ySec[nowIndex] = sqrtf(ySum / FS);
            zSec[nowIndex] = sqrtf(zSum / FS);

            for (int yi = 0; yi < FS; yi++)
            {
                yStd = yStd + pow(yAvg - yA[yi], 2);
            }
            yStd = sqrtf(yStd / 1000);
            

            if (yStd < threshold)
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
            //ESP_LOGI(TAG, "Processing frame %d from node %d", frameNum, node_id);
            ESP_LOGI(TAG, "  FINAL | xSum: %f, ySum: %f, zSum:%f, yStdSum:%f", xSum, ySum, zSum, ySumStd);
            ESP_LOGI(TAG, "x: %f, y: %f, z:%f, yStd:%.2f", xSec[nowIndex], ySec[nowIndex], zSec[nowIndex], yStd);
            ESP_LOGI(TAG, "STOPPED: %d  MOTION: %d", stoppedCounter, motionCounter);
            ESP_LOGI(TAG, "THRESHOLD: %f", threshold);
            
            
            if(nowIndex>=3600){ //reset  because buffer is only 3600 seconds long 
                 nowIndex = 0; 
            }
            nowIndex = nowIndex + 1;
            // ESP_LOGI(TAG, "PROCESS 1 SECOND: %lld",esp_timer_get_time()-m1);
            // minuteCounter = minuteCounter + 1;
        }


    }
}

void unPackSamples(uint8_t *inBuff, int16_t *x, int16_t *y, int16_t *z, int numSamples, int startIndex)
{
    //ESP_LOGI(TAG, "START INDEX %d",startIndex);
    //int64_t m1= esp_timer_get_time();
    //ESP_LOGI(TAG, "%d | x: %d, y: %d, z:%d",startIndex, x[startIndex], y[startIndex], z[startIndex]);
    for (int i = 0; i < (numSamples); i++)
    {
        x[i + startIndex] = *((int16_t *)&inBuff[i * 6]);
        y[i + startIndex] = *((int16_t *)&inBuff[i * 6 + 2]);
        z[i + startIndex] = *((int16_t *)&inBuff[i * 6 + 4]);
        //ESP_LOGI(TAG, "%d | x: %d, y: %d, z:%d",i+startIndex, x[i+startIndex], y[i+startIndex], z[i+startIndex]);
    }
    //ESP_LOGI(TAG, "UNPACK: %lld", esp_timer_get_time()-m1);
}

int compare(const void *a, const void *b)
{
    return *((float *)a) - *((float *)b);
}
