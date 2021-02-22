#include "modem.h"

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/uart.h"
#include "driver/spi_slave.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include <string.h>


#include "sd_card.h"
#include "GatewayCommands.h"

static const char* TAG = "MODEM";

int8_t handleCommand(GatewayUartPacket packet);

uint8_t dataBuffer[10*245];
uint16_t dataBufferIdx=0;

/******************************************/
// SPI
/******************************************/
void IRAM_ATTR my_post_setup_cb(spi_slave_transaction_t *trans)
{
  WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << GPIO_HANDSHAKE));
}
/******************************************/
void IRAM_ATTR my_post_trans_cb(spi_slave_transaction_t *trans)
{
  WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << GPIO_HANDSHAKE));
}
/******************************************/
//INIT SPI
/******************************************/
esp_err_t init_spi_slave()
{
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .mosi_io_num = GPIO_MOSI,
      .miso_io_num = GPIO_MISO,
      .sclk_io_num = GPIO_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4094,
      //.intr_flags = ESP_INTR_FLAG_IRAM
  };

  spi_slave_interface_config_t slvcfg = {
      .mode = 3,
      .spics_io_num = GPIO_CS,
      .queue_size = 3,
      .flags = 0,
      .post_setup_cb = my_post_setup_cb,
      .post_trans_cb = my_post_trans_cb
      };

  gpio_config_t io_conf = {
      .intr_type = GPIO_INTR_DISABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = (1 << GPIO_HANDSHAKE)};
  gpio_config(&io_conf);
  //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
  gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

  gpio_pad_select_gpio(4);
  gpio_set_direction(4, GPIO_MODE_OUTPUT);

  //Use SPI HOST 2
  ret = spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, DMA_CHANNEL);
  return ret;
}
/******************************************/
//SPI TASK
/******************************************/
void spi_task(void* pvParams)
{

  QueueHandle_t dataQueue = (QueueHandle_t)pvParams;
  //WORD_ALIGNED_ATTR char spiSendBuf[129] = "";
  //WORD_ALIGNED_ATTR char spiRecvBuf[129] = "";
  WORD_ALIGNED_ATTR uint8_t* spiSendBuf = (uint8_t*)heap_caps_malloc(256,MALLOC_CAP_DMA);
  WORD_ALIGNED_ATTR uint8_t* spiRecvBuf = (uint8_t*)heap_caps_malloc(256,MALLOC_CAP_DMA);  
  uint8_t sdBuffer[512] = {0};
  uint16_t sdBuffIdx = 0;
  spi_slave_transaction_t t;
  memset(&t, 0, sizeof(t));
  while (1)
  {
    //memset(spiRecvBuf, 0x21, 256);
    //memset(spiSendBuf, 0x21, 256);
    //sprintf(spiSendBuf, "This is the receiver, sending data for transmission number");
    t.length = 256*8;
    t.tx_buffer = spiSendBuf;
    t.rx_buffer = spiRecvBuf;
    spi_slave_queue_trans(HSPI_HOST, &t, portMAX_DELAY);
    esp_err_t ret = spi_slave_get_trans_result(HSPI_HOST, &t, portMAX_DELAY);
    assert(ret == ESP_OK);
    gpio_set_level(4,0);
    //printf("Received %u bytes: %x %x %x %x \n", t.trans_len / 8, spiRecvBuf[0],spiRecvBuf[1],spiRecvBuf[2],spiRecvBuf[3]);
    //ESP_LOGI(TAG,"received %d bytes",t.trans_len/8);
    //Make sure we ahve real data otherwise we get hard fault.
    if((t.trans_len/8) > (8*5)){
      
      gpio_set_level(4,1);
      //last byte tells which node. for now just ignore the last byte.
      //Empty the SPI rx buffer to a different buffer.
      memcpy(&sdBuffer[sdBuffIdx],spiRecvBuf,(t.trans_len / 8)-1);
      sdBuffIdx += ((t.trans_len / 8)-1);
      uint8_t* ptr = &dataBuffer[dataBufferIdx];
      //Send data to processing task through queue. 
      gpio_set_level(4,0);

      memcpy(ptr,spiRecvBuf,245);
      gpio_set_level(4,1);
      BaseType_t err =  xQueueSendToBack(dataQueue,&ptr,0);
      gpio_set_level(4,0);
      dataBufferIdx = (dataBufferIdx+245)%(10*245);
      //printf("buf idx %d\n",dataBufferIdx);
      if(err != pdTRUE){ 
        ESP_LOGW(TAG,"processing queue full!");
        //Should write this to the sd card log.
      }

      gpio_set_level(4,1);
      //Wait till we have around 512 bytes, since the writes take aprox the same amount of time, but half as frequent.
      //ESP_LOGI(TAG,"sd buffer index:%d",sdBuffIdx);
      if(sdBuffIdx >= 480){
        sd_write_buf(sdBuffer, sdBuffIdx);
        sdBuffIdx = 0;
      }

      gpio_set_level(4,0);
    }
    
  }
}
/******************************************/
// UART
/******************************************/

void init_uart()
{
  uart_config_t uart_config = {
      .baud_rate = ECHO_UART_BAUD_RATE,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };
  int intr_alloc_flags = 0;
  ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE_UART * 2, 0, 0, NULL, intr_alloc_flags));
  ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
}
/******************************************/
// UART TASK
/******************************************/
void uart_task(void *arg)
{
  
  char uartRecvBuf[255] = "";
  uint8_t uartRxIdx = 0;
  uint8_t rxLen = 0;
  GatewayUartPacket packet;
  // packet.command = AVG_FORCE_DATA;
  // packet.data.int16[0] = tx_count;
  // packet.len = sizeof(int16_t)*1;//sending one int16_t value.

  // uint8_t bytesToSend = PreparePacket((uint8_t*)uartSendBuf,&packet);
  while (1)
  {
    //Read one bytes at a time
    uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t*)(&uartRecvBuf[uartRxIdx]), 1, portMAX_DELAY);
    ESP_LOGI(TAG,"read Uart Byte");
    if(uartRxIdx == 1){
    //This is always the len byte.
        rxLen = uartRecvBuf[uartRxIdx];
    }
    uartRxIdx++;

    //If count = len  then we have a full command.
    //If uartRxIdx is 0 or 1 we don't have a valid length yet.
    if((uartRxIdx >= rxLen) && uartRxIdx>1){
        uartRxIdx = 0;
        rxLen = 0;

        
        GetPacket((uint8_t*)uartRecvBuf,&packet);
        handleCommand(packet);
    }

  
    // sprintf(uartSendBuf, "%d Testing UART\n",tx_count);
    // printf("****\n");
    // printf("Transmitting %d bytes: %s \n", sizeof(uartSendBuf), uartSendBuf);
    // uart_write_bytes(ECHO_UART_PORT_NUM, uartSendBuf, bytesToSend);
    
    // printf("Received: %s\n", uartRecvBuf);
    // printf("****\n");
    // tx_count++;

    // packet.data.int16[0] = tx_count;
    // bytesToSend = PreparePacket((uint8_t*)uartSendBuf,&packet);

    // vTaskDelay(pdMS_TO_TICKS(60000));//Every 2 minutes
  }
}

int8_t handleCommand(GatewayUartPacket packet){

  ESP_LOGI(TAG,"Handling uart command.");

  switch(packet.command){

    case BLE_CONNECTION_EVENT:{
                                uint8_t con = packet.data.uint8[0];
                                uint8_t node_id = packet.data.uint8[1];
                                ESP_LOGI(TAG,"Received uart command: BLE Connection conn:%d, node:%d",con,node_id);

                                FILE *f = fopen(MOUNT_POINT "/log.txt", "a");
                                fprintf(f,"%d BLE_CONNECTION_EVENT: connected(%d), node_id(%d)\n",xTaskGetTickCount(),con,node_id);
                                fclose(f);
                                
                                break;
    }

    case TIME_UPDATE:{
                        uint8_t month = packet.data.uint8[0];
                        uint8_t day = packet.data.uint8[1];
                        uint8_t hour = packet.data.uint8[2];
                        uint8_t minute = packet.data.uint8[3];
                        uint8_t second = packet.data.uint8[4];
                        ESP_LOGI(TAG,"TIME_UPDATE: /%d/%d %d:%d:%d",month,day,hour,minute,second);
                        
                        FILE *f = fopen(MOUNT_POINT "/log.txt", "a");
                        fprintf(f,"%d TIME_UPDATE: /%d/%d %d:%d:%d\n",xTaskGetTickCount(),month,day,hour,minute,second);
                        fclose(f);
                        
                        break;
    }

    case LTE_RSSI_DATA:{
                        
                        float signalStrength = packet.data.float32[0];
                        ESP_LOGI(TAG,"LTE_RSSI_DATA: %f",signalStrength);

                        FILE *f = fopen(MOUNT_POINT "/log.txt", "a");
                        ESP_LOGI(TAG,"Log file opened.");
                        fprintf(f,"%d LTE_RSSI_DATA: %f%%\n",xTaskGetTickCount(),signalStrength);
                        ESP_LOGI(TAG,"Log file written.");
                        fclose(f);
                        ESP_LOGI(TAG,"Log file closed.");
                        break;
    }

    default:  ESP_LOGI(TAG,"Received unknown UART command.");
              return -1;
              break;
  }
  return 0;

}