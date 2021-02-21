#include "modem.h"

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/uart.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include <string.h>


#include "sd_card.h"
#include "GatewayCommands.h"

static const char* TAG = "MODEM";

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
void spi_task()
{
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
    memset(spiRecvBuf, 0x21, 256);
    memset(spiSendBuf, 0x21, 256);
    //sprintf(spiSendBuf, "This is the receiver, sending data for transmission number");
    t.length = 256*8;
    t.tx_buffer = spiSendBuf;
    t.rx_buffer = spiRecvBuf;
    spi_slave_queue_trans(HSPI_HOST, &t, portMAX_DELAY);
    esp_err_t ret = spi_slave_get_trans_result(HSPI_HOST, &t, portMAX_DELAY);
    assert(ret == ESP_OK);
    gpio_set_level(4,1);
    printf("Received %u bytes: %x %x %x %x \n", t.trans_len / 8, spiRecvBuf[0],spiRecvBuf[1],spiRecvBuf[2],spiRecvBuf[3]);

    //last byte tells which node. for now just ignore the last byte.
    //Empty the SPI rx buffer to a different buffer.
    memcpy(&sdBuffer[sdBuffIdx],spiRecvBuf,(t.trans_len / 8)-1);
    sdBuffIdx += ((t.trans_len / 8)-1);

    //Wait till we have around 512 bytes, since the writes take aprox the same amount of time, but half as frequent.
    ESP_LOGI(TAG,"sd buffer index:%d",sdBuffIdx);
    if(sdBuffIdx >= 480){
      sd_write_buf(sdBuffer, sdBuffIdx);
      sdBuffIdx = 0;
    }

    gpio_set_level(4,0);
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
  char uartSendBuf[33] = "";
  char uartRecvBuf[33] = "";
  int16_t tx_count =-10;
  
  GatewayUartPacket packet;
  packet.command = AVG_FORCE_DATA;
  packet.data.int16[0] = tx_count;
  packet.len = sizeof(int16_t)*1;//sending one int16_t value.

  uint8_t bytesToSend = PreparePacket((uint8_t*)uartSendBuf,&packet);
  while (1)
  {
    // sprintf(uartSendBuf, "%d Testing UART\n",tx_count);
    // printf("****\n");
    printf("Transmitting %d bytes: %s \n", sizeof(uartSendBuf), uartSendBuf);
    uart_write_bytes(ECHO_UART_PORT_NUM, uartSendBuf, bytesToSend);
    uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t*)uartRecvBuf, 32, 20 / portTICK_RATE_MS);
    // printf("Received: %s\n", uartRecvBuf);
    // printf("****\n");
    tx_count++;

    packet.data.int16[0] = tx_count;
    bytesToSend = PreparePacket((uint8_t*)uartSendBuf,&packet);

    vTaskDelay(pdMS_TO_TICKS(12000));//Every 2 minutes
  }
}