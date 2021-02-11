#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "driver/gpio.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/sdmmc_host.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <sys/stat.h>
#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp32/himem.h"


#ifdef CONFIG_IDF_TARGET_ESP32
//#define RCV_HOST    HSPI_HOST
//#define DMA_CHAN    2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    RCV_HOST

#elif defined CONFIG_IDF_TARGET_ESP32C3
#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    RCV_HOST
#endif


//SPI

#define HSPI 1
//#define VSPI 2
#define SPI_HOST HSPI

#ifdef VSPI
#define GPIO_HANDSHAKE 27
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 26
#elif defined HSPI
#define GPIO_HANDSHAKE 27
#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15
#endif
#define DMA_CHANNEL 1

//SD
#define MOUNT_POINT "/sdcard"
#define SPI_DMA_CHAN    2
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCLK 18
#define SD_CS 26
//BLINK
#define PIN 32
//34 can only be input

//UART
#define ECHO_TEST_TXD 25
#define ECHO_TEST_RXD 33
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_UART_PORT_NUM 2
#define ECHO_UART_BAUD_RATE 115200
#define BUF_SIZE_UART 1024

#define TAG "CONFIG"

/******************************************/
// BLINKY
/******************************************/
void blinky(void *params)
{
  int64_t m;
  gpio_pad_select_gpio(PIN);
  gpio_set_direction(PIN, GPIO_MODE_OUTPUT);
  int isOn = 0;
  m = esp_timer_get_time();
  ESP_LOGI(TAG, "am i being called ? ");
  while (true)
  {
    isOn = !isOn;
    gpio_set_level(PIN, isOn);
    m = esp_timer_get_time();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI("TICK", "%lld", esp_timer_get_time() - m);
  }
}

/******************************************/
// SDIO INIT
/******************************************/
esp_err_t init_sdio()
{
  esp_err_t ret;
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 5,
      .allocation_unit_size = 16 * 1024
      };
  sdmmc_card_t *card;
  const char mount_point[] = MOUNT_POINT;
  ESP_LOGI(TAG, "Initializing SD card");
  ESP_LOGI(TAG, "Using SDMMC peripheral");
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  gpio_set_pull_mode(15, GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1- line modes
  gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);  // D0, needed in 4- and 1-line modes
  gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);  // D1, needed in 4-line mode only
  gpio_set_pull_mode(12, GPIO_PULLUP_ONLY); // D2, needed in 4-line mode only
  gpio_set_pull_mode(13, GPIO_PULLUP_ONLY); // D3, needed in 4- and 1-line modes
  ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
  return ret;
}

/******************************************/
// SD INIT
/******************************************/
esp_err_t init_sd()
{
  esp_err_t ret;
  sdmmc_card_t *card;
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 5,
      .allocation_unit_size = 16 * 1024
      };  
  const char mount_point[] = MOUNT_POINT;
  ESP_LOGI(TAG, "Using SPI peripheral");

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  spi_bus_config_t bus_cfg = {
      .mosi_io_num = SD_MOSI,
      .miso_io_num = SD_MISO,
      .sclk_io_num = SD_SCLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4000,
  };
host.slot =VSPI_HOST;

  ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
  if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize bus.");
      return;
  }

  // This initializes the slot without card detect (CD) and write protect (WP) signals.
  // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = SD_CS;
  slot_config.host_id = host.slot;

  ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
  
  //Open then close, in "w" mode, to reset the file on startup.
  FILE *f = fopen(MOUNT_POINT "/testing.txt", "w");
  fclose(f);
  return ret;
}
/******************************************/

/******************************************/
// SD WRITE
/******************************************/
void sd_benchmark()
{
  int64_t totalLatency = 0;
  int64_t minLatency = 9999999;
  int64_t maxLatency = 0;
  int64_t avgLatency = 0;
  int64_t m;
  int64_t m2;
  uint8_t buf[512];
  static int32_t num_writes = 0;
  for (size_t i = 0; i < (512 - 2); i++)
  {
    buf[i] = 'A' + (i % 26);
  }
  buf[512 - 2] = '\r';
  while (true)
  {
    FILE *f = fopen(MOUNT_POINT "/testing.txt", "w");
    if (f == NULL)
    {
      ESP_LOGE("SD", "Failed to open file for writing");
      return;
    }
    m = esp_timer_get_time();
    size_t check = fwrite(buf, 1, 512, f);
    m = esp_timer_get_time() - m;
    num_writes++;

    totalLatency = totalLatency + m;
    if (maxLatency < m)
    {
      maxLatency = m;
    }
    if (minLatency > m)
    {
      minLatency = m;
    }
    avgLatency = totalLatency / (num_writes);

    fclose(f);

    ESP_LOGI("SD", "File written");
    ESP_LOGI("SD", "max latency %lld", maxLatency);
    ESP_LOGI("SD", "min latency %lld", minLatency);
    ESP_LOGI("SD", "avg latency %lld", avgLatency);
    ESP_LOGI("SD", "total latency %lld", totalLatency);
    ESP_LOGI("SD", "num_writes %d", num_writes);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
/******************************************/
void sd_write_buf(uint8_t buf[], size_t len)
{
  int64_t m;
  int64_t m2;
  m = esp_timer_get_time();
  printf("writing %u bytes to SD\n", len);
  printf("%s \n", buf);
  FILE *f = fopen(MOUNT_POINT "/testing.txt", "a");
  if (f == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file for writing");
    return;
  }
  m2 = esp_timer_get_time();
  fwrite(buf, 1, len, f);
  m2 = esp_timer_get_time() - m2;
  printf("t_write %lld \n", m2);
  fclose(f);
  m = esp_timer_get_time() - m;
  printf("t_openfile - t_closefile %lld \n", m);
}

/******************************************/
// SPI
/******************************************/
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
  WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << GPIO_HANDSHAKE));
}
/******************************************/
void my_post_trans_cb(spi_slave_transaction_t *trans)
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
      .max_transfer_sz = 4094
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

  //Use SPI HOST 2
  ret = spi_slave_initialize(SPI_HOST, &buscfg, &slvcfg, DMA_CHANNEL);
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
    spi_slave_queue_trans(SPI_HOST, &t, portMAX_DELAY);
    esp_err_t ret = spi_slave_get_trans_result(SPI_HOST, &t, portMAX_DELAY);
    assert(ret == ESP_OK);
    printf("Received %u bytes: %s \n", t.trans_len / 8, spiRecvBuf);
    sd_write_buf(spiRecvBuf, t.trans_len / 8);
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
static void uart_task(void *arg)
{
  char uartSendBuf[33] = "";
  char uartRecvBuf[33] = "";
  while (1)
  {
    sprintf(uartSendBuf, "Testing UART\n");
    printf("****\n");
    printf("Transmitting %d bytes: %s \n", sizeof(uartSendBuf), uartSendBuf);
    uart_write_bytes(ECHO_UART_PORT_NUM, uartSendBuf, sizeof(uartSendBuf));
    uart_read_bytes(ECHO_UART_PORT_NUM, (uint8_t*)uartRecvBuf, 32, 20 / portTICK_RATE_MS);
    printf("Received: %s\n", uartRecvBuf);
    printf("****\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

/******************************************/
// HIMEM EXAMPLE CODE - remaining 4MB of PSRAM 
/******************************************/

//Fill memory with pseudo-random data generated from the given seed.
//Fills the memory in 32-bit words for speed.
static void fill_mem_seed(int seed, void *mem, int len)
{
    uint32_t *p = (uint32_t *)mem;
    unsigned int rseed = seed ^ 0xa5a5a5a5;
    for (int i = 0; i < len / 4; i++) {
        *p++ = rand_r(&rseed);
    }
}

//Check the memory filled by fill_mem_seed. Returns true if the data matches the data
//that fill_mem_seed wrote (when given the same seed).
//Returns true if there's a match, false when the region differs from what should be there.
static bool check_mem_seed(int seed, void *mem, int len, int phys_addr)
{
    uint32_t *p = (uint32_t *)mem;
    unsigned int rseed = seed ^ 0xa5a5a5a5;
    for (int i = 0; i < len / 4; i++) {
        uint32_t ex = rand_r(&rseed);
        if (ex != *p) {
            printf("check_mem_seed: %x has 0x%08x expected 0x%08x\n", phys_addr+((char*)p-(char*)mem), *p, ex);
            return false;
        }
        p++;
    }
    return true;
}

//Allocate a himem region, fill it with data, check it and release it.
static bool test_region(int check_size, int seed)
{
    esp_himem_handle_t mh; //Handle for the address space we're using
    esp_himem_rangehandle_t rh; //Handle for the actual RAM.
    bool ret = true;

    //Allocate the memory we're going to check.
    ESP_ERROR_CHECK(esp_himem_alloc(check_size, &mh));
    //Allocate a block of address range
    ESP_ERROR_CHECK(esp_himem_alloc_map_range(ESP_HIMEM_BLKSZ, &rh));
    for (int i = 0; i < check_size; i += ESP_HIMEM_BLKSZ) {
        uint32_t *ptr = NULL;
        //Map in block, write pseudo-random data, unmap block.
        ESP_ERROR_CHECK(esp_himem_map(mh, rh, i, 0, ESP_HIMEM_BLKSZ, 0, (void**)&ptr));
        fill_mem_seed(i ^ seed, ptr, ESP_HIMEM_BLKSZ); //
        ESP_ERROR_CHECK(esp_himem_unmap(rh, ptr, ESP_HIMEM_BLKSZ));
    }
    vTaskDelay(5); //give the OS some time to do things so the task watchdog doesn't bark
    for (int i = 0; i < check_size; i += ESP_HIMEM_BLKSZ) {
        uint32_t *ptr;
        //Map in block, check against earlier written pseudo-random data, unmap block.
        ESP_ERROR_CHECK(esp_himem_map(mh, rh, i, 0, ESP_HIMEM_BLKSZ, 0, (void**)&ptr));
        if (!check_mem_seed(i ^ seed, ptr, ESP_HIMEM_BLKSZ, i)) {
            printf("Error in block %d\n", i / ESP_HIMEM_BLKSZ);
            ret = false;
        }
        ESP_ERROR_CHECK(esp_himem_unmap(rh, ptr, ESP_HIMEM_BLKSZ));
        if (!ret) break; //don't check rest of blocks if error occurred
    }
    //Okay, all done!
    ESP_ERROR_CHECK(esp_himem_free(mh));
    ESP_ERROR_CHECK(esp_himem_free_map_range(rh));
    return ret;
}

/******************************************/
//MAIN
/******************************************/

void app_main(void)
{

  //Testing the remaining 4MB of RAM 
  size_t memcnt=esp_himem_get_phys_size();
  size_t memfree=esp_himem_get_free_size();
  printf("Himem has %dKiB of memory, %dKiB of which is free. Testing the free memory...\n", (int)memcnt/1024, (int)memfree/1024);
  assert(test_region(memfree, 0xaaaa));
  printf("Done!\n");


  esp_err_t ret;

  init_uart(); //error checking happens in here..

  if (init_spi_slave() != ESP_OK)
  {
    printf("ERROR INITIALIZING SPI");
    while (1)
    {
    };
  };
  if (init_sd() != ESP_OK)
  {
    printf("ERROR INITIALIZING SD");
    while (1)
    {
    };
  };

  xTaskCreate(&blinky, "blink-led", 2048, NULL, 2, NULL);
  xTaskCreate(&spi_task, "spi-receive", 2048, NULL, 2, NULL); //SD Write is daisy chained to this
  xTaskCreate(&uart_task, "uart-transmit", 2048, NULL, 3, NULL);

  //xTaskCreate(&sd_benchmark, "sd-write", 2048, NULL, 2, NULL);
}
