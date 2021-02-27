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

#include "sd_card.h"
#include "modem.h"
#include "dataProcessing.h"

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



//BLINK
#define PIN 32
//34 can only be input

QueueHandle_t dataQueue;
static const char* TAG = "CONFIG";

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

  // //Testing the remaining 4MB of RAM 
  // size_t memcnt=esp_himem_get_phys_size();
  // size_t memfree=esp_himem_get_free_size();
  // printf("Himem has %dKiB of memory, %dKiB of which is free. Testing the free memory...\n", (int)memcnt/1024, (int)memfree/1024);
  // assert(test_region(memfree, 0xaaaa));
  // printf("Done!\n");


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

  FILE *f = fopen(MOUNT_POINT "/log.txt", "a");
  fprintf(f,"%d ESP32_POWER_ON \n",xTaskGetTickCount());
  fclose(f);

  dataQueue = xQueueCreate(20,sizeof(uint8_t*));//10 items, each a pointer to a buffer.
  //xTaskCreate(&blinky, "blink-led", 2048, NULL, 2, NULL);
  xTaskCreate(&spi_task, "spi-receive", 2600, (void*)dataQueue, 4, NULL); //SD Write is daisy chained to this. Should be highest priority.
  xTaskCreate(&uart_task, "uart-receive", 4096,NULL, 3, NULL);
  xTaskCreate(&processingTask,"processing",32768*2,(void*)dataQueue,2,NULL);
  xTaskCreate(&telemTask,"Telem task",2048,NULL,2,NULL);


  //xTaskCreate(&sd_benchmark, "sd-write", 2048, NULL, 2, NULL);
}
