#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "driver/sdmmc_host.h"
#include "esp_timer.h"

#define MOUNT_POINT "/sdcard"
#define PIN 5
#define TAG "CONFIG"

const size_t BUF_SIZE = 512;
uint8_t buf[512];
int64_t totalLatency = 0;
int64_t minLatency = 9999999;
int64_t maxLatency = 0;
int64_t avgLatency = 0;
int64_t m; 
static int32_t num_writes = 0;
/******************************************/
// BLINKY
/******************************************/
void blinky(void *params)
{
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
    //ESP_LOGI(TAG, "%lld", esp_timer_get_time()-m);
  }
}
/******************************************/

/******************************************/
// CONFIG
/******************************************/
void config(void *params)
{
  ESP_LOGI(TAG, "MY_INT %d", CONFIG_MY_INT);
  ESP_LOGI(TAG, "MY_STRING %s", CONFIG_MY_STRING);

  bool my_bool = false;
#ifdef CONFIG_MY_BOOL
  my_bool = true;
#else
  my_bool = false;
#endif

  ESP_LOGI(TAG, "MY_BOOL %s", my_bool ? "yes" : "no");

  int option = 0;

#ifdef CONFIG_OPTION_1
  option = 1;
#elif CONFIG_OPTION_2
  option = 2;
#else
  option = 3;
#endif

  ESP_LOGI(TAG, "MY_OPTION %d", option);
  while (true)
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
/******************************************/

/******************************************/
// SD INIT
/******************************************/
void sd_init()
{
  esp_err_t ret;
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = false,
      .max_files = 5,
      .allocation_unit_size = 16 * 1024};
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
  if (ret != ESP_OK)
  {
    if (ret == ESP_FAIL)
    {
      ESP_LOGE(TAG, "Failed to mount filesystem. "
                    "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
      gpio_set_level(PIN, 1);
      while (1)
        ;
    }
    else
    {
      ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                    "Make sure SD card lines have pull-up resistors in place.",
               esp_err_to_name(ret));
      gpio_set_level(PIN, 1);
      while (1)
        ;
    }
    return;
  }
  return;
}
/******************************************/

/******************************************/
// CREATE RANDOM BUFFER FOR BENCHMARK
/******************************************/
void create_buffer()
{
  for (size_t i = 0; i < (BUF_SIZE - 2); i++)
  {
    buf[i] = 'A' + (i % 26);
  }
  buf[BUF_SIZE - 2] = '\r';
}
/******************************************/
// SD WRITE
/******************************************/
void sd_write()
{

  while (true)
  {
    //ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen(MOUNT_POINT "/testing.txt", "a");
    if (f == NULL)
    {
      ESP_LOGE(TAG, "Failed to open file for writing");
      return;
    }
    //m = esp_timer_get_time();
    size_t check = fwrite(buf, 1, BUF_SIZE, f);
    //m = esp_timer_get_time() - m;
    
    //ESP_LOGI(TAG, "%d", check);
    num_writes ++;
    
    totalLatency = totalLatency +  m;
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
    
    ESP_LOGI(TAG, "File written");
    ESP_LOGI(TAG, "max latency %lld", maxLatency);
    ESP_LOGI(TAG, "min latency %lld", minLatency);
    ESP_LOGI(TAG, "avg latency %lld", avgLatency);
    ESP_LOGI(TAG, "total latency %lld", totalLatency);
    ESP_LOGI(TAG, "num_writes %d", num_writes);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
/******************************************/

void app_main(void)
{
  printf("Hello world!\n");
  sd_init();
  create_buffer();
  xTaskCreate(&blinky, "blink led", 2048, NULL, 2, NULL);
  //xTaskCreate(&config, "config demo", 2048, NULL, 2, NULL);
  xTaskCreate(&sd_write, "sd write", 2048, NULL, 2, NULL);
}
