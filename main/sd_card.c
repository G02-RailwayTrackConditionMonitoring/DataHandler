#include "sd_card.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/sdmmc_host.h"

static const char* TAG= "SD_CARD";

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
      return ret;
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
// SD Benchmark
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
// SD Write
/******************************************/
void sd_write_buf(uint8_t buf[], size_t len, FILE *f)
{
  
  int64_t m;
  int64_t m2;
  m = esp_timer_get_time();
  // printf("writing %u bytes to SD\n", len);
  //printf("%s \n", buf);
  // FILE *f = fopen(MOUNT_POINT "/testing.txt", "a");
  if (f == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file for writing");
    return;
  }
  m2 = esp_timer_get_time();
  size_t ret = fwrite(buf, 1, len, f);
  m2 = esp_timer_get_time() - m2;
  //printf("t_write %lld, %zu bytes \n", m2, ret);
  //fclose(f);
  m = esp_timer_get_time() - m;
  //printf("t_openfile - t_closefile %lld \n", m);
}
/******************************************/