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
#include <string.h>
#include <esp_vfs.h>


static const char* TAG= "SD_CARD";

FILE* datFile0;
FILE* datFile1;
uint16_t datFile0Count;
uint16_t datFile1Count;
char datFile0Path[30];
char datFile1Path[30];

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
  // FILE *f = fopen(MOUNT_POINT "/testing.txt", "w");

  uint32_t runIndex =0;
  //Open the index file, this contains and integer that increments every restart.
  FILE *indexFile = fopen(MOUNT_POINT "/index.txt","r+");
  if(indexFile ==NULL){
    //If file doesn't exist we try to create it.
    ESP_LOGI(TAG,"Could not open index.txt");
    indexFile = fopen(MOUNT_POINT "/index.txt","w");

    if(indexFile == NULL){
      //If we cant create the file the there is somethign wrong.
      ESP_LOGW(TAG,"Could not create index.txt!");
    }else{
      fputs("1",indexFile);
      fclose(indexFile);
    }
  }else{

    char buf[5];
    fgets(buf,5,indexFile);
    runIndex = atoi(buf);
    
    //Now add one and write to file.
    fseek(indexFile,0,SEEK_SET);
    char buf2[5];
    snprintf(buf2,5,"%d",runIndex+1);
    fputs(buf2,indexFile);
    fclose(indexFile);

  }
  ESP_LOGI(TAG,"ESP32 run index:%d",runIndex);
  
  //Also write the run index to the log file.
  FILE *f = fopen(MOUNT_POINT "/log.txt", "a");
  if(f != NULL){
    fprintf(f,"%d %s:%d\n",xTaskGetTickCount(),"ESP32_RUN_INDEX",runIndex);
    fclose(f);
  }


  //Open the data files use runIndex in file name, so that we get a unique file each time system is run or restarts.
  char buf[30];
  sprintf(datFile0Path,"%s/n%d_r%d.dat",MOUNT_POINT,0,runIndex);
  datFile0 = fopen(datFile0Path,"a");
  if(datFile0 == NULL){
    ESP_LOGW(TAG,"Could not open %s",datFile0Path);
    //Can't really do anything about this?
  }

  //For the second node...
  // sprintf(datFile1Path,"%s/n%d_r%d.dat",MOUNT_POINT,1,runIndex);
  // datFile1 = fopen(datFile1Path,"a");
  // if(datFile1 == NULL){
  //   ESP_LOGW(TAG,"Could not open %s",datFile1Path);
  //   //Can't really do anything about this?
  // }



  // fclose(f);
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

void sd_write_buf(uint8_t buf[], size_t len, uint8_t node_id)
{
  
  if(node_id == 0){
    fwrite(buf, 1, len,datFile0 );
    datFile0Count++;
    
    //About 1 seconds worth of data is 25 counts. (480 bytes ->80 sample -> 40 ms * 25 = 1 sec)
    if(datFile0Count > 25){
       fclose(datFile0);
       datFile0 = fopen(datFile0Path,"a");
      datFile0Count =0;
      //ESP_LOGI(TAG,"Syncing datFile0");
    }
  }
  else if(node_id == 1){
    fwrite(buf, 1, len,datFile1 );
    datFile1Count++;
    
    if(datFile1Count > 25){
      fclose(datFile1);
      datFile0 = fopen(datFile1Path,"a");
      datFile1Count =0;
    }
  }
 
  
}
/******************************************/