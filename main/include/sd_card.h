/* sd_card.h
 *  Contains functions related to using the sd card.
 */
#ifndef SD_CARD_H_
#define SD_CARD_H_

#include <stdint.h>
#include <esp_err.h>

//Pinout 
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCLK 18
#define SD_CS 26

#define MOUNT_POINT "/sdcard"

#define SPI_DMA_CHAN    2


//Initializes the SDIO interface and mounts the file system.
esp_err_t init_sdio();

//Initializes the SD interface and mounts the file system.
esp_err_t init_sd();

//Function that benchmarks the sd card performance.
void sd_benchmark();

//Writes data to the sd card.
void sd_write_buf(uint8_t buf[], size_t len, FILE *f);


#endif