/* modem.h
 *  Contains functions related to communicating with the cellular/BLE modem (i.e . Particle Boron).
 */

#ifndef MODEM_H_
#define MODEM_H_

#include <esp_err.h>


#define USE_HSPI 1

#ifdef USE_VSPI
#define GPIO_HANDSHAKE 27
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 26
#elif defined USE_HSPI
#define GPIO_HANDSHAKE 27
#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15
#endif
#define DMA_CHANNEL 1

//UART
#define ECHO_TEST_TXD 25
#define ECHO_TEST_RXD 33
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_UART_PORT_NUM 2
#define ECHO_UART_BAUD_RATE 115200
#define BUF_SIZE_UART 1024


esp_err_t init_spi_slave();

void init_uart();

void spi_task(void* pvParams);

void uart_task(void *arg);

void telemTask(void* pvParams);

extern float threshold; 

#endif