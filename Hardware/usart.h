#ifndef USART_H
#define USART_H

#include <stdint.h>

#define RX_BUFFER_LEN 1024

#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define BAUDRATE	115200U 
#define USART0_DATA_ADDRESS ((uint32_t)&USART_DATA(USART0))

void bsp_usart_init(uint32_t baudval);
void usart0_dma_send(uint8_t *buffer,uint16_t len);
uint16_t usart0_dma_recv(uint8_t *buffer);

#endif
