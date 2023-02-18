#ifndef FLASH_RW_H
#define FLASH_RW_H

#include <stdbool.h>
#include <stdint.h>
#include "gd32f30x_fmc.h"

void flash_erase_pages(uint32_t address,uint32_t size);

void flash_read_buffer(uint32_t address, void *data,uint32_t data_len);
uint32_t flash_write_buffer(uint32_t address, uint8_t *data,uint32_t data_len);

uint32_t flash_read_word(uint32_t address);
uint16_t flash_read_dword(uint32_t address);
uint8_t flash_read_byte(uint32_t address);

#endif
