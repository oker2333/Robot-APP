#ifndef IAP_H
#define IAP_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "flash_rw.h"
#include "app_config.h"

#define BOOT 0
#define APP  1

void download_address_update(void);

uint32_t Download2Flash(int8_t OTA_Device,uint32_t OTA_Offset,uint8_t *OTA_Data,uint32_t OTA_Length);
uint8_t FlashBinaryCheck(int8_t OTA_Device,uint16_t Binary_Version,uint16_t CRC16,uint32_t OTA_Rev_Bytes);

#endif
