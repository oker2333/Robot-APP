#ifndef IAP_H
#define IAP_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "flash_rw.h"
#include "app_config.h"

void download_address_update(void);

uint32_t Download2Flash(uint32_t OTA_Offset,uint8_t *OTA_Data,uint32_t OTA_Length);
uint8_t FlashBinaryCheck(uint16_t CRC16,uint32_t OTA_Rev_Bytes);

#endif
