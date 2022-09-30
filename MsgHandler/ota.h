#ifndef IAP_H
#define IAP_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "flash_rw.h"
#include "app_config.h"

typedef void (*Jump_To_ADDR_t)(void);

uint32_t APP_Download_Address(void);
void Write_APP_Address(void);

#endif
