#ifndef IAP_H
#define IAP_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "flash_rw.h"
#include "ymodem.h"
#include "app_config.h"

#define IAP_FLAG_ADDR ((uint32_t)(0x0803F800U))

#define APPRUN_FLAG_DATA      0x5A5A   //APP不需要做任何处理，直接运行状态
#define UPDATE_FLAG_DATA      0xEEEE   //下载标志的数据
#define UPLOAD_FLAG_DATA      0xDDDD   //上传标志的数据

typedef void (*Jump_To_ADDR_t)(void);


extern char FileName[FILE_NAME_LENGTH];

uint32_t IAP_ReadFlag(void);
void IAP_WriteFlag(uint32_t flag);

uint32_t APP_Download_Address(void);
void Write_APP_Address(void);

void set_download_status(int32_t status);
int32_t get_download_status(void);

uint32_t APP_Jump_Address(void);

int32_t Download2Flash(void);

int32_t Jump_to_APP(void);

#endif
