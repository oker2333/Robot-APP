#include <stdio.h>
#include "ota.h"
#include "crc.h"

typedef void (*Jump_To_ADDR_t)(void);

void Write_Jump_Addr(uint32_t Jump_Addr)
{
	 flash_write_buffer(JUMP_ADDR_ADDRESS, (uint8_t*)&Jump_Addr,sizeof(uint32_t));
}

void Write_Boot_Ver(uint16_t Boot_Ver)
{
	 flash_write_buffer(BOOT_VERSION_ADDRESS, (uint8_t*)&Boot_Ver,sizeof(uint16_t));
}

static uint32_t app_download_address = APP_ADDRESS_B;
static uint32_t boot_download_address = IAP_ADDRESS;
static uint32_t download_address = 0x00;

void download_address_update(void)
{
	  uint32_t jump_address = 0x00;
		flash_read_buffer(JUMP_ADDR_ADDRESS,(uint8_t*)&jump_address,sizeof(uint32_t));
	  if(jump_address == APP_ADDRESS_A)
		{
			 app_download_address = APP_ADDRESS_B;
		}else if(jump_address == APP_ADDRESS_B)
		{
			 app_download_address = APP_ADDRESS_A;
		}
		printf("app download address = 0x%x\n",app_download_address);
}


uint32_t Download2Flash(int8_t OTA_Device,uint32_t OTA_Offset,uint8_t *OTA_Data,uint32_t OTA_Length)
{
	 static int Frame_ID = 0;
	 static uint16_t OTA_Count = 0x00;
	 static uint8_t OTA_Page_Buffer[FMC_PAGE_SIZE];
	
	 if(OTA_Device == BOOT)
	 {
		  download_address = boot_download_address;
	 }else if(OTA_Device == APP)
	 {
		  download_address = app_download_address;
	 }else
	 {
		  return 0;
	 }
	
	 Mem_Copy(&OTA_Page_Buffer[OTA_Offset%FMC_PAGE_SIZE],OTA_Data,OTA_Length);
	 OTA_Count += OTA_Length;
	
	 if((OTA_Count % FMC_PAGE_SIZE) == 0)
	 {
		  uint32_t Cur_Download_Addr = Frame_ID * FMC_PAGE_SIZE + download_address;
		  
		  flash_write_buffer(Cur_Download_Addr, OTA_Page_Buffer,OTA_Count);
		  printf("[frame id %d]Current Download Address 0x%x,OTA_Count = %d\r\n",Frame_ID,Cur_Download_Addr,OTA_Count);
		  
		  Frame_ID++;
		  OTA_Count = 0x00;
	 }
	 else if(OTA_Length < 128)
	 {
		  uint32_t Cur_Download_Addr = Frame_ID * FMC_PAGE_SIZE + download_address;
		  
		  flash_write_buffer(Cur_Download_Addr, OTA_Page_Buffer,OTA_Count);
		  printf("[frame id %d]Current Download Address 0x%x,OTA_Count = %d\r\n",Frame_ID,Cur_Download_Addr,OTA_Count);
		  OTA_Count = 0x00;
		  Frame_ID = 0x00;
	 }
	 return (OTA_Offset + OTA_Length);
}

uint8_t FlashBinaryCheck(int8_t OTA_Device,uint16_t Binary_Version,uint16_t CRC16,uint32_t OTA_Rev_Bytes)
{
	 uint16_t FileCRCActual = CRC16_CCITT_FALSE(((uint8_t*)download_address), OTA_Rev_Bytes);
	 printf("FileSize = %d Bytes,FileCRC = 0x%x,FileCRCActual = 0x%x\r\n",OTA_Rev_Bytes,CRC16,FileCRCActual);
	 
	 if(CRC16 == FileCRCActual)
	 {
		  if(OTA_Device == BOOT)
			{
				 Write_Boot_Ver(Binary_Version);
			}
			else if(OTA_Device == APP)
			{
				 Write_Jump_Addr(download_address);
			}
		  return 0;
	 }
	 return 1;
}
