#include "ota.h"
#include "crc.h"

typedef void (*Jump_To_ADDR_t)(void);

uint32_t APP_Download_Address(void)
{
	uint32_t download_addr = 0x00;
	uint32_t cur_addr = flash_read_word(APP_ADDR_ADDRESS);
	if(cur_addr == APP_A_ADDRESS)
	{
		 download_addr = APP_B_ADDRESS;
	}
	else if(cur_addr == APP_B_ADDRESS)
	{
		 download_addr = APP_A_ADDRESS;
	}
	else
	{
		 download_addr = APP_A_ADDRESS;
	}
	return download_addr;
}

void Write_APP_Address(void)
{
	 uint32_t jump_address = APP_Download_Address();
	 flash_write_buffer(APP_ADDR_ADDRESS, (uint8_t*)&jump_address,sizeof(uint32_t));
}

static uint8_t OTA_Page_Buffer[FMC_PAGE_SIZE];
static uint16_t OTA_Count = 0x00;

uint32_t Download2Flash(uint32_t OTA_Offset,uint8_t *OTA_Data,uint32_t OTA_Length)
{
	 Mem_Copy(OTA_Page_Buffer[OTA_Offset%FMC_PAGE_SIZE],OTA_Data,OTA_Length);
	 OTA_Count += OTA_Length;
	 if((OTA_Count == FMC_PAGE_SIZE) || (OTA_Length < 256))
	 {
		  uint32_t download_address = APP_Download_Address();
		  flash_write_buffer(download_address + OTA_Offset, OTA_Page_Buffer,OTA_Count);
		  OTA_Count = 0x00;
	 }
	 return (OTA_Offset + OTA_Length);
}

uint8_t FlashBinaryCheck(uint16_t CRC16,uint32_t OTA_Rev_Bytes)
{
	 uint32_t download_address = APP_Download_Address();
	 if(CRC16 == CRC16_CCITT_FALSE(((uint8_t*)download_address), OTA_Rev_Bytes))
	 {
		  Write_APP_Address();
		  return 0;
	 }
	 return 1;
}
