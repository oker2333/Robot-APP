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

uint32_t Download2Flash(uint32_t OTA_Offset,uint8_t *OTA_Data,uint32_t OTA_Length)
{
	 static int Frame_ID = 0;
	 static uint16_t OTA_Count = 0x00;
	 static uint8_t OTA_Page_Buffer[FMC_PAGE_SIZE];
	
	 Mem_Copy(&OTA_Page_Buffer[OTA_Offset%FMC_PAGE_SIZE],OTA_Data,OTA_Length);
	 OTA_Count += OTA_Length;
	
	 if((OTA_Count % FMC_PAGE_SIZE) == 0)
	 {
		  uint32_t Base_Download_Addr = APP_Download_Address();
		  uint32_t Cur_Download_Addr = Frame_ID * FMC_PAGE_SIZE + Base_Download_Addr;
		  
		  flash_write_buffer(Cur_Download_Addr, OTA_Page_Buffer,OTA_Count);
		  printf("[frame id %d]Current Download Address 0x%x,OTA_Count = %d\r\n",Frame_ID,Cur_Download_Addr,OTA_Count);
		  
		  Frame_ID++;
		  OTA_Count = 0x00;
	 }
	 else if(OTA_Length < 128)
	 {
		  uint32_t Base_Download_Addr = APP_Download_Address();
		  uint32_t Cur_Download_Addr = Frame_ID * FMC_PAGE_SIZE + Base_Download_Addr;
		  
		  flash_write_buffer(Cur_Download_Addr, OTA_Page_Buffer,OTA_Count);
		  printf("[frame id %d]Current Download Address 0x%x,OTA_Count = %d\r\n",Frame_ID,Cur_Download_Addr,OTA_Count);
		  OTA_Count = 0x00;
		  Frame_ID = 0x00;
	 }
	 return (OTA_Offset + OTA_Length);
}

uint8_t FlashBinaryCheck(uint16_t CRC16,uint32_t OTA_Rev_Bytes)
{
	 uint32_t download_address = APP_Download_Address();
	 uint16_t FileCRCActual = CRC16_CCITT_FALSE(((uint8_t*)download_address), OTA_Rev_Bytes);
	 printf("FileSize = %d Bytes,FileCRC = 0x%x,FileCRCActual = 0x%x\r\n",OTA_Rev_Bytes,CRC16,FileCRCActual);
	 
	 if(CRC16 == FileCRCActual)
	 {
		  Write_APP_Address();
		  return 0;
	 }
	 return 1;
}
