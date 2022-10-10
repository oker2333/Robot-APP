#include "ota.h"
#include "crc.h"

typedef void (*Jump_To_ADDR_t)(void);

void Write_APP_Size(uint32_t File_Bytes)
{
	 flash_write_buffer(APP_SIZE_ADDRESS, (uint8_t*)&File_Bytes,sizeof(uint32_t));
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
		  uint32_t Cur_Download_Addr = Frame_ID * FMC_PAGE_SIZE + APP_BACKUP_ADDRESS;
		  
		  flash_write_buffer(Cur_Download_Addr, OTA_Page_Buffer,OTA_Count);
		  printf("[frame id %d]Current Download Address 0x%x,OTA_Count = %d\r\n",Frame_ID,Cur_Download_Addr,OTA_Count);
		  
		  Frame_ID++;
		  OTA_Count = 0x00;
	 }
	 else if(OTA_Length < 128)
	 {
		  uint32_t Cur_Download_Addr = Frame_ID * FMC_PAGE_SIZE + APP_BACKUP_ADDRESS;
		  
		  flash_write_buffer(Cur_Download_Addr, OTA_Page_Buffer,OTA_Count);
		  printf("[frame id %d]Current Download Address 0x%x,OTA_Count = %d\r\n",Frame_ID,Cur_Download_Addr,OTA_Count);
		  OTA_Count = 0x00;
		  Frame_ID = 0x00;
	 }
	 return (OTA_Offset + OTA_Length);
}

uint8_t FlashBinaryCheck(uint16_t CRC16,uint32_t OTA_Rev_Bytes)
{
	 uint16_t FileCRCActual = CRC16_CCITT_FALSE(((uint8_t*)APP_BACKUP_ADDRESS), OTA_Rev_Bytes);
	 printf("FileSize = %d Bytes,FileCRC = 0x%x,FileCRCActual = 0x%x\r\n",OTA_Rev_Bytes,CRC16,FileCRCActual);
	 
	 if(CRC16 == FileCRCActual)
	 {
		  Write_APP_Size(OTA_Rev_Bytes);
		  return 0;
	 }
	 return 1;
}
