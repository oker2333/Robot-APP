#include "ota.h"

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

