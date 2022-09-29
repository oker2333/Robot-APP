#include "flash_rw.h"
#include "app_config.h"

void flash_write_pages(uint32_t address,uint32_t size)
{
	  fmc_unlock();
	
		for(int i = (address / FMC_PAGE_SIZE);i <= ((address + size) / FMC_PAGE_SIZE);i++)
		{
			 fmc_page_erase(i * FMC_PAGE_SIZE);
		}
		
	  fmc_lock();
}

uint32_t flash_write_buffer(uint32_t address, uint8_t *data,uint32_t data_len)
{
	  fmc_state_enum fmc_state;
	  uint32_t ret = 0;
	  uint8_t remainder = 0;
	  uint32_t write_value = 0;
	  uint16_t half_word = 0;
	  
	  fmc_unlock();

		remainder = address % 2;
	
		address -= remainder;
	
		for(int i = (address / FMC_PAGE_SIZE);i <= ((address + data_len) / FMC_PAGE_SIZE);i++)
		{
			 fmc_page_erase(i * FMC_PAGE_SIZE);
		}
		
	  if(remainder){
			  half_word = data[0] << 8;
			  fmc_state = fmc_halfword_program(address, half_word);
				
				fmc_flag_clear(FMC_FLAG_BANK0_END);
				fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
				fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
			
				if(fmc_state != FMC_READY){
				    ret = address;
					goto exit;
				}
			
			  address += 2;
			  data_len -= remainder;
			  data++;
		}
		
		for(int j = 0;j < data_len/4;j++)
		{
			  write_value = (data[4*j + 0] << 0) | (data[4*j + 1] << 8) | (data[4*j + 2] << 16) | (data[4*j + 3] << 24);
			  
				fmc_state = fmc_word_program(address, write_value);
			  address += 4;
			
				fmc_flag_clear(FMC_FLAG_BANK0_END);
				fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
				fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
			
				if(fmc_state != FMC_READY){
				    ret = address;
					goto exit;
				}
		}
		
		if(data_len%4 != 0)
		{
			 switch(data_len%4)
			 {
				 case 1:
					 write_value = data[data_len-1];
					 break;
				 case 2:
					 write_value = data[data_len-2] | (data[data_len-1] << 8);
					 break;
				 case 3:
					 write_value = data[data_len-3] | (data[data_len-2] << 8) | (data[data_len-1] << 16);
					 break;
				 default:
					 write_value = 0;
					 break;
			 }
				fmc_state = fmc_word_program(address, write_value);
			
				fmc_flag_clear(FMC_FLAG_BANK0_END);
				fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
				fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

				if(fmc_state != FMC_READY){
				    ret = address;
					goto exit;
				}			 
		}
		
exit:
	  fmc_lock();
		return ret;
}

void flash_read_buffer(uint32_t address, uint8_t *data,uint32_t data_len)
{
	  uint32_t* ptr = (uint32_t*)data;
		for(int i = 0;i < data_len/4;i++){
				*ptr++ = (*((volatile uint32_t*)(uint32_t)address));
			  address += 4;
		}
		
		uint8_t* byte_ptr = (uint8_t*)ptr;
		
		for(int j = 0;j < data_len%4;j++)
		{
			 *byte_ptr++ = (*((volatile uint8_t*)(uint32_t)address++));
		}
}


fmc_state_enum flash_write_word(uint32_t address, uint32_t data)
{
	  fmc_state_enum state;
		
    fmc_unlock();
		state = fmc_word_program(address, data);
	
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
	
	  fmc_lock();
		
		return state;
}

fmc_state_enum flash_write_dword(uint32_t address, uint16_t data)
{
	  fmc_state_enum state;
	
    fmc_unlock();
		state = fmc_halfword_program(address, data);

    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
	
	  fmc_lock();
		
		return state;
}

uint32_t flash_read_word(uint32_t address)
{
	  return *((uint32_t*)address);
}

uint16_t flash_read_dword(uint32_t address)
{
	  return *((uint16_t*)address);
}

uint8_t flash_read_byte(uint32_t address)
{
	  return *((uint8_t*)address);
}
