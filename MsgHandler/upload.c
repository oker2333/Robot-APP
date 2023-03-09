#include <stdint.h>
#include <stdbool.h>

#include "upload.h"
#include "handler.h"
#include "switch.h"
#include "print.h"
#include "time_counter.h"

#define MONITOR_NUMBER 10

typedef struct{
   uint8_t timer;
	 uint8_t status;
	 uint8_t retries;
	 uint8_t ack;
	 uint16_t cmd;
	 uint16_t id;
}Timeout_Couter;

static int8_t key_type_backup = NO_PRESS;

Timeout_Couter monitor[MONITOR_NUMBER] = 
{
  {0,0,0,0,UPLOAD_KEY_TYPE}
};

int8_t monitor_cmd_search(uint16_t command)
{
	 int8_t index = -1;
	 for(int i = 0;i < MONITOR_NUMBER;i++)
	 {
		 if(monitor[i].cmd == command)
		 {
			  index = i;
			  break;
		 }
	 }
	return index;
}

void monitor_status_update(uint8_t status,int8_t index)
{
	 monitor[index].status = status;
}

void monitor_ack_update(uint16_t command,uint16_t sequence)
{
	 int8_t index = monitor_cmd_search(command);
	 if(index < 0)
	 {
		  return;
	 }
	 
	 if((monitor[index].id == sequence) || (monitor[index].cmd == command))
		  monitor[index].ack = true;
	 else
		  monitor[index].ack = false;
}

void monitor_timer_update(uint8_t time_interval)
{
	 for(int i = 0;i < MONITOR_NUMBER;i++)
	 {
		  if(monitor[i].status == true)
			{
				 monitor[i].timer += time_interval;
			}else{
				 monitor[i].timer = 0;
			}
	 }
}

void monitor_reset(int8_t index)
{
	 monitor[index].retries = 0;
	 monitor[index].status = 0;
	 monitor[index].timer = 0;
	 monitor[index].ack = 0;
	 monitor[index].id = 0;
}

void monitor_update(uint8_t time_interval)
{
	 monitor_timer_update(time_interval);
	 
	 for(int i = 0;i < MONITOR_NUMBER;i++)
	 {
		  if(((monitor[i].status == true) && 
				(monitor[i].timer >= ACTIVE_UPLOAD_TIMEOUT) && 
				(monitor[i].retries < ACTIVE_UPLOAD_MAX_RETRIES) && 
				(monitor[i].ack == false)) || 
				((monitor[i].status == true) && 
				(monitor[i].retries == 0)))
			{
				 uint16_t sequence = find_free_invoke_id();
				 
				 switch(monitor[i].cmd)
				 {
					 case UPLOAD_KEY_TYPE:
						 do{
							 uint8_t DataLength = 0;
							 uint8_t UserData[1] = {0};
							 UserData[DataLength++] = key_type_backup;
							 Create_Date_Frame(sequence,UPLOAD_KEY_TYPE,UserData,DataLength);
						 }while(0);
					 break;
				 }
				 monitor[i].timer = 0;
				 monitor[i].retries++;
				 monitor[i].id = sequence;
				 
				 print_info("active upload send cmd 0x%x for %d times\n",monitor[i].cmd,monitor[i].retries);
			}
			else if((monitor[i].status == true) && (monitor[i].ack == true))
		  {
					monitor_status_update(false,i);
				  print_info("active upload get host ack 0x%x\n",monitor[i].cmd|0x8000);
		  }
			else if((monitor[i].status == true) && (monitor[i].retries >= ACTIVE_UPLOAD_MAX_RETRIES) && (monitor[i].timer >= ACTIVE_UPLOAD_TIMEOUT))
		  {
				  monitor_status_update(false,i);
				  print_info("active upload cmd 0x%x retries 3 times and failed\n",monitor[i].cmd);
		  }
	 }
}

void active_uploader(void)
{
	 static uint32_t timestamp = 0; 
	 
	 uint32_t time_gap = TimeStamp_access() - timestamp;
	 timestamp = TimeStamp_access();
	 
   /*����ֵ���*/
	 int8_t key_type = get_key_type();
	 if(key_type)
	 {
		  key_type_backup = key_type;
		  int8_t index = monitor_cmd_search(UPLOAD_KEY_TYPE);
		  monitor_reset(index);
		  monitor_status_update(true,index);
		  set_key_type(NO_PRESS);
	 }
	 
	 monitor_update(time_gap);	 
}
