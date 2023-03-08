#include <stdint.h>
#include <stdio.h>

#include "upload.h"
#include "handler.h"
#include "timing.h"
#include "fifo.h"
#include "ota.h"
#include "crc.h"
#include "fms.h"
#include "print.h"
#include "app_config.h"

#include "motor.h"

union U8_TO_FLOAT
{
	float f;
	unsigned char u8[4];
};

float u8_to_float(uint8_t* buffer)
{
	union U8_TO_FLOAT temp;
	for(int i = 0;i < 4;i ++)
	{
		temp.u8[i] = buffer[i];
	}
	return temp.f;
}

void float_to_u8(uint8_t* buffer,float value)
{
	union U8_TO_FLOAT temp;
	temp.f = value;
}

uint16_t find_free_invoke_id(void)
{
  static uint16_t invoke_id = 0;
	return invoke_id++;
}

void Create_Date_Frame(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
	 uint16_t index = 0;
	 uint16_t crc16 = 0x00;
   uint8_t Data_Frame[300];
	
	 Data_Frame[index++] = 0xA5;
	 Data_Frame[index++] = 0xA5;
	 Data_Frame[index++] = ((DataLength+HEADER_BYTES+TAIL_BYTES) >> 8) & 0xff;
	 Data_Frame[index++] = ((DataLength+HEADER_BYTES+TAIL_BYTES) >> 0) & 0xff;
	 Data_Frame[index++] = (sequence >> 8) & 0xff;
	 Data_Frame[index++] = (sequence >> 0) & 0xff;
	 Data_Frame[index++] = (cmd >> 8) & 0xff;
	 Data_Frame[index++] = (cmd >> 0) & 0xff;
	 
	 Mem_Copy(&Data_Frame[index],UserData,DataLength);
	 index += DataLength;
	 crc16 = CRC16_CCITT_FALSE(&Data_Frame[2],DataLength+HEADER_BYTES);
	 Data_Frame[index++] = (crc16 >> 8) & 0xff;
	 Data_Frame[index++] = (crc16 >> 0) & 0xff;
	 Data_Frame[index++] = 0x5A;
	 Data_Frame[index] = 0x5A;
	 
	 FIFO_Add(Queue_Usart1_TX, Data_Frame, DataLength+HEADER_BYTES+TAIL_BYTES+4);
}

bool datalink_frame_send(msg_cmd_t cmd,Sensor_Id_t id,uint8_t* buffer,uint16_t len,uint32_t timeout_ms)
{
	print_info("[datalink_frame_send]cmd = 0x%x\r\n",cmd);
	
	uint16_t invoke_id = 0;
	bool ret = false;
	uint8_t fail_count = 0;
	for(int i = 1;i <= RYTEIES;i++)
	{
		invoke_id = find_free_invoke_id();
		Create_Date_Frame(invoke_id,cmd,buffer,len);

		if(semaphore_timed_wait(id,timeout_ms)){
			ret = true;
			break;
		}else{
		  fail_count++;
		}
	}
	if(fail_count != 0)
	{
		 robot_print("[datalink_frame_send]wait ack failed %d times\r\n",fail_count);
	}
	return ret;
}

void Online_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
		print_info("[Online_Handler]cmd = 0x%x\r\n",cmd);
		
		switch(cmd)
		{
			case ONLINE_HEARTBEAT:
				do{
					int index = 0;
					uint8_t buffer[1] = {0};
					buffer[index++] = 0x00;
					Create_Date_Frame(sequence,ONLINE_HEARTBEAT_ACK,buffer,index);
					robot_print("Heart Beart From Host and reply\r\n");
				}while(0);
			break;
			
			case ONLINE_HEARTBEAT_ACK:

			break;
		}
}

void Inquire_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
		print_info("[Inquire_Handler]cmd = 0x%x\r\n",cmd);
}

void Control_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{	
	  print_info("[Control_Handler]cmd = 0x%x\r\n",cmd);
		
	  switch(cmd)
		{
			case CONTROL_SPEED:
				do{
					int16_t left_velocity = (UserData[1] << 8) | UserData[0];
					int16_t right_velocity = (UserData[3] << 8) | UserData[2];
					pid_motor_control(left_velocity,right_velocity);

					int index = 0;
					uint8_t buffer[3] = {0};
					buffer[index++] = 0x00;
					buffer[index++] = CONTROL_SPEED >> 8;
					buffer[index++] = CONTROL_SPEED & 0xFF;
					Create_Date_Frame(sequence,CONTROL_ACK,buffer,index);
					robot_print("speed from host and reply\r\n");
				}while(0);
			break;
			
			case CONTROL_TIMING_PARAM:
				do{
					uint16_t bit_mask = (UserData[1] << 8) | UserData[0];
					TimingUpload_Set(bit_mask,&UserData[2]);
					
					int index = 0;
					uint8_t buffer[3] = {0};
					buffer[index++] = 0x00;
					buffer[index++] = CONTROL_TIMING_PARAM >> 8;
					buffer[index++] = CONTROL_TIMING_PARAM & 0xFF;
					Create_Date_Frame(sequence,CONTROL_ACK,buffer,index);
					robot_print("timing param from host and reply\r\n");
				}while(0);
			break;
				
			case CONTROL_RESET:
				do{
					NVIC_SystemReset();
					
					robot_print("system reset from host and don't reply\r\n");
				}while(0);
			break;
			
			case CONTROL_ACK:
				
			break;
		}
}

void Timing_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{

}

void Upload_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
	print_info("[Upload_Handler]cmd = 0x%x\r\n",cmd);	
	
	switch(cmd)
	{
		 case UPLOAD_KEY_TYPE:
			
		 break;
		
		 case UPLOAD_ACK:
			 do{
				 uint16_t ack_cmd = (UserData[1] << 8) | UserData[2];
				 if(ack_cmd == UPLOAD_KEY_TYPE)
				 {
						monitor_ack_update(UPLOAD_KEY_TYPE,sequence);
				 }
			 }while(0);

		 break;
	}
}

void OTA_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
	 print_info("[OTA_Handler]cmd = 0x%x\r\n",cmd);

	 static int8_t OTA_Device = -1;
	 static uint16_t Binary_Version = 0x00;
	 static uint32_t OTA_Rev_Bytes = 0;
	 
	 switch(cmd)
	 {
		 case OTA_START:
			 do{
				 int index = 0;
				 uint8_t buffer[4] = {0};
				 
				 OTA_Device = UserData[0];
				 Binary_Version = (UserData[1] << 8) | UserData[2];
				 buffer[index++] = OTA_Device;
				 buffer[index++] = SUCCEED;
				 buffer[index++] = (cmd >> 8) & 0xff;
				 buffer[index++] = (cmd >> 0) & 0xff;
				 Create_Date_Frame(sequence,OTA_ACK,buffer,index);
			 }while(0);
		 break;
		 
		 case OTA_FRAME:
			 do{
				 int index = 0;
				 uint8_t buffer[4] = {0};
				 
				 OTA_Device = UserData[0];
				 uint32_t OTA_Offset = (UserData[1] << 24) | (UserData[2] << 16) | (UserData[3] << 8) | (UserData[4] << 0);
				 uint32_t OTA_Length = (UserData[5] << 8) | (UserData[6] << 0);
				 robot_print("OTA_Offset = %d,OTA_Length = %d\r\n",OTA_Offset,OTA_Length);

				 OTA_Rev_Bytes = Download2Flash(OTA_Device,OTA_Offset,&UserData[7],OTA_Length);
				 
				 buffer[index++] = OTA_Device;
				 buffer[index++] = SUCCEED;
				 buffer[index++] = (cmd >> 8) & 0xff;
				 buffer[index++] = (cmd >> 0) & 0xff;
				 Create_Date_Frame(sequence,OTA_ACK,buffer,index);
			 }while(0);
		 break;
		 
		 case OTA_END:
			 do{
				 int index = 0;
				 uint8_t buffer[4] = {0};
				 
				 OTA_Device = UserData[0];
				 uint16_t CRC16 = (UserData[1] << 8) | (UserData[2] << 0);
				 uint8_t OTA_Result = FlashBinaryCheck(OTA_Device,Binary_Version,CRC16,OTA_Rev_Bytes);

				 buffer[index++] = OTA_Device;
				 buffer[index++] = OTA_Result;
				 buffer[index++] = (cmd >> 8) & 0xff;
				 buffer[index++] = (cmd >> 0) & 0xff;
				 Create_Date_Frame(sequence,OTA_ACK,buffer,index);
			 }while(0);
		 break;
		 
		 case OTA_ACK:
			 
		 break;
	 }	 
}

msg_handler_t Callback_Handler[CALLBACK_NUM] = {Online_Handler,Inquire_Handler,Control_Handler,Timing_Handler,Upload_Handler,OTA_Handler};
