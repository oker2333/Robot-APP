#include <stdint.h>
#include <stdio.h>

#include "handler.h"
#include "fifo.h"
#include "ota.h"
#include "crc.h"
#include "fms.h"
#include "print.h"
#include "app_config.h"

#include "motor.h"

static uint16_t invoke_id = 0;

uint16_t find_free_invoke_id(void)
{
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

bool datalink_frame_send(msg_cmd_t cmd,Sensor_Id_t id,uint8_t* buffer,uint16_t len)
{
	printf("[datalink_frame_send]cmd = 0x%x\r\n",cmd);
	uint16_t invoke_id = 0;
	bool ret = false;
	uint8_t fail_count = 0;
	for(int i = 1;i <= RYTEIES;i++)
	{
		invoke_id = find_free_invoke_id();
		Create_Date_Frame(invoke_id,cmd,buffer,len);

		if(semaphore_timed_wait(id)){
			ret = true;
			break;
		}else{
		  fail_count++;
		}
	}
	if(fail_count != 0)
	{
		 printf("[datalink_frame_send]wait ack failed %d times\r\n",fail_count);
	}
	return ret;
}

void Online_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
	  int index = 0;
	  uint8_t buffer[10] = {0};
		switch(cmd)
		{
			case ONLINE_HEARTBEAT:
				buffer[index++] = 0x00;
			  Create_Date_Frame(sequence,ONLINE_HEARTBEAT_ACK,buffer,index);
				print_info("Heart Beart From Host\r\n");
			break;
			
			case ONLINE_HEARTBEAT_ACK:

			break;
		}
}

void Inquire_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{

}

void Control_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
	  int index = 0;
	  uint8_t buffer[10] = {0};
		int16_t left_velocity = 0;
		int16_t right_velocity = 0;
		
	  switch(cmd)
		{
			case CONTROL_SPEED:
			  left_velocity = (UserData[1] << 8) | UserData[0];
			  right_velocity = (UserData[3] << 8) | UserData[2];
				pid_motor_control(left_velocity,right_velocity);
			 
				buffer[index++] = 0x00;
				buffer[index++] = CONTROL_SPEED >> 8;
				buffer[index++] = CONTROL_SPEED & 0xFF;
			  Create_Date_Frame(sequence,CONTROL_ACK,buffer,index);
				print_info("Speed From Host\r\n");
			break;
			
			case CONTROL_ACK:
				
			break;
		}
		printf("[Control_Handler]cmd = 0x%x\r\n",cmd);
}

void Timing_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{

}

void Upload_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
	uint16_t ack_cmd = 0x00;
	
	switch(cmd)
	{
		 case UPLOAD_KEY_TYPE:
			
		 break;
		
		 case UPLOAD_ACK:
			 ack_cmd = (UserData[2] << 8) | UserData[3];
		   if(ack_cmd == UPLOAD_KEY_TYPE)
			   semaphore_post(KEY_ID);
		 break;
	}
	printf("[Upload_Handler]cmd = 0x%x\r\n",cmd);
}

void OTA_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
	 int index = 0;
	 uint8_t buffer[10] = {0};

	 static int8_t OTA_Device = -1;
	 static uint16_t Binary_Version = 0x00;
	 static uint32_t OTA_Rev_Bytes = 0;
	 
	 switch(cmd)
	 {
		 case OTA_START:
			 OTA_Device = UserData[0];
		   Binary_Version = (UserData[1] << 8) | UserData[2];
		   buffer[index++] = OTA_Device;
		   buffer[index++] = SUCCEED;
		   buffer[index++] = (cmd >> 8) & 0xff;
		   buffer[index++] = (cmd >> 0) & 0xff;
		   Create_Date_Frame(sequence,OTA_ACK,buffer,index);
		 break;
		 
		 case OTA_FRAME:
			 OTA_Device = UserData[0];
		   uint32_t OTA_Offset = (UserData[1] << 24) | (UserData[2] << 16) | (UserData[3] << 8) | (UserData[4] << 0);
		   uint32_t OTA_Length = (UserData[5] << 8) | (UserData[6] << 0);
		   printf("OTA_Offset = %d,OTA_Length = %d\r\n",OTA_Offset,OTA_Length);

		   OTA_Rev_Bytes = Download2Flash(OTA_Device,OTA_Offset,&UserData[7],OTA_Length);
		   
		   buffer[index++] = OTA_Device;
		   buffer[index++] = SUCCEED;
		   buffer[index++] = (cmd >> 8) & 0xff;
		   buffer[index++] = (cmd >> 0) & 0xff;
		   Create_Date_Frame(sequence,OTA_ACK,buffer,index);
		 break;
		 
		 case OTA_END:
			 OTA_Device = UserData[0];
		   uint16_t CRC16 = (UserData[1] << 8) | (UserData[2] << 0);
		   uint8_t OTA_Result = FlashBinaryCheck(OTA_Device,Binary_Version,CRC16,OTA_Rev_Bytes);

		   buffer[index++] = OTA_Device;
		   buffer[index++] = OTA_Result;
		   buffer[index++] = (cmd >> 8) & 0xff;
		   buffer[index++] = (cmd >> 0) & 0xff;
		   Create_Date_Frame(sequence,OTA_ACK,buffer,index);		 
		 break;
		 
		 case OTA_ACK:
			 
		 break;
	 }	 
	 printf("[OTA_Handler]cmd = 0x%x\r\n",cmd);
}

msg_handler_t Callback_Handler[CALLBACK_NUM] = {Online_Handler,Inquire_Handler,Control_Handler,Timing_Handler,Upload_Handler,OTA_Handler};
