#include "handler.h"
#include "fifo.h"
#include "ota.h"
#include "crc.h"
#include "fms.h"
#include "app_config.h"

#include <stdint.h>

#define ACK_DATA_SIZE 128

uint8_t ack_data[ACK_DATA_SIZE];
uint16_t ack_length = 0;
uint16_t ack_cmd = 0;

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

void Online_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
		
}

void Inquire_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{

}

void Control_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{

}

void Timing_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{

}

void Upload_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{

}

void OTA_Handler(uint16_t sequence,uint16_t cmd,uint8_t *UserData,uint16_t DataLength)
{
	 uint16_t index = 0;
	 static int8_t OTA_Device = 0;
	 static uint32_t OTA_Rev_Bytes = 0;
	 
	 switch(cmd)
	 {
		 case OTA_START:
			 OTA_Device = UserData[0];
		   ack_cmd = OTA_ACK;
		   ack_data[0] = OTA_Device;
		   ack_data[1] = SUCCEED;
		   ack_data[2] = (cmd >> 8) & 0xff;
		   ack_data[3] = (cmd >> 0) & 0xff;
		   ack_length = 0x04;
		   Create_Date_Frame(sequence,ack_cmd,ack_data,ack_length);
		 break;
		 
		 case OTA_FRAME:
			 OTA_Device = UserData[0];
		   uint32_t OTA_Offset = (UserData[1] << 24) | (UserData[2] << 16) | (UserData[3] << 8) | (UserData[4] << 0);
		   uint32_t OTA_Length = (UserData[5] << 8) | (UserData[6] << 0);
		   printf("OTA_Offset = %d,OTA_Length = %d\r\n",OTA_Offset,OTA_Length);

		   OTA_Rev_Bytes = Download2Flash(OTA_Offset,&UserData[7],OTA_Length);
		   
		   ack_cmd = OTA_ACK;
		   ack_data[0] = OTA_Device;
		   ack_data[1] = SUCCEED;
		   ack_data[2] = (cmd >> 8) & 0xff;
		   ack_data[3] = (cmd >> 0) & 0xff;
		   ack_length = 0x04;
		   Create_Date_Frame(sequence,ack_cmd,ack_data,ack_length);
		 break;
		 
		 case OTA_END:
			 OTA_Device = UserData[0];
		   uint16_t CRC16 = (UserData[1] << 8) | (UserData[2] << 0);
		   uint8_t OTA_Result = FlashBinaryCheck(CRC16,OTA_Rev_Bytes);

		   ack_cmd = OTA_ACK;
		   ack_data[0] = OTA_Device;
		   ack_data[1] = OTA_Result;
		   ack_data[2] = (cmd >> 8) & 0xff;
		   ack_data[3] = (cmd >> 0) & 0xff;
		   ack_length = 0x04;
		   Create_Date_Frame(sequence,ack_cmd,ack_data,ack_length);		 
		 break;
		 
		 case OTA_ACK:
			 
		 break;
	 }	 
}

msg_handler_t Callback_Handler[CALLBACK_NUM] = {Online_Handler,Inquire_Handler,Control_Handler,Timing_Handler,Upload_Handler,OTA_Handler};
