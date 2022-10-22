#include "fms.h"
#include "fifo.h"
#include "print.h"
#include "handler.h"
#include "crc.h"
#include "app_config.h"

struct data_frame_struct_t frame;

void Receive_FMS(uint8_t data_byte)
{
    static fms_state_t state = FMS_RECEIVE_STATE_IDLE;
	  switch(state)
		{
			case FMS_RECEIVE_STATE_IDLE:
				if(data_byte == 0xA5)
				{
					 state = FMS_RECEIVE_STATE_PREAMBLE;
				}
			break;
			
			case FMS_RECEIVE_STATE_PREAMBLE:
				if(data_byte == 0xA5)
				{
					 state = FMS_RECEIVE_STATE_LENGTH_H;
					 frame.DataCRCActual = 0xFFFF;
				}else
				{
					  state = FMS_RECEIVE_STATE_IDLE;
				}
			break;
			
			case FMS_RECEIVE_STATE_LENGTH_H:
				frame.DataLength = data_byte << 8;
			  frame.DataCRCActual = CRC16_Check(data_byte,frame.DataCRCActual);
			  state = FMS_RECEIVE_STATE_LENGTH_L;
			break;
			
			case FMS_RECEIVE_STATE_LENGTH_L:
				frame.DataLength += data_byte;
			  frame.DataCRCActual = CRC16_Check(data_byte,frame.DataCRCActual);
			  state = FMS_RECEIVE_STATE_SEQUENCE_H;
			break;

			case FMS_RECEIVE_STATE_SEQUENCE_H:
				frame.sequence = data_byte << 8;
			  frame.DataCRCActual = CRC16_Check(data_byte,frame.DataCRCActual);
			  state = FMS_RECEIVE_STATE_SEQUENCE_L;
			break;
			
			case FMS_RECEIVE_STATE_SEQUENCE_L:
				frame.sequence += data_byte;
			  frame.DataCRCActual = CRC16_Check(data_byte,frame.DataCRCActual);
			  state = FMS_RECEIVE_STATE_CMD_H;
			break;
			
			case FMS_RECEIVE_STATE_CMD_H:
				frame.command = data_byte << 8;
			  frame.DataCRCActual = CRC16_Check(data_byte,frame.DataCRCActual);
			  state = FMS_RECEIVE_STATE_CMD_L;
			break;
			
			case FMS_RECEIVE_STATE_CMD_L:
				frame.command += data_byte;
			  frame.index = 0x00;
			  frame.DataCRCActual = CRC16_Check(data_byte,frame.DataCRCActual);
				if(frame.index == (frame.DataLength - HEADER_BYTES - TAIL_BYTES))
				{
					 state = FMS_RECEIVE_STATE_CRC_H;
				}else
				{
					 state = FMS_RECEIVE_STATE_DATA;
				}
			break;
			
			case FMS_RECEIVE_STATE_DATA:
				frame.UserData[frame.index++] = data_byte;
			  frame.DataCRCActual = CRC16_Check(data_byte,frame.DataCRCActual);
			  if(frame.index == (frame.DataLength - HEADER_BYTES - TAIL_BYTES))
				{
					 state = FMS_RECEIVE_STATE_CRC_H;
				}
			break;
			
			case FMS_RECEIVE_STATE_CRC_H:
				frame.DataCRC = data_byte << 8;
			  state = FMS_RECEIVE_STATE_CRC_L;
			break;
			
			case FMS_RECEIVE_STATE_CRC_L:
				frame.DataCRC += data_byte;
			  if(frame.DataCRC != frame.DataCRCActual)
				{
					 state = FMS_RECEIVE_STATE_IDLE;
				}
				else
				{
					 state = FMS_RECEIVE_STATE_TAIL_H;
				}
			break;
			
			case FMS_RECEIVE_STATE_TAIL_H:
				if(data_byte == 0x5A)
				{
					 state = FMS_RECEIVE_STATE_TAIL_L;
				}
				else
				{
					 state = FMS_RECEIVE_STATE_IDLE;
				}
			break;
				
			case FMS_RECEIVE_STATE_TAIL_L:
				if(data_byte == 0x5A)
				{
					 uint8_t index = (frame.command & COMAND_MASK) >> 8;
					 Callback_Handler[index](frame.sequence,frame.command,frame.UserData,frame.index);
				}
		
				state = FMS_RECEIVE_STATE_IDLE;
			break;
				
			default:
				state = FMS_RECEIVE_STATE_IDLE;
			break;
		
		}
}

void DataFrame_Handle(void)
{
	 while(FIFO_Count(Queue_Usart1_RX))
	 {
		  Receive_FMS(FIFO_Get(Queue_Usart1_RX));
	 }
}

void DataFrame_Transmit(void)
{
	 FIFO_Tansmit(Queue_Usart1_TX);
}
