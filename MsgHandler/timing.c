#include <stdio.h>
#include <stdlib.h>

#include "print.h"
#include "timing.h"
#include "app_config.h"
#include "time_counter.h"

/*
bit0:  TOF����
bit1:  �����ź�
bit2:  ���ټ�
bit3:  IMU����
bit4:  IMU_Eularŷ����
bit5:  ������
bit6:  ��̼�
bit7:  
*/

typedef enum{
	TOF_BIT = 0,
	IR_BIT = 1,
	HALL_ENCODER_BIT = 2,
	IMU_DATA_BIT = 3,
	IMU_EULAE_BIT = 4,
	ODOMETER_BIT = 5,
	KEY_BIT = 6,
	ORITENTION = 7,
	BIT_MASK_NUM = 8
}BitMask_t;

static const uint16_t BitValue_List[BIT_MASK_NUM] = 
																	{1 << TOF_BIT,
																	 1 << IR_BIT,
																	 1 << HALL_ENCODER_BIT,
																	 1 << IMU_DATA_BIT,
																	 1 << IMU_EULAE_BIT,
																	 1 << ODOMETER_BIT,
																	 1 << KEY_BIT,
																	 1 << ORITENTION};

/**********************��ʱ�ϴ���ʱ��***********************/

static uint16_t TimingUpload_Timer[BIT_MASK_NUM] = {0};		/*������λ:1ms*/

static void UploadTimerUpdate(uint8_t bit_mask,uint16_t time_cycle)
{
	 TimingUpload_Timer[bit_mask] += time_cycle;
}

static void UploadTimerClear(uint8_t bit_mask)
{
	 TimingUpload_Timer[bit_mask] = 0x00;
}

static uint16_t UploadTimerAccess(uint8_t bit_mask)
{
	return TimingUpload_Timer[bit_mask];
}

/*******************��ʱ�ϴ�����(ʱ����)**************************/

static uint8_t TimingUpload_Span[BIT_MASK_NUM] = {0};			/*������λ:5ms*/

static uint16_t UploadSpanAccess(uint8_t bit_mask)
{
	return TimingUpload_Span[bit_mask]*5;
}

static uint16_t BitMask = 0x0000;

void TimingUpload_Set(uint16_t bit_mask,uint8_t* gap_buffer)
{
	BitMask = bit_mask;
	Mem_Copy(TimingUpload_Span,gap_buffer,BIT_MASK_NUM);
	
	robot_print("BitMask = 0x%x,TimingUpload_Span[] = {",BitMask);
	for(int i = 0;i < BIT_MASK_NUM;i++)
	{
		 robot_print("%d ",TimingUpload_Span[i]);
	}
	robot_print("}\r\n");
}

/*********************��ʱ�ϴ�����API************************/

static uint8_t timing_upload_frame(BitMask_t Bit,uint8_t* buffer,uint8_t index)
{
	 switch(Bit){
			case TOF_BIT:
				do{
					int16_t tof_mm = tof_mm_get();
					buffer[index++] = (tof_mm >> 0) & 0xFF;
					buffer[index++] = (tof_mm >> 8) & 0xFF;					
				}while(0);
			break;

			case IR_BIT:
				do{
					uint8_t ir_data = ir_value_get();
					buffer[index++] = (ir_data >> 0) & 0xFF;					
				}while(0);
			break;

			case HALL_ENCODER_BIT:
				do{
					buffer[index++] = 0x22;
				}while(0);
			break;

			case IMU_DATA_BIT:
				do{
					buffer[index++] = 0x33;
				}while(0);
			break;

			case IMU_EULAE_BIT:
				do{
					buffer[index++] = 0x44;
				}while(0);
			break;

			case ODOMETER_BIT:
				do{
					buffer[index++] = 0x55;
				}while(0);
			break;
				
			case KEY_BIT:
				do{
					buffer[index++] = 0x66;
				}while(0);
			break;
				
			case ORITENTION:
				do{
					buffer[index++] = 0x77;
				}while(0);
			break;

			default:
				robot_print("Unsupport Sensor Bit\n");
			break;
	 }
	 return index;
}

static void timing_upload_unit(uint16_t bit_mask)
{
	 if(bit_mask == 0x00)
	 {
		  return;
	 }

	 uint8_t index = 0;
	 uint8_t UserData[32] = {0};
	 uint16_t sequence = find_free_invoke_id();
	 
	 UserData[index++] = (bit_mask >> 0) & 0xFF;
	 UserData[index++] = (bit_mask >> 8) & 0xFF;
	 
	 for(BitMask_t shift_bit = TOF_BIT;shift_bit < BIT_MASK_NUM;shift_bit++)
	 {
		  if(bit_mask & BitValue_List[shift_bit])
			{
				 index = timing_upload_frame(shift_bit,UserData,index);
			}
	 }
	 
	 Create_Date_Frame(sequence,TIMING_UPLOAD,UserData,index);	
}

void timing_uploader(void)
{
	 uint16_t Current_BitMask = 0x0000;

	 static uint32_t timestamp = 0;
	 uint32_t time_gap = TimeStamp_access() - timestamp;
	 timestamp = TimeStamp_access();
	 
	 for(BitMask_t shift_bit = TOF_BIT;shift_bit < BIT_MASK_NUM;shift_bit++)
	 {
		 if(BitMask & BitValue_List[shift_bit])
		 {
			  UploadTimerUpdate(shift_bit,time_gap);
			  
			  if((UploadTimerAccess(shift_bit) >= UploadSpanAccess(shift_bit)) 
					&& (UploadSpanAccess(shift_bit) != 0))
				{
					 Current_BitMask |= BitValue_List[shift_bit];
					 UploadTimerClear(shift_bit);
				}
		 }
	 }
	 timing_upload_unit(Current_BitMask);
}
