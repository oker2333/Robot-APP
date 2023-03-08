#include <stdio.h>
#include <stdlib.h>

#include "print.h"
#include "timing.h"
#include "app_config.h"
#include "time_counter.h"

/*
bit0:  TOF数据
bit1:  红外信号
bit2:  轮速计
bit3:  IMU数据
bit4:  IMU_Eular欧拉角
bit5:  编码器
bit6:  里程计
bit7:  
*/

typedef enum{
	TOF_BIT = 0,
	IR_BIT = 1,
	HALL_ENCODER_BIT = 2,
	IMU_DATA_BIT = 3,
	IMU_EULAE_BIT = 4,
	ODOMETER_BIT = 5,
	BIT_MASK_NUM = 6
}BitMask_t;

static const uint16_t BitValue_List[BIT_MASK_NUM] = {1 << TOF_BIT,1 << IR_BIT,1 << HALL_ENCODER_BIT,1 << IMU_DATA_BIT,1 << IMU_EULAE_BIT,1 << ODOMETER_BIT};

/**********************定时上传计时器***********************/

static uint16_t TimingUpload_Timer[BIT_MASK_NUM] = {0};		/*基本单位:1ms*/

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

/*******************定时上传参数(时间间隔)**************************/

static uint8_t TimingUpload_Span[BIT_MASK_NUM] = {0};			/*基本单位:5ms*/

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
	robot_print("}\n");
}

/*********************定时上传功能API************************/

static uint8_t timing_upload_frame(BitMask_t Bit,uint8_t* buffer,uint8_t index)
{
	 switch(Bit){
			case TOF_BIT:
				buffer[index++] = (tof_mm >> 0) & 0xFF;
			  buffer[index++] = (tof_mm >> 8) & 0xFF;
			break;

			case IR_BIT:
				buffer[index++] = (ir_value >> 0) & 0xFF;
			break;

			case HALL_ENCODER_BIT:
				buffer[index++] = (tof_mm >> 0) & 0xFF;
			  buffer[index++] = (tof_mm >> 8) & 0xFF;
			break;

			case IMU_DATA_BIT:
				buffer[index++] = (tof_mm >> 0) & 0xFF;
			  buffer[index++] = (tof_mm >> 8) & 0xFF;
			break;

			case IMU_EULAE_BIT:
				buffer[index++] = (tof_mm >> 0) & 0xFF;
			  buffer[index++] = (tof_mm >> 8) & 0xFF;
			break;

			case ODOMETER_BIT:
				buffer[index++] = (tof_mm >> 0) & 0xFF;
			  buffer[index++] = (tof_mm >> 8) & 0xFF;
			break;

			default:
				robot_print("Unsupport Sensor Bit\n");
			break;
	 }
	 return index;
}

static void timing_upload_unit(uint16_t bit_mask)
{
	 uint8_t index = 0;
	 uint8_t UserData[32] = {0};
	 uint16_t sequence = find_free_invoke_id();
	 
	 UserData[index++] = (bit_mask >> 8) & 0xFF;
	 UserData[index++] = (bit_mask >> 0) & 0xFF;
	 
	 for(BitMask_t shift_bit = TOF_BIT;shift_bit < BIT_MASK_NUM;shift_bit++)
	 {
		  if(BitMask & BitValue_List[shift_bit])
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
