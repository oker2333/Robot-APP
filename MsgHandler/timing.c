#include <stdio.h>

#include "print.h"
#include "timing.h"
#include "app_config.h"

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
	TOF_BIT = 1 << 0,
	IR_BIT = 1 << 1,
	HALL_ENCODER_BIT = 1 << 2,
	IMU_DATA_BIT = 1 << 3,
	IMU_EULAE_BIT = 1 << 4,
	ODOMETER_BIT = 1 << 5,
	BIT_MASK_NUM = 6
}BitMask_t;

typedef int Sensor_Type_t;

static uint16_t BitMask = 0x00FF;

void BitMask_Set(uint16_t DataByte)
{
	 BitMask = DataByte;
	 robot_print("Set BitMask = 0x%x\n",BitMask);
}

static uint8_t timing_upload_frame(Sensor_Type_t Bit,uint8_t* buffer,uint8_t index)
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

void timing_uploader(void)
{
	 static uint32_t timestamp = 0; 
	 
	 uint32_t time_gap = g_Timestamp - timestamp;
	 if(time_gap < TIMING_UPLOAD_CYCLE)
	 {
		  return;
	 }
	 timestamp = g_Timestamp;
	
	 uint8_t index = 0;
	 uint8_t UserData[32] = {0};
	 uint16_t sequence = find_free_invoke_id();
	 
	 UserData[index++] = (BitMask >> 8) & 0xFF;
	 UserData[index++] = (BitMask >> 0) & 0xFF;
	 
	 for(Sensor_Type_t sensor_bit = 0;sensor_bit < BIT_MASK_NUM;sensor_bit++)
	 {
		  if(BitMask & sensor_bit)
			{
				 index = timing_upload_frame(sensor_bit,UserData,index);
			}
	 }
	 
	 Create_Date_Frame(sequence,TIMING_UPLOAD,UserData,index);
}
