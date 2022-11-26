#include <stdint.h>
#include <stdio.h>

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
	TOF_BIT,
	IR_BIT,
	HALL_ENCODER_BIT,
	IMU_DATA_BIT,
	IMU_EULAE_BIT,
	ODOMETER_BIT,
	BIT_MASK_NUM
}BitMask_t;

typedef int SensorBit_t;

static uint16_t BitMask = 0x00FF;

void BitMask_Set(uint16_t DataByte)
{
	 BitMask = DataByte;
	 robot_print("BitMask = 0x%x\n",BitMask);
}

uint8_t timing_upload_frame(SensorBit_t Bit,uint8_t* buffer,uint8_t index)
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
	 uint8_t index = 0;
	 uint8_t UserData[32] = {0};
	 uint16_t sequence = find_free_invoke_id();
	 
	 UserData[index++] = (BitMask >> 8) & 0xFF;
	 UserData[index++] = (BitMask >> 0) & 0xFF;
	 
	 for(SensorBit_t sensor_bit = 0;sensor_bit < BIT_MASK_NUM;sensor_bit++)
	 {
		  if(BitMask & (1 << sensor_bit))
			{
				 index = timing_upload_frame(sensor_bit,UserData,index);
			}
	 }
	 
	 Create_Date_Frame(sequence,TIMING_UPLOAD,UserData,index);
}
