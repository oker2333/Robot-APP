#include "sensor_param.h"

int16_t tof_mm = 0;

void tof_mm_set(uint16_t data)
{
	 tof_mm = data;
}

int16_t tof_mm_get(void)
{
	 return tof_mm;
}

uint8_t ir_value = -1;

void ir_value_set(uint8_t data)
{
	 ir_value = data;
}

uint8_t ir_value_get(void)
{
	 return ir_value;
}

