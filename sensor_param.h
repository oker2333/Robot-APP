#ifndef SENSOR_PARAM_H_
#define SENSOR_PARAM_H_

#include <stdint.h>

void tof_mm_set(uint16_t data);
int16_t tof_mm_get(void);

void ir_value_set(uint8_t data);
uint8_t ir_value_get(void);

void set_key_type(int8_t type);
int8_t get_key_type(void);

#endif
