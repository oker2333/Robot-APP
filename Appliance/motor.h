#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

void motor_init(void);
void motor_control(int32_t left,int32_t right);
void motor_info(int32_t* left,int32_t* right);

#endif
