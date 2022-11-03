#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

void motor_init(void);
void motor_control(int32_t left,int32_t right);

void PID_Controller(int left_target,int right_target);


#endif
