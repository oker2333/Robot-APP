#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>

void motor_init(void);
void motor_control(int32_t left,int32_t right);

void Left_PID_Controller(int left_measure);
void Right_PID_Controller(int right_measure);

#endif
