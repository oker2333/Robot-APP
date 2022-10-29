#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>
#include <stdbool.h>

void encoder_init(void);

bool right_velocity_measurement(uint32_t time_interval);

#endif
