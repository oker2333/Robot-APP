#ifndef SWITCH_H_
#define SWITCH_H_

#include <stdint.h>

void switch_init(void);

void set_key_type(int8_t type);
int8_t get_key_type(void);

#endif
