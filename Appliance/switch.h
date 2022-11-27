#ifndef SWITCH_H_
#define SWITCH_H_

#include <stdint.h>

typedef enum{
	NO_PRESS = 0,
	SHORT_PRESS = 1,
	LONG_PRESS = 2
}KeyType_t;

void switch_init(void);

void set_key_type(int8_t type);
int8_t get_key_type(void);

#endif
