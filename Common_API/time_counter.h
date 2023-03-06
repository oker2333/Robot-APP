#ifndef TIME_COUNTER_H_
#define TIME_COUNTER_H_

#include <stdint.h>

void TimeStamp_update(void);
uint32_t TimeStamp_access(void);

char* RTOS_TimeStamp(void);
void RTOS_TimeUpdate(void);

#endif
