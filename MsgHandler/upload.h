#ifndef UPLOAD_H_
#define UPLOAD_H_

#include <stdint.h>
#include <stdbool.h>

void active_uploader(void);

int8_t monitor_cmd_search(uint16_t command);
void monitor_status_update(uint8_t status,int8_t index);
void monitor_ack_update(uint16_t command,uint16_t sequence);
void monitor_timer_update(uint8_t time_interval);
void monitor_reset(int8_t index);
void monitor_update(uint8_t time_interval);

#endif
