#ifndef SYNCHRONOUS_H_
#define SYNCHRONOUS_H_

#define TIMEOUT_MS 200
#define RYTEIES 3

#include <stdbool.h>

typedef enum{
	 KEY_ID,
	 SENSOR_NUM
}Sensor_Id_t;

bool semaphore_timed_wait(Sensor_Id_t id);
bool semaphore_post(Sensor_Id_t id);

#endif
