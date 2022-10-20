#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "synchronous.h"

#define semaphore_lock() (void*)0
#define semaphore_unlock() (void*)0

typedef struct{
	bool occupied;
	SemaphoreHandle_t semaphore;
}Sem_Info_t;

static Sem_Info_t sem_pool[SENSOR_NUM];

static SemaphoreHandle_t get_semaphore(Sensor_Id_t id)
{
	SemaphoreHandle_t sem_ptr = NULL;
	semaphore_lock();
	if(sem_pool[id].occupied == false)
	{
		sem_pool[id].occupied = true;
		sem_pool[id].semaphore = xSemaphoreCreateBinary();
		sem_ptr = sem_pool[id].semaphore;
	}
	semaphore_unlock();
	return sem_ptr;
}

static SemaphoreHandle_t search_semaphore(Sensor_Id_t id)
{
	SemaphoreHandle_t sem_ptr = NULL;
	semaphore_lock();
	if(sem_pool[id].occupied == true)
	{
		sem_ptr = sem_pool[id].semaphore;
	}
	semaphore_unlock();
	return sem_ptr;
}

static void free_semaphore(Sensor_Id_t id)
{
	semaphore_lock();
	sem_pool[id].occupied = false;
	semaphore_unlock();
}

bool semaphore_timed_wait(Sensor_Id_t id)
{
	BaseType_t err = pdFALSE;
	SemaphoreHandle_t sem_ptr = NULL;
	
	sem_ptr = get_semaphore(id);
	if(!sem_ptr){
		printf("get_semaphore failed\n");
		return false;
	}
	
	err = xSemaphoreTake(sem_ptr, portMAX_DELAY);
	if(err == pdFALSE){
		vSemaphoreDelete(sem_ptr);
		free_semaphore(id);
		printf("sem_timedwait error\r\n");
		return false;
	}

	vSemaphoreDelete(sem_ptr);
	free_semaphore(id);

	return true;
}

bool semaphore_post(Sensor_Id_t id)
{
	SemaphoreHandle_t sem_ptr = search_semaphore(id);
	if(sem_ptr == NULL){
		printf("semaphore posted doesn't exist\r\n");
		return false;
	}
	
	BaseType_t err = pdFALSE; 
	err = xSemaphoreGive(sem_ptr);
	if(err == pdFALSE)
	{
		 printf("semaphore posted failed\r\n");
		 return false;
	}
	
	return true;
}
