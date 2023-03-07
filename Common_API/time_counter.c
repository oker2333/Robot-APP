#include <stdio.h>
#include "time_counter.h"

typedef struct{
	uint16_t ms;		/*0 ~ 999*/
	uint8_t sec;		/*0 ~ 59*/
	uint8_t min;		/*0 ~ 59*/
	uint8_t hour;		/*0 ~ 24*/
	uint8_t day;		/*0 ~ 255*/
}Date_t;

static Date_t Time;

char* RTOS_TimeStamp(void)
{
	static char date_string[20] = {0};
	if(Time.day != 0)
	{
		sprintf(date_string,"[%d:%d:%d:%d:%d]",Time.day,Time.hour,Time.min,Time.sec,Time.ms);
	}else if(Time.hour != 0)
	{
		sprintf(date_string,"[%d:%d:%d:%d]",Time.hour,Time.min,Time.sec,Time.ms);
	}else if(Time.min != 0)
	{
		sprintf(date_string,"[%d:%d:%d]",Time.min,Time.sec,Time.ms);
	}else if(Time.sec != 0)
	{
		sprintf(date_string,"[%d:%d]",Time.sec,Time.ms);
	}else if(Time.ms != 0)
	{
		 sprintf(date_string,"[%d]",Time.ms);
	}
	
  return date_string;
}

void RTOS_TimeUpdate(void)
{
	  Time.ms++;
		if(Time.ms == 1000)
		{
			 Time.sec++;
			 Time.ms = 0;
			 if(Time.sec == 60)
			 {
				  Time.min++;
				  Time.sec = 0;
				  if(Time.min == 60)
				  {
						Time.hour++;
						Time.min = 0;
				    if(Time.hour == 24)
				    {
							Time.day++;
							Time.hour = 0;
				    }
				  }
			 }
		}
}

static uint32_t g_Timer = 0;

void TimeStamp_update(void)
{
	g_Timer++;
}

uint32_t TimeStamp_access(void)
{
	return g_Timer;
}

static uint32_t IR_Timer = 0;

void IR_TimeStamp(void)
{
	IR_Timer = g_Timer;
}

uint32_t IR_TimeSpan(void)
{
	 return g_Timer - IR_Timer;
}
