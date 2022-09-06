/*
 * vl6180x_sample_plat.h
 *
 * VL6180 sample code wraper for STM32 Nucleo F401re board  plus VL6180x shield
 */

#ifndef VL6180X_SAMPLE_PLAT_H_
#define VL6180X_SAMPLE_PLAT_H_

#include <stdio.h>
#include <stdarg.h>
#include <vl6180x_api.h>
#include <string.h>

#include <gd32f30x_gpio.h>
#include "FreeRTOS.h"
#include "task.h"

#ifdef TRACE
#include "diag\trace.h"
#else 
#define trace_printf(...) (void)0
#endif

#define debug_stuff() (void)0

#define MyDev_printf(fmt, ...) trace_printf(fmt, ##__VA_ARGS__)
#define MyDev_Init(dev) (dev=0x52)

#define MyDev_ClearInterruptCpu(...) /* nothing*/
#define MyDev_SetEvent(...) /* nothing */

#define MyDev_uSleep(x) vTaskDelay(pdMS_TO_TICKS(x/1000))

#define SetDisplayString(str) (void)str

#define ALPHA (int)(0.85*(1<<16))
static inline void MyDev_ShowRange(VL6180xDev_t dev, int range, int duration){
    static char str[8]; // must remain valid until next display is done : this is why it cannot be a local variable
    static int Range; // must remain valid for leaky integrator
    // Apply leaky to smooth the display
    Range = (Range * ALPHA + range * ((1 << 16) - ALPHA)) >> 16;
    sprintf(str,"r%3d",Range);
    SetDisplayString(str);
}

static inline void MyDev_ShowErr(VL6180xDev_t dev, int err){
    static char str[8];
    sprintf(str,"E-%02d",err);
    SetDisplayString(str);
}


static inline void MyDev_ShowLux(VL6180xDev_t dev, int lux){
    static char str[8];
    if( lux >= 1000)
        sprintf(str,"L%3d ",lux/1000);
    else
        sprintf(str,"l%3d ",lux);
    SetDisplayString(str);
}


static inline void MyDev_ShowIntr(char c){
	  static char IntStr[8];
    IntStr[0]='I';
    IntStr[1]=c;
    IntStr[2]=c;
    IntStr[3]=c;
    IntStr[4]=0;
    SetDisplayString(IntStr);
}

static inline int MyDev_UserSayStop(VL6180xDev_t dev){
    return 0;
}


static inline void MyDev_UserWait(VL6180xDev_t dev , const char * msg)
{
		
}

static inline void MyDev_Printf(VL6180xDev_t dev , const char *fmt, ...)  {
    static char buffer[128];
    va_list ap;
    va_start(ap, fmt);
    vsprintf(buffer, fmt, ap);
    MyDev_UserWait(dev,buffer);
}

#define VL6180X_Reset() GPIO_BC(GPIOB) = GPIO_PIN_8
#define VL6180X_WaitMilliSec(x) vTaskDelay(pdMS_TO_TICKS(x))
#define VL6180X_Set() GPIO_BOP(GPIOB) = GPIO_PIN_8

static inline void MyDev_SetChipEnable(VL6180xDev_t dev) {
		VL6180X_Reset();
		VL6180X_WaitMilliSec(5);
		VL6180X_Set();
		VL6180X_WaitMilliSec(5);
}

/* enable interrupt at CPU level */
#define MyDev_EnableInterrupt(...) 
#define MyDev_RestartInterrupt(...) 

static inline void MyDev_WaitForEvent(VL6180xDev_t dev){
	  
}


#define HandleError(...) \
	do{ \
		SetDisplayString("ErrF"); \
	}while(0)

/*vl6180 als and range api*/
extern VL6180xDev_t theVL6180xDev;
void Sample_SimpleRanging(void);	
void Sample_Interrupt(void);
	
#endif /* VL6180X_SAMPLE_PLAT_H_ */
