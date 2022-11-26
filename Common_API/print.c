#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "print.h"
#include "fifo.h"
#include "app_config.h"
#include "gd32f30x_libopt.h"


#define logENTER_CRITICAL_FROM_ISR() 	uint32_t ulReturn = taskENTER_CRITICAL_FROM_ISR()
#define logEXIT_CRITICAL_FROM_ISR() 	taskEXIT_CRITICAL_FROM_ISR(ulReturn)

int fputc(int ch, FILE *f)
{
	  #if FIFO_DEBUG
		FIFO_Put(Queue_log,ch);
	  #else
	  (void)f;
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
		usart_data_transmit(USART0, ch);
		#endif
	  return 0;
}

#define LOG_MAX_LEN 64

int robot_print(const char *const fmt, ...)
{
	  logENTER_CRITICAL_FROM_ISR();
	  
	  uint8_t len;
    char log_str[LOG_MAX_LEN];

  	va_list args;
    va_start(args, fmt);
    len = vsnprintf((char *)log_str, LOG_MAX_LEN, (char const *)fmt, args);	  
    va_end(args);
	  
	  for(int i = 0;i <= len;i++)
	  {
			#if FIFO_DEBUG
			FIFO_Put(Queue_log,log_str[i]);
			#else
			while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
			usart_data_transmit(USART0, log_str[i]);
			#endif
		}
		
		logEXIT_CRITICAL_FROM_ISR();
}

#if FIFO_DEBUG
extern FIFO_BUFFER *Queue_log;

void print_logs(void)
{
	  FIFO_Tansmit(Queue_log);
}
#endif
