#include "print.h"
#include <stdio.h>
#include "fifo.h"
#include "app_config.h"
#include "gd32f30x_libopt.h"

extern FIFO_BUFFER *Queue_log;

int fputc(int ch, FILE *f)
{
	  #if FIFO_DEBUG
		FIFO_Put(Queue_log,ch);
	  #else
	  (void)f;
    usart_data_transmit(USART0, ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
	  #endif
	  return 0;
}

void print_logs(void)
{
	  FIFO_Tansmit(Queue_log);
}
