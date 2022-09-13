#include "print.h"
#include <stdio.h>
#include "fifo.h"

extern FIFO_BUFFER *Queue_log;

int fputc(int ch, FILE *f)
{
		FIFO_Put(Queue_log,ch);
	  return 0;
}

void print_logs(void)
{
	  FIFO_Tansmit(Queue_log);
}
