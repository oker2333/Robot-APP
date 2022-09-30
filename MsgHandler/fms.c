#include "fms.h"
#include "fifo.h"
#include <stdint.h>

extern FIFO_BUFFER *Queue_Communicate_RX;

void Receive_FMS(uint8_t data_byte)
{
    
}

void DataFrame_Handle(void)
{
	 while(FIFO_Count(Queue_Communicate_RX))
	 {
		  Receive_FMS(FIFO_Get(Queue_Communicate_RX));
	 }
}


