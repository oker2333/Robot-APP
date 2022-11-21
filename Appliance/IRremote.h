#ifndef IRREMOTE_H_
#define IRREMOTE_H_

typedef enum{
	  START,
	  MENU,
	  LEFT,
	  RIGHT,
	  UP,
	  DOWN,
	  TEST,
	  CANCEL,
	  PAUSE,
	  C,
    ZERO,	
    ONE,
		TWO,
	  THREE,
	  FOUR,
	  FIVE,
	  SIX,
	  SEVEN,
	  EIGHT,
	  NINE,
	  NONE
}IR_Key_t;

void ir_rx_init(void);
IR_Key_t IR_Key_Obtain(void);

#endif
