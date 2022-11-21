#include <stdio.h>
#include <string.h>

#include "IRremote.h"
#include "gd32f30x.h"

#define TIMER_PERIOD 10000

void ir_rx_init(void)
{
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_TIMER1);

    /*configure PA15 (TIMER1 CH0) as alternate function*/
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_15);

    timer_deinit(TIMER1);
	
		gpio_pin_remap_config(GPIO_TIMER1_FULL_REMAP, ENABLE);
	
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 119;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = TIMER_PERIOD;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* TIMER1 CH0 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_FALLING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 10;
    timer_input_capture_config(TIMER1,TIMER_CH_0,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* clear channel 0 interrupt bit */
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0 | TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1,TIMER_INT_CH0 | TIMER_INT_UP);
    /* channel 0 interrupt enable */
    nvic_irq_enable(TIMER1_IRQn, 10, 0);

    /* TIMER1 counter enable */
    timer_enable(TIMER1);
}

/**
  * @brief  This function handles TIMER1 interrupt request.
  * @param  None
  * @retval None
  */
void ir_state_machine (void);
void timer_update(void);

void TIMER1_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_CH0))
    {
        ir_state_machine();

        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0);
    }
		
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP))
    {
			  timer_update();
        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);
    }
}

/**************************************红外接收器状态机**********************************************/

typedef enum{
   IDLE,
	 LEADER_CODE,
	 ADDRESS_ORIGINAL,
	 ADDRESS_REVERSE,
	 COMMAND_ORIGINAL,
	 COMMAND_REVERSE,
	 REPEAT_CODE_PRE,
	 REPEAT_CODE
}IR_State_t;

typedef struct{
    uint8_t address_original;
	  uint8_t address_reverse;
	  uint8_t command_original;
	  uint8_t command_reverse;
	  uint8_t repeat_conter;
	  uint8_t index;
}IR_Data_t;

#define SYSTICK_COUNT_MAX TIMER_PERIOD

#define TIMER_STATE_TIMEOUT 12

#define MAX_RATIO 1.1f
#define MIN_RATIO 0.9f

#define LEADER_CODE_TIMEOUT 13500u
#define BIT_SET_TIMEOUT 2250u
#define BIT_RESET_TIMEOUT 1125u
#define REPEAT_CODE_PRE_V1_TIMEOUT 108000u
#define REPEAT_CODE_PRE_V2_TIMEOUT 96750u
#define REPEAT_CODE_TIMEOUT 11250u

#define LEADER_CODE_TIMEOUT_MAX (LEADER_CODE_TIMEOUT * MAX_RATIO)
#define LEADER_CODE_TIMEOUT_MIN (LEADER_CODE_TIMEOUT * MIN_RATIO)

#define BIT_SET_TIMEOUT_MAX (BIT_SET_TIMEOUT * MAX_RATIO)
#define BIT_SET_TIMEOUT_MIN (BIT_SET_TIMEOUT * MIN_RATIO)

#define BIT_RESET_TIMEOUT_MAX (BIT_RESET_TIMEOUT * MAX_RATIO)
#define BIT_RESET_TIMEOUT_MIN (BIT_RESET_TIMEOUT * MIN_RATIO)

#define REPEAT_CODE_PRE_V1_MAX (REPEAT_CODE_PRE_V1_TIMEOUT * MAX_RATIO)
#define REPEAT_CODE_PRE_V1_MIN (REPEAT_CODE_PRE_V1_TIMEOUT * MIN_RATIO)

#define REPEAT_CODE_PRE_V2_MAX (REPEAT_CODE_PRE_V2_TIMEOUT * MAX_RATIO)
#define REPEAT_CODE_PRE_V2_MIN (REPEAT_CODE_PRE_V2_TIMEOUT * MIN_RATIO)

#define REPEAT_CODE_TIMEOUT_MAX (REPEAT_CODE_TIMEOUT * MAX_RATIO)
#define REPEAT_CODE_TIMEOUT_MIN (REPEAT_CODE_TIMEOUT * MIN_RATIO)

static IR_Data_t ir_data = {0};
static IR_State_t ir_state = IDLE;

static uint32_t timer_update_count = 0;

void timer_update(void)
{
	timer_update_count++;
	if(timer_update_count >= TIMER_STATE_TIMEOUT)
	{
		 timer_update_count = 0;
		 ir_state = IDLE;
	}	
}

static void Timer_Reset(void)
{
	 timer_update_count = 0;
	 TIMER_CNT(TIMER1) = 0;
}

static uint32_t Timer_Interval(void)
{
	 return timer_update_count * TIMER_PERIOD + TIMER_CNT(TIMER1);
}

void ir_state_machine (void)
{
	 static uint32_t time_accumulation = 0x00;
	 uint32_t timeout = 0x00;
	 
	 switch(ir_state)
	 {
		  case IDLE:
			   Timer_Reset();
			   ir_state = LEADER_CODE;
		  break;
			
		  case LEADER_CODE:
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= LEADER_CODE_TIMEOUT_MIN) && (timeout <= LEADER_CODE_TIMEOUT_MAX))
				 {
					  time_accumulation = timeout;
					  memset(&ir_data,0,sizeof(IR_Data_t));
					  ir_state = ADDRESS_ORIGINAL;
				 }
				 else
				 {
					  ir_state = LEADER_CODE;
				 }
		  break;
			
		  case ADDRESS_ORIGINAL:
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= BIT_SET_TIMEOUT_MIN) && (timeout <= BIT_SET_TIMEOUT_MAX))
				 {
					  time_accumulation += timeout;
					  ir_data.address_original |= (1 << ir_data.index);
					  ir_data.index++;
				 }
				 else if((timeout >= BIT_RESET_TIMEOUT_MIN) && (timeout <= BIT_RESET_TIMEOUT_MAX))
				 {
					  time_accumulation += timeout;
					  ir_data.address_original &= ~(1 << ir_data.index);
					  ir_data.index++;
				 }
				 else
				 {
					   ir_state = LEADER_CODE;
				 }
				 
				 if(ir_data.index == 8)
				 {
					   ir_state = ADDRESS_REVERSE;
					   ir_data.index = 0;
					   printf("address_original = 0x%x\n",ir_data.address_original);
				 }
		  break;

		  case ADDRESS_REVERSE:
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= BIT_SET_TIMEOUT_MIN) && (timeout <= BIT_SET_TIMEOUT_MAX))
				 {
					  time_accumulation += timeout;
					  ir_data.address_reverse |= (1 << ir_data.index);
					  ir_data.index++;
				 }
				 else if((timeout >= BIT_RESET_TIMEOUT_MIN) && (timeout <= BIT_RESET_TIMEOUT_MAX))
				 {
					  time_accumulation += timeout;
					  ir_data.address_reverse &= ~(1 << ir_data.index);
					  ir_data.index++;
				 }
				 else
				 {
					   ir_state = LEADER_CODE;
				 }
				 
				 if(ir_data.index == 8)
				 {
					   ir_state = COMMAND_ORIGINAL;
					   ir_data.index = 0;
					   printf("address_reverse = 0x%x\n",ir_data.address_reverse);
				 }
		  break;

		  case COMMAND_ORIGINAL:
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= BIT_SET_TIMEOUT_MIN) && (timeout <= BIT_SET_TIMEOUT_MAX))
				 {
					  time_accumulation += timeout;
					  ir_data.command_original |= (1 << ir_data.index);
					  ir_data.index++;
				 }
				 else if((timeout >= BIT_RESET_TIMEOUT_MIN) && (timeout <= BIT_RESET_TIMEOUT_MAX))
				 {
					  time_accumulation += timeout;
					  ir_data.command_original &= ~(1 << ir_data.index);
					  ir_data.index++;
				 }
				 else
				 {
					   ir_state = LEADER_CODE;
				 }
				 
				 if(ir_data.index == 8)
				 {
					   ir_state = COMMAND_REVERSE;
					   ir_data.index = 0;
					   printf("command_original = 0x%x\n",ir_data.command_original);
				 }
		  break;

		  case COMMAND_REVERSE:
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= BIT_SET_TIMEOUT_MIN) && (timeout <= BIT_SET_TIMEOUT_MAX))
				 {
					  time_accumulation += timeout;
					  ir_data.command_reverse |= (1 << ir_data.index);
					  ir_data.index++;
				 }
				 else if((timeout >= BIT_RESET_TIMEOUT_MIN) && (timeout <= BIT_RESET_TIMEOUT_MAX))
				 {
					  time_accumulation += timeout;
					  ir_data.command_reverse &= ~(1 << ir_data.index);
					  ir_data.index++;
				 }
				 else
				 {
					   ir_state = LEADER_CODE;
				 }
				 
				 if(ir_data.index == 8)
				 {
					   ir_state = REPEAT_CODE_PRE;
					   ir_data.index = 0;
					   printf("command_reverse = 0x%x\n",ir_data.command_reverse);
				 }
		  break;

			case REPEAT_CODE_PRE:
				 timeout = Timer_Interval();
			   Timer_Reset();
			   time_accumulation += timeout;
			   if(((time_accumulation >= REPEAT_CODE_PRE_V1_MIN) && (time_accumulation <= REPEAT_CODE_PRE_V1_MAX)) || 
					  ((timeout >= REPEAT_CODE_PRE_V2_MIN) && (timeout <= REPEAT_CODE_PRE_V2_MAX)))
				 {
					  time_accumulation = 0x00;
					  ir_state = REPEAT_CODE;
				 }
				 else
				 {
						ir_state = LEADER_CODE;
				 }
			break;
				 
		  case REPEAT_CODE:
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= REPEAT_CODE_TIMEOUT_MIN) && (timeout <= REPEAT_CODE_TIMEOUT_MAX))
				 {
					  ir_data.repeat_conter++;
					  ir_state = REPEAT_CODE_PRE;
					  printf("repeat_conter = %d\n",ir_data.repeat_conter);
				 }
				 else
				 {
					  ir_state = LEADER_CODE;
				 }
			
		  break;
			
		  default:
			   printf("IR State Error\n");
		  break;
	 }
}
