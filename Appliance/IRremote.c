#include <stdio.h>
#include <string.h>

#include "IRremote.h"
#include "gd32f30x.h"

#define TIMER_PERIOD 10000

static uint32_t timer_update_count = 0;

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
void infrared_receiver_state_machine (void);

void TIMER1_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_CH0))
    {
        infrared_receiver_state_machine();

        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0);
    }
		
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP))
    {
			  timer_update_count++;
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

#define ALLOWED_OFFSET 400u

#define LEADER_CODE_TIMEOUT 13500u
#define BIT_SET_TIMEOUT 2250u
#define BIT_RESET_TIMEOUT 1125u

static uint32_t current_count = 0x00;
static uint32_t last_count = 0x00;

static void Timer_Reset(void)
{
	 timer_update_count = 0;
	 TIMER_CNT(TIMER1) = 0;
}

static uint32_t Timer_Interval(void)
{
	 uint32_t timeout = timer_update_count*TIMER_PERIOD + TIMER_CNT(TIMER1);
	 return timeout;
}

void infrared_receiver_state_machine (void)
{
	 static IR_Data_t ir_data = {0};
	 static IR_State_t ir_state = IDLE;
	 uint32_t timeout = 0x00;
	 
	 switch(ir_state)
	 {
		  case IDLE:
				 printf("IDLE\n");
			   Timer_Reset();
			   memset(&ir_data,0,sizeof(IR_Data_t));
			   ir_state = LEADER_CODE;
		  break;
			
		  case LEADER_CODE:
				printf("LEADER_CODE\n");
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= (LEADER_CODE_TIMEOUT - ALLOWED_OFFSET)) && (timeout <= (LEADER_CODE_TIMEOUT + ALLOWED_OFFSET)))
				 {
					  ir_state = ADDRESS_ORIGINAL;
				 }
				 else
				 {
					  ir_state = LEADER_CODE;
				 }
		  break;
			
		  case ADDRESS_ORIGINAL:
				printf("ADDRESS_ORIGINAL\n");
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= (BIT_SET_TIMEOUT - ALLOWED_OFFSET)) && (timeout <= (BIT_SET_TIMEOUT + ALLOWED_OFFSET)))
				 {
					  ir_data.address_original |= (1 << ir_data.index);
					  ir_data.index++;
				 }
				 else if((timeout >= (BIT_RESET_TIMEOUT - ALLOWED_OFFSET)) && (timeout <= (BIT_RESET_TIMEOUT + ALLOWED_OFFSET)))
				 {
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
				printf("ADDRESS_REVERSE\n");
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= (BIT_SET_TIMEOUT - ALLOWED_OFFSET)) && (timeout <= (BIT_SET_TIMEOUT + ALLOWED_OFFSET)))
				 {
					  ir_data.address_reverse |= (1 << ir_data.index);
					  ir_data.index++;
				 }
				 else if((timeout >= (BIT_RESET_TIMEOUT - ALLOWED_OFFSET)) && (timeout <= (BIT_RESET_TIMEOUT + ALLOWED_OFFSET)))
				 {
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
				printf("COMMAND_ORIGINAL\n");
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= (BIT_SET_TIMEOUT - ALLOWED_OFFSET)) && (timeout <= (BIT_SET_TIMEOUT + ALLOWED_OFFSET)))
				 {
					  ir_data.command_original |= (1 << ir_data.index);
					  ir_data.index++;
				 }
				 else if((timeout >= (BIT_RESET_TIMEOUT - ALLOWED_OFFSET)) && (timeout <= (BIT_RESET_TIMEOUT + ALLOWED_OFFSET)))
				 {
					  ir_data.command_original &= ~(1 << ir_data.index);
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
					   printf("command_original = 0x%x\n",ir_data.command_original);
				 }
		  break;

		  case COMMAND_REVERSE:
				printf("COMMAND_REVERSE\n");
			   timeout = Timer_Interval();
			   Timer_Reset();
			   if((timeout >= (BIT_SET_TIMEOUT - ALLOWED_OFFSET)) && (timeout <= (BIT_SET_TIMEOUT + ALLOWED_OFFSET)))
				 {
					  ir_data.command_reverse |= (1 << ir_data.index);
					  ir_data.index++;
				 }
				 else if((timeout >= (BIT_RESET_TIMEOUT - ALLOWED_OFFSET)) && (timeout <= (BIT_RESET_TIMEOUT + ALLOWED_OFFSET)))
				 {
					  ir_data.command_reverse &= ~(1 << ir_data.index);
					  ir_data.index++;
				 }
				 else
				 {
					   ir_state = LEADER_CODE;
				 }
				 
				 if(ir_data.index == 8)
				 {
					   ir_state = REPEAT_CODE;
					   ir_data.index = 0;
					   printf("command_reverse = 0x%x\n",ir_data.command_reverse);
				 }
		  break;

		  case REPEAT_CODE:
			   printf("REPEAT_CODE\n");
		  break;
			
		  default:
			   printf("IR State Error\n");
		  break;
	 }
}
