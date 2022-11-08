#include "IRremote.h"
#include "gd32f30x.h"
/**
* @breaf 红外摇控器发射红外信息，红外接收传感器的接收信号，判断接收到的是0 还是 1,
* 破解摇器的控制码
* 
* detailed 使用定时器的捕获功能；读取红外接收传感器输出引脚脉冲宽度（us高电平）。
* 实现方法，将定时器初始化为捕获上升沿，捕获到上升沿，清空定时器计数值，将定时器
* 设置为捕获到下降沿，捕获到下降沿，读定时器计数器值，将定时器配置为捕获上升沿。将
* 断读取到的计数器值 450左右 为1， 1600左右为 0，4500左右为引导码，不同的红外信号
* 编码方式不同，要根据不同的信号作出调整；
*/
__IO uint16_t readvalue1 = 0, readvalue2 = 0;
__IO uint16_t ccnumber = 0;
__IO uint32_t count = 0;
__IO float fre = 0;

static timer_ic_parameter_struct timer_icinitpara;
static timer_parameter_struct timer_initpara;

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
    timer_initpara.period            = 10000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);

    /* TIMER1 CH3 input capture configuration */
    timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0;
    timer_input_capture_config(TIMER1,TIMER_CH_0,&timer_icinitpara);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER1);
    /* clear channel 3 interrupt bit */
    timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0 | TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1,TIMER_INT_CH0 | TIMER_INT_UP);
    /* channel 3 interrupt enable */
    nvic_irq_enable(TIMER1_IRQn, 10, 0);

    /* TIMER1 counter enable */
    timer_enable(TIMER1);
}

//遥控器接收状态
//[7]:收到了引导码标志 [6]:得到了一个按键的所有信息
//[5]:保留 [4]:标记上升沿是否已经被捕获 [3:0]:溢出计时器
static uint8_t RmtSta = 0;
static uint16_t Dval; //下降沿时计数器的值
static uint8_t RmtRec = 0; //红外接收到的数据
static uint8_t RmtCnt = 0; //按键按下的次数
static uint8_t iri = 0, irj = 0;
uint8_t IrCodeSize = 6;
uint8_t ir_rx_data[20] = {0};
/**
  * @brief  This function handles TIMER1 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER1_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_CH0))
    {
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_15))        //捕获到上升没沿，读取GPIO PIN状态当为1时是上升沿
        {
            timer_counter_value_config(TIMER1, 0);
            timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_FALLING;
            timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
            timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
            timer_icinitpara.icfilter    = 0;
            timer_input_capture_config(TIMER1,TIMER_CH_0,&timer_icinitpara);
        }
        else        ////捕获到下降沿，为0是下降沿
        {
            Dval = timer_counter_read(TIMER1);
            timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
            timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
            timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
            timer_icinitpara.icfilter    = 0;
            timer_input_capture_config(TIMER1,TIMER_CH_0,&timer_icinitpara);
            if (RmtSta & 1<<7)
            {
                if(Dval>300 && Dval<800)
                {
                    RmtRec>>=1; //右移一位
                    RmtRec|=0<<7; //接收到 0
                    iri++;
                }
                else if(Dval>1200 && Dval<1900)
                {
                    RmtRec>>=1; //右移一位
                    RmtRec|=1<<7; //接收到 1
                    iri++;
                }
                else if((Dval>4750 && Dval<5500))
                {
                    RmtSta &= ~(1<<7);
                }
                if(iri == 8)
                {
                    iri=0;
                    ir_rx_data[irj]=RmtRec;
                    irj++;
                    RmtRec = 0;
                }
                if(irj == IrCodeSize)
                {
                    for (iri=0; iri < IrCodeSize; iri++)
                    {
                        usart_data_transmit(UART4, ir_rx_data[iri]);
                        while(!usart_flag_get(UART4,USART_FLAG_TBE));
                    }
                }
            }
            else if((Dval > 4200 && Dval < 4700)&& (!(RmtSta & 1 << 7)))
            {
                RmtSta |= 1 << 7;
                RmtRec = 0;
                iri = 0;
            }
        }
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_CH0);
    }
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP))     //定时器超时，没有获捕获到任何信号
    {
        timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_RISING;
        timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
        timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
        timer_icinitpara.icfilter = 0;
        timer_input_capture_config(TIMER1,TIMER_CH_0,&timer_icinitpara);
        timer_interrupt_flag_clear(TIMER1,TIMER_INT_FLAG_UP);
        irj = 0;
        iri = 0;
        RmtSta &= ~(1<<7);
    }
}
