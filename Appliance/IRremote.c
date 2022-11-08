#include "IRremote.h"
#include "gd32f30x.h"
/**
* @breaf ����ҡ�������������Ϣ��������մ������Ľ����źţ��жϽ��յ�����0 ���� 1,
* �ƽ�ҡ���Ŀ�����
* 
* detailed ʹ�ö�ʱ���Ĳ����ܣ���ȡ������մ�����������������ȣ�us�ߵ�ƽ����
* ʵ�ַ���������ʱ����ʼ��Ϊ���������أ����������أ���ն�ʱ������ֵ������ʱ��
* ����Ϊ�����½��أ������½��أ�����ʱ��������ֵ������ʱ������Ϊ���������ء���
* �϶�ȡ���ļ�����ֵ 450���� Ϊ1�� 1600����Ϊ 0��4500����Ϊ�����룬��ͬ�ĺ����ź�
* ���뷽ʽ��ͬ��Ҫ���ݲ�ͬ���ź�����������
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

//ң��������״̬
//[7]:�յ����������־ [6]:�õ���һ��������������Ϣ
//[5]:���� [4]:����������Ƿ��Ѿ������� [3:0]:�����ʱ��
static uint8_t RmtSta = 0;
static uint16_t Dval; //�½���ʱ��������ֵ
static uint8_t RmtRec = 0; //������յ�������
static uint8_t RmtCnt = 0; //�������µĴ���
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
        if(gpio_input_bit_get(GPIOA, GPIO_PIN_15))        //��������û�أ���ȡGPIO PIN״̬��Ϊ1ʱ��������
        {
            timer_counter_value_config(TIMER1, 0);
            timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_FALLING;
            timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
            timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
            timer_icinitpara.icfilter    = 0;
            timer_input_capture_config(TIMER1,TIMER_CH_0,&timer_icinitpara);
        }
        else        ////�����½��أ�Ϊ0���½���
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
                    RmtRec>>=1; //����һλ
                    RmtRec|=0<<7; //���յ� 0
                    iri++;
                }
                else if(Dval>1200 && Dval<1900)
                {
                    RmtRec>>=1; //����һλ
                    RmtRec|=1<<7; //���յ� 1
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
    if(SET == timer_interrupt_flag_get(TIMER1,TIMER_INT_FLAG_UP))     //��ʱ����ʱ��û�л񲶻��κ��ź�
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
