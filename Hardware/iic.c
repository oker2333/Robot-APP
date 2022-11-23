#include "iic.h"

/*************************************************Hardware IIC*************************************************/

#define POLLING_COUNT 1000
static uint32_t delay_counts = 0;

void bsp_iic_init(I2C_TypeDef I2Cx)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);	
	
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);	
	
    /* configure I2C0 clock */
    i2c_clock_config(I2Cx, 400000, I2C_DTCY_2);
    /* configure I2C0 address */
    i2c_mode_addr_config(I2Cx, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C_SLAVE_ADDRESS7);
    /* enable I2C0 */
    i2c_enable(I2Cx);
    /* enable acknowledge */
    i2c_ack_config(I2Cx, I2C_ACK_ENABLE);
}

uint8_t VL6180_Read_Single_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint16_t Reg_Address)
{
		uint8_t value = 0xFF;
    /* wait until I2C bus is idle */
		delay_counts = POLLING_COUNT;
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY) && delay_counts--);
		
		/* 1.开始*/
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);		

    /* wait until SBSEND bit is set */
	  delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND) && delay_counts--);		
		
		/* 2.设备地址 */
    /* send slave address to I2C bus*/
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set*/
	  delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND) && delay_counts--);
		
    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);		
		
		/* 3.寄存器地址*/
		/* send address 15...8*/
		i2c_data_transmit(I2Cx, Reg_Address >> 8);
		delay_counts = POLLING_COUNT;
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE) && delay_counts--);
		
		/* send address 7...0*/
		i2c_data_transmit(I2Cx, Reg_Address & 0xFF);
		delay_counts = POLLING_COUNT;
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE) && delay_counts--){}
		
		/* 4.停止 */
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2Cx);

		/* 5.开始 */
    /* wait until stop condition generate */ 
		delay_counts = POLLING_COUNT;
    while(I2C_CTL0(I2Cx)&0x0200 && delay_counts--);				
		
    /* wait until I2C bus is idle */
		delay_counts = POLLING_COUNT;
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY) && delay_counts--);
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);
    /* wait until SBSEND bit is set */
		delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND) && delay_counts--);

		/* 6.设备地址 */
    /* send slave address to I2C bus */
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_RECEIVER);

    /* wait until ADDSEND bit is set */
		delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND) && delay_counts--);
    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);
		
		/* 7.读取数据 */
		/* wait until the second last data byte is received into the shift register */
		delay_counts = POLLING_COUNT;
		while(!i2c_flag_get(I2Cx, I2C_FLAG_BTC) && delay_counts--);
		/* disable acknowledge */
		i2c_ack_config(I2Cx, I2C_ACK_DISABLE);
		/* wait until the RBNE bit is set */
		delay_counts = POLLING_COUNT;
		while(!i2c_flag_get(I2Cx, I2C_FLAG_RBNE) && delay_counts--);
		value = i2c_data_receive(I2Cx);

		/* 8.结束 */
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2Cx);
    /* wait until stop condition generate */
		delay_counts = POLLING_COUNT;
    while(I2C_CTL0(I2Cx)&0x0200 && delay_counts--);
    /* enable acknowledge */
    i2c_ack_config(I2Cx, I2C_ACK_ENABLE);
		
		return value;
}

void VL6180_Write_Single_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint16_t Reg_Address,uint8_t WriteValue)
{
    /* wait until I2C bus is idle */
		delay_counts = POLLING_COUNT;
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY) && delay_counts--);
		
		/* 1.开始*/
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);		

    /* wait until SBSEND bit is set */
		delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND) && delay_counts--);
		
		/* 2.设备地址 */
    /* send slave address to I2C bus*/
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set*/
	  delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND) && delay_counts--);

    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);
		
		/* 3.寄存器地址*/
		i2c_data_transmit(I2Cx, Reg_Address >> 8);
		delay_counts = POLLING_COUNT;
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE) && delay_counts--){}
		
		i2c_data_transmit(I2Cx, Reg_Address & 0xFF);
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE)){}
			
		/* 4.连续写数据 */
		/* send a data byte */
		i2c_data_transmit(I2Cx,WriteValue);
	
		/* wait until the transmission data register is empty*/
		delay_counts = POLLING_COUNT;
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE) && delay_counts--){}
		
		/* 5.停止 */
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2Cx);

    /* wait until stop condition generate */ 
		delay_counts = POLLING_COUNT;
    while(I2C_CTL0(I2Cx)&0x0200 && delay_counts--);		
}

uint8_t VL6180_Write_Multiple_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint8_t *pBuffer,uint8_t NumByteToWrite)
{
    /* wait until I2C bus is idle */
		delay_counts = POLLING_COUNT;
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY) && delay_counts--);
		
		/* 1.开始*/
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);		

    /* wait until SBSEND bit is set */
	  delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND) && delay_counts--);
		
		/* 2.设备地址 */
    /* send slave address to I2C bus*/
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set*/
	  delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND) && delay_counts--);

    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);
			
    /* 3.寄存器地址 && 连续写数据 */
    while(NumByteToWrite--)
    {
			/* send a data byte */
			i2c_data_transmit(I2Cx,*pBuffer);
		
			/* wait until the transmission data register is empty*/
			delay_counts = POLLING_COUNT;
			while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE) && delay_counts--){}
      pBuffer++;
    }
		
		/* 4.停止 */
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2Cx);

    /* wait until stop condition generate */ 
		delay_counts = POLLING_COUNT;
    while(I2C_CTL0(I2Cx)&0x0200 && delay_counts--);
		return 0;
}

uint8_t VL6180_Read_Multiple_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint8_t *pBuffer,uint8_t NumByteToRead)
{
		/* 1.开始 */
    /* wait until stop condition generate */ 
		delay_counts = POLLING_COUNT;
    while(I2C_CTL0(I2Cx)&0x0200 && delay_counts--);				
		
    /* wait until I2C bus is idle */
		delay_counts = POLLING_COUNT;
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY) && delay_counts--);
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);
    /* wait until SBSEND bit is set */
		delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND) && delay_counts--);

		/* 2.设备地址 */
    /* send slave address to I2C bus */
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_RECEIVER);

    /* wait until ADDSEND bit is set */
		delay_counts = POLLING_COUNT;
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND) && delay_counts--);
    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);
		
    /* 3.连续读数据 */
    while (NumByteToRead)
    {
        if(NumByteToRead==1)
        {
					  delay_counts = POLLING_COUNT;
						while(!i2c_flag_get(I2Cx, I2C_FLAG_BTC) && delay_counts--);
						/* disable acknowledge */
						i2c_ack_config(I2Cx, I2C_ACK_DISABLE);
        }
				delay_counts = POLLING_COUNT;
				while(!i2c_flag_get(I2Cx, I2C_FLAG_RBNE) && delay_counts--);
				*pBuffer++ = i2c_data_receive(I2Cx);
        NumByteToRead--;
    }

		/* 4.结束 */
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2Cx);
    /* wait until stop condition generate */
		delay_counts = POLLING_COUNT;
    while(I2C_CTL0(I2Cx)&0x0200 && delay_counts--);
    /* enable acknowledge */
    i2c_ack_config(I2Cx, I2C_ACK_ENABLE);
		
		return 0;
}
/********************************************delay*****************************************************************/
/**
 * @defgroup delay delay function
 * @brief    delay function modules
 * @{
 */

/**
 * @brief  delay clock init
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t delay_init(void)
{
	 
}

/**
 * @brief     delay us
 * @param[in] us
 * @note      none
 */
void delay_us(uint32_t us)
{
	 
}

/**
 * @brief     delay ms
 * @param[in] ms
 * @note      none
 */
void delay_ms(uint32_t ms)
{
	 
}

/*************************************************Software IIC*************************************************/

/**
 * @brief bit operate definition
 */
#define BITBAND(addr, bitnum)    ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2)) 
#define MEM_ADDR(addr)           *((unsigned long *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))

/**
 * @brief iic gpio operate definition
 */
#if 0
#define SCL_IN()          gpio_init(GPIOE, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,GPIO_PIN_7)
#define SCL_OUT()         gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7)
#define SDA_IN()          gpio_init(GPIOE, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,GPIO_PIN_8)
#define SDA_OUT()         gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8)
#else
#define SCL_IN()          {MEM_ADDR(GPIOE+0x00) &= ~(0x0F << (7 * 4)); MEM_ADDR(GPIOE+0x00) |= 0 << 7 * 4;}
#define SCL_OUT()         {MEM_ADDR(GPIOE+0x00) &= ~(0x0F << (7 * 4)); MEM_ADDR(GPIOE+0x00) |= 1 << 7 * 4;}
#define SDA_IN()          {MEM_ADDR(GPIOE+0x04) &= ~(0x0F << (0 * 4)); MEM_ADDR(GPIOE+0x04) |= 0 << 0 * 4;}
#define SDA_OUT()         {MEM_ADDR(GPIOE+0x04) &= ~(0x0F << (0 * 4)); MEM_ADDR(GPIOE+0x04) |= 1 << 0 * 4;}
#endif
#define IIC_SCL           BIT_ADDR(GPIOE+0x0C, 7)
#define IIC_SDA           BIT_ADDR(GPIOE+0x0C, 8)
#define READ_SCL          BIT_ADDR(GPIOE+0x08, 7)
#define READ_SDA          BIT_ADDR(GPIOE+0x08, 8)


/**
 * @brief  iic bus init
 * @return status code
 *         - 0 success
 * @note   SCL is PE7 and SDA is PE8
 */
uint8_t software_iic_init(void)
{
		rcu_periph_clock_enable(RCU_GPIOE);
	  rcu_periph_clock_enable(RCU_AF);
	  
		gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7 | GPIO_PIN_8);
    
    IIC_SDA = 1;
    IIC_SCL = 1;
	  
    return 0;
}

/**
 * @brief  iic bus deinit
 * @return status code
 *         - 0 success
 * @note   SCL is PE7 and SDA is PE8
 */
uint8_t software_iic_deinit(void)
{
    
    IIC_SDA = 1;
    IIC_SCL = 1;
	  
    return 0;
}

/**
 * @brief  iic bus send start
 * @note   none
 */
static void _iic_start(void)
{
    SDA_OUT();
    IIC_SDA = 1;
    IIC_SCL = 1;
    delay_us(4);
    IIC_SDA = 0;
    delay_us(4);
    IIC_SCL = 0;
}

/**
 * @brief  iic bus send stop
 * @note   none
 */
static void _iic_stop(void)
{
    SDA_OUT();
    IIC_SCL = 0;
    IIC_SDA = 0;
    delay_us(4);
    IIC_SCL = 1;
    delay_us(4);
    IIC_SDA = 1;
    delay_us(4);
}

/**
 * @brief  iic wait ack
 * @return status code
 *         - 0 get ack
 *         - 1 no ack
 * @note   none
 */
static uint8_t _iic_wait_ack(void)
{
    uint16_t uc_err_time = 0;
    
    SDA_IN();
    IIC_SDA = 1; 
    delay_us(1);
    IIC_SCL = 1; 
    delay_us(1);
    while (READ_SDA)
    {
        uc_err_time++;
        if (uc_err_time > 250)
        {
            _iic_stop();
            
            return 1;
        }
    }
    IIC_SCL = 0;
   
    return 0;
}

/**
 * @brief  iic bus send ack
 * @note   none
 */
static void _iic_ack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 0;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0;
}

/**
 * @brief  iic bus send nack
 * @note   none
 */
static void _iic_nack(void)
{
    IIC_SCL = 0;
    SDA_OUT();
    IIC_SDA = 1;
    delay_us(2);
    IIC_SCL = 1;
    delay_us(2);
    IIC_SCL = 0; 
}

/**
 * @brief     iic send one byte
 * @param[in] txd send byte
 * @note      none
 */
static void _iic_send_byte(uint8_t txd)
{
    uint8_t t;   
    
    SDA_OUT();
    IIC_SCL = 0;
    for (t=0; t<8; t++)
    {
        IIC_SDA = (txd&0x80) >> 7;
        txd <<= 1;
        delay_us(2);
        IIC_SCL = 1;
        delay_us(2);
        IIC_SCL = 0;
        delay_us(2);
    }
}

/**
 * @brief     iic read one byte
 * @param[in] ack is whether to send ack
 * @return    read byte
 * @note      none
 */
static uint8_t _iic_read_byte(uint8_t ack)
{
    uint8_t i,receive = 0;
    
    SDA_IN();
    for (i=0; i<8; i++)
    {
        IIC_SCL = 0;
        delay_us(2);
        IIC_SCL = 1;
        receive <<= 1;
        if (READ_SDA)
        {
            receive++;
        }
        delay_us(1);
    }
    if (!ack)
    {
        _iic_nack();
    }
    else
    {
        _iic_ack();
    }
    
    return receive;
}

/**
 * @brief     iic bus write command
 * @param[in] addr is iic device write address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      SCL is PE7 and SDA is PE8
 */
uint8_t iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{
    uint16_t i; 
    
    _iic_start();
    _iic_send_byte(addr);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    for (i=0; i<len; i++)
    {
        _iic_send_byte(buf[i]);
        if (_iic_wait_ack())
        {
            _iic_stop();
            
            return 1;
        }
    }
    _iic_stop();
    
    return 0;
} 

/**
 * @brief     iic bus write
 * @param[in] addr is iic device write address
 * @param[in] reg is iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      SCL is PE7 and SDA is PE8
 */
uint8_t iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i; 
  
    _iic_start();
    _iic_send_byte(addr);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    _iic_send_byte(reg);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    for (i=0; i<len; i++)
    {
        _iic_send_byte(buf[i]);
        if (_iic_wait_ack())
        {
            _iic_stop(); 
            
            return 1;
        }
    }
    _iic_stop();
    
    return 0;
} 

/**
 * @brief     iic bus write with 16 bits register address 
 * @param[in] addr is iic device write address
 * @param[in] reg is iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      SCL is PE7 and SDA is PE8
 */
uint8_t iic_write_address16(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i; 
  
    _iic_start();
    _iic_send_byte(addr);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    _iic_send_byte((reg>>8)&0xFF);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    _iic_send_byte(reg&0xFF);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    for (i=0; i<len; i++)
    {
        _iic_send_byte(buf[i]);
        if (_iic_wait_ack())
        {
            _iic_stop();
            
            return 1;
        }
    }
    _iic_stop();
    
    return 0;
} 

/**
 * @brief      iic bus read
 * @param[in]  addr is iic device write address
 * @param[in]  reg is iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       SCL is PE7 and SDA is PE8
 */
uint8_t iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{ 
    _iic_start();
    _iic_send_byte(addr);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    _iic_send_byte(reg);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    _iic_start();
    _iic_send_byte(addr+1);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    
    while (len)
    {
        if (len == 1)
        {
            *buf = _iic_read_byte(0);
        }
        else
        {
            *buf = _iic_read_byte(1);
        }
        len--;
        buf++;
    }
    _iic_stop();
    
    return 0;
}

/**
 * @brief      iic bus read with 16 bits register address 
 * @param[in]  addr is iic device write address
 * @param[in]  reg is iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       SCL is PE7 and SDA is PE8
 */
uint8_t iic_read_address16(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
{ 
    _iic_start();
    _iic_send_byte(addr);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    _iic_send_byte((reg>>8)&0xFF);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }    
    _iic_send_byte(reg&0xFF);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    _iic_start();
    _iic_send_byte(addr+1);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    while (len)
    {
        if (len == 1)
        {
            *buf = _iic_read_byte(0);
        }
        else
        {
            *buf = _iic_read_byte(1);
        }
        len--;
        buf++;
    }
    _iic_stop();
    
    return 0;
}

/**
 * @brief      iic bus read with 16 bits register address 
 * @param[in]  addr is iic device write address
 * @param[in]  reg is iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       SCL is PE7 and SDA is PE8
 */
uint8_t iic_read_address16_with_scl(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t cnt; 
    
    _iic_start();
    _iic_send_byte(addr);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    _iic_send_byte((reg >> 8) & 0xFF);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }    
    _iic_send_byte(reg & 0xFF);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    delay_ms(100);
    cnt = 1000;
    SCL_IN();
    while (cnt)
    {
        if (READ_SCL == 1)
        {
            break;
        }
        delay_ms(1);
        cnt--;
    }
    SCL_OUT();
    if (cnt == 0)
    {
        return 1;
    }
    _iic_start();
    _iic_send_byte(addr + 1);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    while (len)
    {
        if (len == 1)
        {
            *buf = _iic_read_byte(0);
        }
        else
        {
            *buf = _iic_read_byte(1);
        }
        len--;
        buf++;
    }
    _iic_stop();
    
    return 0;
}

/**
 * @brief      iic bus read command
 * @param[in]  addr is iic device write address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       SCL is PE7 and SDA is PE8
 */
uint8_t iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len)
{ 
    _iic_start();
    _iic_send_byte(addr + 1);
    if (_iic_wait_ack())
    {
        _iic_stop();
        
        return 1;
    }
    while (len)
    {
        if (len == 1)
        {
            *buf = _iic_read_byte(0);
        }
        else
        {
            *buf = _iic_read_byte(1); 
        }
        len--;
        buf++;
    }
    _iic_stop(); 
    
    return 0;
}













