#include "iic.h"

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
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY));
		
		/* 1.开始*/
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);		

    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND));		
		
		/* 2.设备地址 */
    /* send slave address to I2C bus*/
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set*/
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND));
		
    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);		
		
		/* 3.寄存器地址*/
		/* send address 15...8*/
		i2c_data_transmit(I2Cx, Reg_Address >> 8);
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE));
		
		/* send address 7...0*/
		i2c_data_transmit(I2Cx, Reg_Address & 0xFF);
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE)){}
		
		/* 4.停止 */
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2Cx);

		/* 5.开始 */
    /* wait until stop condition generate */ 
    while(I2C_CTL0(I2Cx)&0x0200);				
		
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY));
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND));

		/* 6.设备地址 */
    /* send slave address to I2C bus */
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_RECEIVER);

    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);
		
		/* 7.读取数据 */
		/* wait until the second last data byte is received into the shift register */
		while(!i2c_flag_get(I2Cx, I2C_FLAG_BTC));
		/* disable acknowledge */
		i2c_ack_config(I2Cx, I2C_ACK_DISABLE);
		/* wait until the RBNE bit is set */
		while(!i2c_flag_get(I2Cx, I2C_FLAG_RBNE));
		value = i2c_data_receive(I2Cx);

		/* 8.结束 */
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2Cx);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2Cx)&0x0200);
    /* enable acknowledge */
    i2c_ack_config(I2Cx, I2C_ACK_ENABLE);
		
		return value;
}

void VL6180_Write_Single_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint16_t Reg_Address,uint8_t WriteValue)
{
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY));
		
		/* 1.开始*/
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);		

    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND));
		
		/* 2.设备地址 */
    /* send slave address to I2C bus*/
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set*/
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND));

    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);
		
		/* 3.寄存器地址*/
		i2c_data_transmit(I2Cx, Reg_Address >> 8);
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE)){}
		
		i2c_data_transmit(I2Cx, Reg_Address & 0xFF);
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE)){}
			
		/* 4.连续写数据 */
		/* send a data byte */
		i2c_data_transmit(I2Cx,WriteValue);
	
		/* wait until the transmission data register is empty*/
		while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE)){}
		
		/* 5.停止 */
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2Cx);

    /* wait until stop condition generate */ 
    while(I2C_CTL0(I2Cx)&0x0200);		
}

uint8_t VL6180_Write_Multiple_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint8_t *pBuffer,uint8_t NumByteToWrite)
{
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY));
		
		/* 1.开始*/
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);		

    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND));
		
		/* 2.设备地址 */
    /* send slave address to I2C bus*/
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_TRANSMITTER);

    /* wait until ADDSEND bit is set*/
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND));

    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);
			
    /* 3.寄存器地址 && 连续写数据 */
    while(NumByteToWrite--)
    {
			/* send a data byte */
			i2c_data_transmit(I2Cx,*pBuffer);
		
			/* wait until the transmission data register is empty*/
			while(!i2c_flag_get(I2Cx, I2C_FLAG_TBE)){}
      pBuffer++;
    }
		
		/* 4.停止 */
    /* send a stop condition to I2C bus*/
    i2c_stop_on_bus(I2Cx);

    /* wait until stop condition generate */ 
    while(I2C_CTL0(I2Cx)&0x0200);
		return 0;
}

uint8_t VL6180_Read_Multiple_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint8_t *pBuffer,uint8_t NumByteToRead)
{
		/* 1.开始 */
    /* wait until stop condition generate */ 
    while(I2C_CTL0(I2Cx)&0x0200);				
		
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2Cx, I2C_FLAG_I2CBSY));
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2Cx);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2Cx, I2C_FLAG_SBSEND));

		/* 2.设备地址 */
    /* send slave address to I2C bus */
    i2c_master_addressing(I2Cx, SlaveAddress, I2C_RECEIVER);

    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(I2Cx, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2Cx, I2C_FLAG_ADDSEND);
		
    /* 3.连续读数据 */
    while (NumByteToRead)
    {
        if(NumByteToRead==1)
        {
						while(!i2c_flag_get(I2Cx, I2C_FLAG_BTC));
						/* disable acknowledge */
						i2c_ack_config(I2Cx, I2C_ACK_DISABLE);
        }
				while(!i2c_flag_get(I2Cx, I2C_FLAG_RBNE));
				*pBuffer++ = i2c_data_receive(I2Cx);
        NumByteToRead--;
    }

		/* 4.结束 */
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2Cx);
    /* wait until stop condition generate */
    while(I2C_CTL0(I2Cx)&0x0200);
    /* enable acknowledge */
    i2c_ack_config(I2Cx, I2C_ACK_ENABLE);
		
		return 0;
}
