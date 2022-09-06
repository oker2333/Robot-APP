#ifndef IIC_H
#define IIC_H

#include "gd32f30x_libopt.h"
#include <stdlib.h>

#define  I2C_SLAVE_ADDRESS7		0x52

typedef uint32_t I2C_TypeDef;

void bsp_iic_init(I2C_TypeDef I2Cx);
uint8_t VL6180_Read_Single_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint16_t Reg_Address);
void VL6180_Write_Single_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint16_t Reg_Address,uint8_t WriteValue);
uint8_t VL6180_Write_Multiple_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint8_t *pBuffer,uint8_t NumByteToWrite);
uint8_t VL6180_Read_Multiple_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint8_t *pBuffer,uint8_t NumByteToRead);


#endif
