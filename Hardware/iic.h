#ifndef IIC_H
#define IIC_H

#include "gd32f30x_libopt.h"
#include <stdlib.h>

/*************************************************Hardware IIC*************************************************/

#define  I2C_SLAVE_ADDRESS7		0x52

typedef uint32_t I2C_TypeDef;

void bsp_iic_init(I2C_TypeDef I2Cx);
uint8_t VL6180_Read_Single_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint16_t Reg_Address);
void VL6180_Write_Single_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint16_t Reg_Address,uint8_t WriteValue);
uint8_t VL6180_Write_Multiple_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint8_t *pBuffer,uint8_t NumByteToWrite);
uint8_t VL6180_Read_Multiple_Register(I2C_TypeDef I2Cx, uint8_t SlaveAddress, uint8_t *pBuffer,uint8_t NumByteToRead);

/*************************************************Software IIC*************************************************/


/**
 * @defgroup iic iic function
 * @brief    iic function modules
 * @{
 */

/**
 * @brief  iic bus init
 * @return status code
 *         - 0 success
 * @note   SCL is PE7 and SDA is PE8
 */
uint8_t software_iic_init(void);

/**
 * @brief  iic bus deinit
 * @return status code
 *         - 0 success
 * @note   SCL is PE7 and SDA is PE8
 */
uint8_t software_iic_deinit(void);

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
uint8_t iic_write_cmd(uint8_t addr, uint8_t *buf, uint16_t len);

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
uint8_t iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

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
uint8_t iic_write_address16(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len);

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
uint8_t iic_read_cmd(uint8_t addr, uint8_t *buf, uint16_t len);

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
uint8_t iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

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
uint8_t iic_read_address16(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len);

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
uint8_t iic_read_address16_with_scl(uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len);

/**
 * @}
 */








#endif
