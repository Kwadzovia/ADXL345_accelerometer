/**
 ******************************************************************************
 * @file    adxl345.h
 * @brief   This file contains all the function prototypes for
 *          the adxl345.c file
 * @author  Seli Kwadzovia
 ******************************************************************************
 */

#ifndef __ADXL345_H__
#define __ADXL345_H__

// Includes
#include "stm32f4xx_hal.h"
#include <stdint.h>

// Defines
#define ADXL345_RW_BIT (1 << 7) // SPI Read-Write Bit
#define ADXL345_MB_BIT (1 << 6) // SPI Multi-Byte Bit
#define ADXL345_CMD_X_ACC 0x32
#define ADXL345_CMD_Y_ACC 0x34
#define ADXL345_CMD_Z_ACC 0x36

// enums
enum adxl345_acc_direction
{
    E_ADXL345_X_ACC,
    E_ADXL345_Y_ACC,
    E_ADXL345_Z_ACC,
};

// Function Prototypes

/*
 * adxl345_read_register - Read single or multiple continuous registers
 *
 * @param   uint8_t *rx_data - pointer to receive buffer
 * @param   uint8_t register_addr - ADXL345 Command Address
 * @param   uint32_t num_bytes - number of bytes to transfer, does not include the register address
 *
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef adxl345_read_register(uint8_t *rx_data, uint8_t register_addr, uint32_t num_bytes);

/*
 * adxl345_write_register - Write single or multiple continuous registers
 *
 * @param   uint8_t *input_data - pointer to input data
 * @param   uint8_t register_addr - ADXL345 Command Address
 * @param   uint32_t num_bytes - number of bytes to transfer, does not include the register address
 *
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef adxl345_write_register(uint8_t *input_data, uint8_t register_addr, uint32_t num_bytes);

/*
 * adxl345_read_acc - Reads and converts acceleration from two's complement to decimal
 *
 * @param   int16_t *rx_buffer - pointer to store acceleration value
 * @param   enum adxl345_acc_direction - Acceleration direction
 *
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef adxl345_read_acc(int16_t *rx_buffer, enum adxl345_acc_direction direction);

/*
 * adxl345_init - Initialize ADXL345
 *
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef adxl345_init(void);

#endif // #ifndef __ADXL345_H__
