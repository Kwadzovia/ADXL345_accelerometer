/**
 ******************************************************************************
 * @file    adxl345.c
 * @brief   Simple Adxl345 accelerometer driver. Made to work with NucleoF446RE.
 * @author  Seli Kwadzovia
 ******************************************************************************
 */

#include "../Inc/adxl345.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include <spi.h>
#include <stdint.h>

HAL_StatusTypeDef adxl345_read_register(uint8_t *rx_data, uint8_t register_addr, uint32_t num_bytes)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t command_byte = register_addr;

    // Set Multi Byte Bit
    if (num_bytes == 0)
    {
        status = HAL_ERROR;
        return status;
    }
    else if (num_bytes == 1)
    {
        command_byte &= ~(ADXL345_MB_BIT);
    }
    else
    {
        command_byte |= ADXL345_MB_BIT;
    }

    // Set Read-Write Bit
    command_byte |= ADXL345_RW_BIT;

    // Start Transfer
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    status |= HAL_SPI_Transmit(&hspi2, &command_byte, 1, HAL_MAX_DELAY);
    status |= HAL_SPI_TransmitReceive(&hspi2, &command_byte, rx_data, num_bytes, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

    return status;
};

HAL_StatusTypeDef adxl345_write_register(uint8_t *input_data, uint8_t register_addr, uint32_t num_bytes)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t command_byte = register_addr;

    // Set Multi Byte Bit
    if (num_bytes == 0)
    {
        status = HAL_ERROR;
        return status;
    }
    else if (num_bytes == 1)
    {
        command_byte &= ~(ADXL345_MB_BIT);
    }
    else
    {
        command_byte |= ADXL345_MB_BIT;
    }

    // Set Read-Write Bit
    command_byte &= ~(ADXL345_RW_BIT);

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    status |= HAL_SPI_Transmit(&hspi2, &command_byte, 1, HAL_MAX_DELAY);
    status |= HAL_SPI_Transmit(&hspi2, input_data, num_bytes, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

    return status;
};

HAL_StatusTypeDef adxl345_read_acc(int16_t *rx_buffer, enum adxl345_acc_direction direction)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t temp_buffer[2];
    uint8_t command_addr;

    switch (direction)
    {
    case E_ADXL345_X_ACC:
        command_addr = ADXL345_CMD_X_ACC;
        break;
    case E_ADXL345_Y_ACC:
        command_addr = ADXL345_CMD_Y_ACC;
        break;
    case E_ADXL345_Z_ACC:
        command_addr = ADXL345_CMD_Z_ACC;
        break;
    default:
        status = HAL_ERROR;
        return status;
    }

    status |= adxl345_read_register(&temp_buffer[0], command_addr, 2);
    if (temp_buffer[1] & (1 << 7))
    {
        temp_buffer[0] = ~temp_buffer[0];
        temp_buffer[1] = ~temp_buffer[1];
        *rx_buffer = (int16_t)(-(temp_buffer[0] + (temp_buffer[1] << 8) + 1));
    }
    else
    {
        *rx_buffer = (int16_t)(temp_buffer[0] + (temp_buffer[1] << 8));
    }

    return status;
}

HAL_StatusTypeDef adxl345_init(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t input_value;

    // Turn Off Standby
    input_value = (1 << 3);
    status |= adxl345_write_register(&input_value, 0x2D, 1);

    return status;
}
