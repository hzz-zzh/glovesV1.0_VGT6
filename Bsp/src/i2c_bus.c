#include "i2c_bus.h"

#include <stddef.h>

#include "main.h"
#include "stm32h5xx_hal.h"

static I2C_HandleTypeDef *I2cBus_GetHandle(I2cBusId_t bus_id)
{
    if (bus_id == I2C_BUS_1)
    {
        return &hi2c1;
    }

    if (bus_id == I2C_BUS_2)
    {
        return &hi2c2;
    }

    return NULL;
}

static GloveStatus_t I2cBus_MapHalStatus(HAL_StatusTypeDef status)
{
    if (status == HAL_OK)
    {
        return GLOVE_STATUS_OK;
    }

    if (status == HAL_TIMEOUT)
    {
        return GLOVE_STATUS_TIMEOUT;
    }

    if (status == HAL_BUSY)
    {
        return GLOVE_STATUS_NOT_READY;
    }

    return GLOVE_STATUS_ERROR;
}

static GloveStatus_t I2cBus_MapMemAddrSize(uint16_t bus_mem_addr_size,
                                           uint16_t *hal_mem_addr_size)
{
    if (hal_mem_addr_size == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if (bus_mem_addr_size == I2C_BUS_MEM_ADDR_SIZE_8BIT)
    {
        *hal_mem_addr_size = I2C_MEMADD_SIZE_8BIT;
        return GLOVE_STATUS_OK;
    }

    if (bus_mem_addr_size == I2C_BUS_MEM_ADDR_SIZE_16BIT)
    {
        *hal_mem_addr_size = I2C_MEMADD_SIZE_16BIT;
        return GLOVE_STATUS_OK;
    }

    return GLOVE_STATUS_INVALID_PARAM;
}

GloveStatus_t I2cBus_MemRead(I2cBusId_t bus_id,
                             uint8_t device_addr_7bit,
                             uint16_t mem_addr,
                             uint16_t mem_addr_size,
                             uint8_t *data,
                             uint16_t size,
                             uint32_t timeout_ms)
{
    I2C_HandleTypeDef *hi2c;
    HAL_StatusTypeDef hal_status;
    GloveStatus_t status;
    uint16_t hal_mem_addr_size;

    hi2c = I2cBus_GetHandle(bus_id);
    if ((hi2c == NULL) || (data == NULL) || (size == 0U))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = I2cBus_MapMemAddrSize(mem_addr_size, &hal_mem_addr_size);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    hal_status = HAL_I2C_Mem_Read(hi2c,
                                  ((uint16_t)device_addr_7bit << 1),
                                  mem_addr,
                                  hal_mem_addr_size,
                                  data,
                                  size,
                                  timeout_ms);

    return I2cBus_MapHalStatus(hal_status);
}

GloveStatus_t I2cBus_MemWrite(I2cBusId_t bus_id,
                              uint8_t device_addr_7bit,
                              uint16_t mem_addr,
                              uint16_t mem_addr_size,
                              const uint8_t *data,
                              uint16_t size,
                              uint32_t timeout_ms)
{
    I2C_HandleTypeDef *hi2c;
    HAL_StatusTypeDef hal_status;
    GloveStatus_t status;
    uint16_t hal_mem_addr_size;

    hi2c = I2cBus_GetHandle(bus_id);
    if ((hi2c == NULL) || (data == NULL) || (size == 0U))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = I2cBus_MapMemAddrSize(mem_addr_size, &hal_mem_addr_size);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    hal_status = HAL_I2C_Mem_Write(hi2c,
                                   ((uint16_t)device_addr_7bit << 1),
                                   mem_addr,
                                   hal_mem_addr_size,
                                   (uint8_t *)data,
                                   size,
                                   timeout_ms);

    return I2cBus_MapHalStatus(hal_status);
}
