#include "max17043.h"

#include <stddef.h>

#define MAX17043_I2C_ADDRESS_7BIT          (0x36U)

#define MAX17043_REG_VCELL                 (0x02U)
#define MAX17043_REG_SOC                   (0x04U)
#define MAX17043_REG_VERSION               (0x08U)

#define MAX17043_REGISTER_SIZE_BYTES       (2U)

static GloveStatus_t Max17043_ReadRegister16(const Max17043Handle_t *handle,
                                             uint8_t reg_addr,
                                             uint16_t *value)
{
    uint8_t data[MAX17043_REGISTER_SIZE_BYTES];
    GloveStatus_t status;

    if ((handle == NULL) || (value == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = I2cBus_MemRead(handle->bus_id,
                            MAX17043_I2C_ADDRESS_7BIT,
                            reg_addr,
                            I2C_BUS_MEM_ADDR_SIZE_8BIT,
                            data,
                            MAX17043_REGISTER_SIZE_BYTES,
                            handle->timeout_ms);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    *value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    return GLOVE_STATUS_OK;
}

static uint16_t Max17043_ConvertVcellToMv(uint16_t raw_vcell)
{
    return (uint16_t)((((uint32_t)(raw_vcell >> 4)) * 125U) / 100U);
}

static uint16_t Max17043_ConvertSocToCentiPercent(uint16_t raw_soc)
{
    return (uint16_t)(((uint32_t)raw_soc * 100U) / 256U);
}

GloveStatus_t Max17043_Init(Max17043Handle_t *handle,
                            I2cBusId_t bus_id,
                            uint32_t timeout_ms)
{
    if (handle == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    handle->bus_id = bus_id;
    handle->timeout_ms = (timeout_ms == 0U) ? MAX17043_DEFAULT_TIMEOUT_MS : timeout_ms;

    return GLOVE_STATUS_OK;
}

GloveStatus_t Max17043_ReadBattery(const Max17043Handle_t *handle,
                                   Max17043BatteryData_t *data)
{
    uint16_t raw_vcell;
    uint16_t raw_soc;
    GloveStatus_t status;

    if (data == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = Max17043_ReadRegister16(handle, MAX17043_REG_VCELL, &raw_vcell);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Max17043_ReadRegister16(handle, MAX17043_REG_SOC, &raw_soc);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    data->raw_vcell = raw_vcell;
    data->raw_soc = raw_soc;
    data->voltage_mv = Max17043_ConvertVcellToMv(raw_vcell);
    data->soc_centi_percent = Max17043_ConvertSocToCentiPercent(raw_soc);
    data->soc_percent = (uint8_t)(raw_soc >> 8);
    data->reserved = 0U;

    return GLOVE_STATUS_OK;
}

GloveStatus_t Max17043_ReadVersion(const Max17043Handle_t *handle,
                                   uint16_t *version)
{
    return Max17043_ReadRegister16(handle, MAX17043_REG_VERSION, version);
}
