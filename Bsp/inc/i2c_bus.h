#ifndef I2C_BUS_H
#define I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"

typedef enum
{
    I2C_BUS_1 = 0,
    I2C_BUS_2 = 1
} I2cBusId_t;

#define I2C_BUS_MEM_ADDR_SIZE_8BIT       (1U)
#define I2C_BUS_MEM_ADDR_SIZE_16BIT      (2U)

GloveStatus_t I2cBus_MemRead(I2cBusId_t bus_id,
                             uint8_t device_addr_7bit,
                             uint16_t mem_addr,
                             uint16_t mem_addr_size,
                             uint8_t *data,
                             uint16_t size,
                             uint32_t timeout_ms);

GloveStatus_t I2cBus_MemWrite(I2cBusId_t bus_id,
                              uint8_t device_addr_7bit,
                              uint16_t mem_addr,
                              uint16_t mem_addr_size,
                              const uint8_t *data,
                              uint16_t size,
                              uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* I2C_BUS_H */
