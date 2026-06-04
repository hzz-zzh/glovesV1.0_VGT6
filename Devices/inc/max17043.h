#ifndef MAX17043_H
#define MAX17043_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"
#include "i2c_bus.h"

#define MAX17043_DEFAULT_TIMEOUT_MS        (20U)

typedef struct
{
    I2cBusId_t bus_id;
    uint32_t timeout_ms;
} Max17043Handle_t;

typedef struct
{
    uint16_t raw_vcell;
    uint16_t raw_soc;
    uint16_t voltage_mv;
    uint16_t soc_centi_percent;
    uint8_t soc_percent;
    uint8_t reserved;
} Max17043BatteryData_t;

GloveStatus_t Max17043_Init(Max17043Handle_t *handle,
                            I2cBusId_t bus_id,
                            uint32_t timeout_ms);
GloveStatus_t Max17043_ReadBattery(const Max17043Handle_t *handle,
                                   Max17043BatteryData_t *data);
GloveStatus_t Max17043_ReadVersion(const Max17043Handle_t *handle,
                                   uint16_t *version);

#ifdef __cplusplus
}
#endif

#endif /* MAX17043_H */
