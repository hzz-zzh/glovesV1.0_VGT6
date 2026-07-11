#include "bq25622.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define BQ25622_DEBUG_ENABLE                      (0U)
#define BQ25622_I2C_ADDRESS_7BIT                  (0x6BU)

#define BQ25622_REG_CHARGE_CURRENT_LIMIT          (0x02U)
#define BQ25622_REG_CHARGE_VOLTAGE_LIMIT          (0x04U)
#define BQ25622_REG_INPUT_CURRENT_LIMIT           (0x06U)
#define BQ25622_REG_TERMINATION_CONTROL            (0x12U)
#define BQ25622_REG_CHARGE_CONTROL_0                (0x14U)
#define BQ25622_REG_CHARGE_TIMER_CONTROL            (0x15U)
#define BQ25622_REG_CHARGER_CONTROL_1             (0x16U)
#define BQ25622_REG_CHARGER_CONTROL_4             (0x19U)
#define BQ25622_REG_CHARGER_STATUS_0              (0x1DU)
#define BQ25622_REG_CHARGER_STATUS_1              (0x1EU)
#define BQ25622_REG_FAULT_STATUS_0                (0x1FU)
#define BQ25622_REG_ADC_CONTROL                   (0x26U)
#define BQ25622_REG_IBUS_ADC                      (0x28U)
#define BQ25622_REG_IBAT_ADC                      (0x2AU)
#define BQ25622_REG_VBUS_ADC                      (0x2CU)
#define BQ25622_REG_VBAT_ADC                      (0x30U)

#define BQ25622_REGISTER_SIZE_BYTES               (2U)

#define BQ25622_CHARGE_CURRENT_MIN_MA             (80U)
#define BQ25622_CHARGE_CURRENT_MAX_MA             (3520U)
#define BQ25622_CHARGE_CURRENT_STEP_MA            (80U)
#define BQ25622_CHARGE_CURRENT_CODE_LSB_MA        (80U)
#define BQ25622_CHARGE_CURRENT_REG_SHIFT          (6U)
#define BQ25622_CHARGE_CURRENT_FIELD_MASK         (0x3FU)

#define BQ25622_CHARGE_VOLTAGE_MIN_MV             (3500U)
#define BQ25622_CHARGE_VOLTAGE_MAX_MV             (4800U)
#define BQ25622_CHARGE_VOLTAGE_STEP_MV            (10U)
#define BQ25622_CHARGE_VOLTAGE_CODE_LSB_MV        (10U)
#define BQ25622_CHARGE_VOLTAGE_REG_SHIFT          (3U)
#define BQ25622_CHARGE_VOLTAGE_FIELD_MASK         (0x1FFU)

#define BQ25622_INPUT_CURRENT_MIN_MA              (100U)
#define BQ25622_INPUT_CURRENT_MAX_MA              (3200U)
#define BQ25622_INPUT_CURRENT_STEP_MA             (20U)
#define BQ25622_INPUT_CURRENT_CODE_LSB_MA         (20U)
#define BQ25622_INPUT_CURRENT_REG_SHIFT           (4U)
#define BQ25622_INPUT_CURRENT_FIELD_MASK          (0xFFU)

#define BQ25622_CHARGER_CONTROL_1_WATCHDOG_MASK   (0x03U)
#define BQ25622_CHARGER_CONTROL_4_EN_EXTILIM_MASK (0x04U)
#define BQ25622_ADC_ENABLE_12BIT_AVERAGE           (0x8CU)
#define BQ25622_TERMINATION_CURRENT_MIN_MA         (10U)
#define BQ25622_TERMINATION_CURRENT_MAX_MA         (620U)
#define BQ25622_TERMINATION_CURRENT_STEP_MA        (10U)

#define BQ25622_DEBUG_PRINTF(...)                 \
    do                                            \
    {                                             \
        if (BQ25622_DEBUG_ENABLE != 0U)           \
        {                                         \
            printf(__VA_ARGS__);                  \
        }                                         \
    } while (0)

static GloveStatus_t Bq25622_ReadRegister8(const Bq25622Handle_t *handle,
                                           uint8_t reg_addr,
                                           uint8_t *value)
{
    GloveStatus_t status;

    if ((handle == NULL) || (value == NULL))
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] read8 failed: invalid param\r\n");
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = I2cBus_MemRead(handle->bus_id,
                            BQ25622_I2C_ADDRESS_7BIT,
                            reg_addr,
                            I2C_BUS_MEM_ADDR_SIZE_8BIT,
                            value,
                            1U,
                            handle->timeout_ms);

    if (status != GLOVE_STATUS_OK)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] read8 reg=0x%02X status=%u\r\n",
               (unsigned int)reg_addr,
               (unsigned int)status);
    }

    return status;
}

static GloveStatus_t Bq25622_WriteRegister8(const Bq25622Handle_t *handle,
                                            uint8_t reg_addr,
                                            uint8_t value)
{
    GloveStatus_t status;

    if (handle == NULL)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] write8 failed: handle is NULL\r\n");
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = I2cBus_MemWrite(handle->bus_id,
                             BQ25622_I2C_ADDRESS_7BIT,
                             reg_addr,
                             I2C_BUS_MEM_ADDR_SIZE_8BIT,
                             &value,
                             1U,
                             handle->timeout_ms);

    if (status != GLOVE_STATUS_OK)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] write8 reg=0x%02X value=0x%02X status=%u\r\n",
               (unsigned int)reg_addr,
               (unsigned int)value,
               (unsigned int)status);
    }

    return status;
}

static GloveStatus_t Bq25622_ReadRegister16(const Bq25622Handle_t *handle,
                                            uint8_t reg_addr,
                                            uint16_t *value)
{
    uint8_t data[BQ25622_REGISTER_SIZE_BYTES];
    GloveStatus_t status;

    if ((handle == NULL) || (value == NULL))
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] read16 failed: invalid param\r\n");
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = I2cBus_MemRead(handle->bus_id,
                            BQ25622_I2C_ADDRESS_7BIT,
                            reg_addr,
                            I2C_BUS_MEM_ADDR_SIZE_8BIT,
                            data,
                            BQ25622_REGISTER_SIZE_BYTES,
                            handle->timeout_ms);
    if (status != GLOVE_STATUS_OK)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] read16 reg=0x%02X status=%u\r\n",
               (unsigned int)reg_addr,
               (unsigned int)status);
        return status;
    }

    *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

    return GLOVE_STATUS_OK;
}

static GloveStatus_t Bq25622_WriteRegister16(const Bq25622Handle_t *handle,
                                             uint8_t reg_addr,
                                             uint16_t value)
{
    uint8_t data[BQ25622_REGISTER_SIZE_BYTES];
    GloveStatus_t status;

    if (handle == NULL)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] write failed: handle is NULL\r\n");
        return GLOVE_STATUS_INVALID_PARAM;
    }

    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);

    status = I2cBus_MemWrite(handle->bus_id,
                             BQ25622_I2C_ADDRESS_7BIT,
                             reg_addr,
                             I2C_BUS_MEM_ADDR_SIZE_8BIT,
                             data,
                             BQ25622_REGISTER_SIZE_BYTES,
                             handle->timeout_ms);
    if (status != GLOVE_STATUS_OK)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] write16 reg=0x%02X value=0x%04X status=%u\r\n",
               (unsigned int)reg_addr,
               (unsigned int)value,
               (unsigned int)status);
    }

    return status;
}

GloveStatus_t Bq25622_Init(Bq25622Handle_t *handle,
                           I2cBusId_t bus_id,
                           uint32_t timeout_ms)
{
    if (handle == NULL)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] init failed: handle is NULL\r\n");
        return GLOVE_STATUS_INVALID_PARAM;
    }

    handle->bus_id = bus_id;
    handle->timeout_ms = (timeout_ms == 0U) ? BQ25622_DEFAULT_TIMEOUT_MS : timeout_ms;

    return GLOVE_STATUS_OK;
}

GloveStatus_t Bq25622_SetChargeCurrentLimitMa(const Bq25622Handle_t *handle,
                                              uint16_t current_ma)
{
    uint16_t charge_current_code;
    uint16_t reg_value;
    uint16_t readback_reg_value;
    uint16_t readback_charge_current_code;
    uint16_t readback_current_ma;
    uint8_t low_byte;
    uint8_t high_byte;
    GloveStatus_t status;

    if ((current_ma < BQ25622_CHARGE_CURRENT_MIN_MA) ||
        (current_ma > BQ25622_CHARGE_CURRENT_MAX_MA) ||
        ((current_ma % BQ25622_CHARGE_CURRENT_STEP_MA) != 0U))
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] invalid charge current=%u mA, range=%u-%u mA, step=%u mA\r\n",
               (unsigned int)current_ma,
               (unsigned int)BQ25622_CHARGE_CURRENT_MIN_MA,
               (unsigned int)BQ25622_CHARGE_CURRENT_MAX_MA,
               (unsigned int)BQ25622_CHARGE_CURRENT_STEP_MA);
        return GLOVE_STATUS_INVALID_PARAM;
    }

    charge_current_code = (uint16_t)(current_ma / BQ25622_CHARGE_CURRENT_CODE_LSB_MA);
    reg_value = (uint16_t)(charge_current_code << BQ25622_CHARGE_CURRENT_REG_SHIFT);
    low_byte = (uint8_t)(reg_value & 0xFFU);
    high_byte = (uint8_t)((reg_value >> 8) & 0xFFU);

    BQ25622_DEBUG_PRINTF("[BQ25622] charge current target=%u mA reg=0x%04X data={0x%02X,0x%02X}\r\n",
           (unsigned int)current_ma,
           (unsigned int)reg_value,
           (unsigned int)low_byte,
           (unsigned int)high_byte);

    status = Bq25622_WriteRegister16(handle, BQ25622_REG_CHARGE_CURRENT_LIMIT, reg_value);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister16(handle,
                                    BQ25622_REG_CHARGE_CURRENT_LIMIT,
                                    &readback_reg_value);
    if (status == GLOVE_STATUS_OK)
    {
        readback_charge_current_code =
            (uint16_t)((readback_reg_value >> BQ25622_CHARGE_CURRENT_REG_SHIFT) &
                       BQ25622_CHARGE_CURRENT_FIELD_MASK);
        readback_current_ma =
            (uint16_t)(readback_charge_current_code * BQ25622_CHARGE_CURRENT_CODE_LSB_MA);

        BQ25622_DEBUG_PRINTF("[BQ25622] charge current readback reg=0x%04X current=%u mA\r\n",
               (unsigned int)readback_reg_value,
               (unsigned int)readback_current_ma);
        if (readback_current_ma != current_ma)
        {
            return GLOVE_STATUS_ERROR;
        }
    }

    return status;
}

GloveStatus_t Bq25622_SetTerminationCurrentMa(const Bq25622Handle_t *handle,
                                              uint16_t current_ma)
{
    uint16_t reg_value;
    uint16_t readback;
    GloveStatus_t status;

    if ((current_ma < BQ25622_TERMINATION_CURRENT_MIN_MA) ||
        (current_ma > BQ25622_TERMINATION_CURRENT_MAX_MA) ||
        ((current_ma % BQ25622_TERMINATION_CURRENT_STEP_MA) != 0U))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    reg_value = (uint16_t)((current_ma / BQ25622_TERMINATION_CURRENT_STEP_MA) << 3);
    status = Bq25622_WriteRegister16(handle, BQ25622_REG_TERMINATION_CONTROL, reg_value);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister16(handle, BQ25622_REG_TERMINATION_CONTROL, &readback);
    if ((status == GLOVE_STATUS_OK) && ((readback & 0x01F8U) != (reg_value & 0x01F8U)))
    {
        return GLOVE_STATUS_ERROR;
    }
    return status;
}

GloveStatus_t Bq25622_EnableAdc(const Bq25622Handle_t *handle)
{
    uint8_t readback;
    GloveStatus_t status;

    /* 连续12位采样并启用滑动平均，降低负载瞬态对状态判断的影响。 */
    status = Bq25622_WriteRegister8(handle,
                                    BQ25622_REG_ADC_CONTROL,
                                    BQ25622_ADC_ENABLE_12BIT_AVERAGE);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }
    status = Bq25622_ReadRegister8(handle, BQ25622_REG_ADC_CONTROL, &readback);
    if ((status == GLOVE_STATUS_OK) &&
        ((readback & 0xF8U) != (BQ25622_ADC_ENABLE_12BIT_AVERAGE & 0xF8U)))
    {
        return GLOVE_STATUS_ERROR;
    }
    return status;
}

GloveStatus_t Bq25622_EnableChargeSafety(const Bq25622Handle_t *handle)
{
    uint8_t control0;
    uint8_t timer_control;
    uint8_t charger_control1;
    GloveStatus_t status;

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGE_CONTROL_0, &control0);
    if (status != GLOVE_STATUS_OK) return status;
    control0 |= 0x04U; /* EN_TERM */
    status = Bq25622_WriteRegister8(handle, BQ25622_REG_CHARGE_CONTROL_0, control0);
    if (status != GLOVE_STATUS_OK) return status;

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGE_TIMER_CONTROL, &timer_control);
    if (status != GLOVE_STATUS_OK) return status;
    timer_control = (uint8_t)((timer_control | 0x04U) & (uint8_t)(~0x01U));
    status = Bq25622_WriteRegister8(handle, BQ25622_REG_CHARGE_TIMER_CONTROL, timer_control);
    if (status != GLOVE_STATUS_OK) return status;

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_1, &charger_control1);
    if (status != GLOVE_STATUS_OK) return status;
    charger_control1 |= 0x20U; /* EN_CHG */
    charger_control1 &= (uint8_t)(~BQ25622_CHARGER_CONTROL_1_WATCHDOG_MASK);
    status = Bq25622_WriteRegister8(handle, BQ25622_REG_CHARGER_CONTROL_1, charger_control1);
    if (status != GLOVE_STATUS_OK) return status;

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGE_CONTROL_0, &control0);
    if ((status != GLOVE_STATUS_OK) || ((control0 & 0x04U) == 0U)) return GLOVE_STATUS_ERROR;
    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGE_TIMER_CONTROL, &timer_control);
    if ((status != GLOVE_STATUS_OK) || ((timer_control & 0x05U) != 0x04U)) return GLOVE_STATUS_ERROR;
    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_1, &charger_control1);
    if ((status != GLOVE_STATUS_OK) || ((charger_control1 & 0x23U) != 0x20U)) return GLOVE_STATUS_ERROR;
    return GLOVE_STATUS_OK;
}

static int16_t Bq25622_SignExtend(uint16_t value, uint8_t width)
{
    uint16_t sign_bit = (uint16_t)(1UL << (width - 1U));
    uint16_t mask = (uint16_t)((1UL << width) - 1UL);

    value &= mask;
    if ((value & sign_bit) != 0U)
    {
        value |= (uint16_t)(~mask);
    }
    return (int16_t)value;
}

GloveStatus_t Bq25622_ReadStatusSnapshot(const Bq25622Handle_t *handle,
                                         Bq25622StatusSnapshot_t *snapshot)
{
    uint16_t ibus_raw;
    uint16_t ibat_raw;
    uint16_t vbus_raw;
    uint16_t vbat_raw;
    uint16_t ibat_code;
    GloveStatus_t status;

    if ((handle == NULL) || (snapshot == NULL))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }
    (void)memset(snapshot, 0, sizeof(*snapshot));

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_STATUS_0,
                                   &snapshot->charger_status0);
    if (status != GLOVE_STATUS_OK) return status;
    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_STATUS_1,
                                   &snapshot->charge_status);
    if (status != GLOVE_STATUS_OK) return status;
    snapshot->vbus_status = (uint8_t)(snapshot->charge_status & 0x07U);
    snapshot->charge_status = (uint8_t)((snapshot->charge_status >> 3) & 0x03U);
    snapshot->vbus_present = ((snapshot->vbus_status != 0U) &&
                              (snapshot->vbus_status != 7U)) ? 1U : 0U;

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_FAULT_STATUS_0,
                                   &snapshot->fault_status0);
    if (status != GLOVE_STATUS_OK) return status;
    snapshot->temperature_status = (uint8_t)(snapshot->fault_status0 & 0x07U);

    status = Bq25622_ReadRegister16(handle, BQ25622_REG_IBUS_ADC, &ibus_raw);
    if (status != GLOVE_STATUS_OK) return status;
    status = Bq25622_ReadRegister16(handle, BQ25622_REG_IBAT_ADC, &ibat_raw);
    if (status != GLOVE_STATUS_OK) return status;
    status = Bq25622_ReadRegister16(handle, BQ25622_REG_VBUS_ADC, &vbus_raw);
    if (status != GLOVE_STATUS_OK) return status;
    status = Bq25622_ReadRegister16(handle, BQ25622_REG_VBAT_ADC, &vbat_raw);
    if (status != GLOVE_STATUS_OK) return status;

    ibat_code = (uint16_t)(ibat_raw >> 2);
    snapshot->input_current_ma = (int16_t)(Bq25622_SignExtend((uint16_t)(ibus_raw >> 1), 15U) * 2);
    snapshot->battery_current_ma = (int16_t)(Bq25622_SignExtend(ibat_code, 14U) * 4);
    snapshot->current_valid = (ibat_code == 0x2000U) ? 0U : 1U;
    snapshot->vbus_voltage_mv = (uint16_t)((((uint32_t)(vbus_raw >> 2) * 397U) + 50U) / 100U);
    snapshot->battery_voltage_mv = (uint16_t)((((uint32_t)((vbat_raw >> 1) & 0x0FFFU) * 199U) + 50U) / 100U);
    return GLOVE_STATUS_OK;
}

GloveStatus_t Bq25622_DisableWatchdog(const Bq25622Handle_t *handle)
{
    uint8_t value;
    uint8_t new_value;
    uint8_t readback;
    GloveStatus_t status;

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_1, &value);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    new_value = (uint8_t)(value & (uint8_t)(~BQ25622_CHARGER_CONTROL_1_WATCHDOG_MASK));

    if (new_value != value)
    {
        status = Bq25622_WriteRegister8(handle, BQ25622_REG_CHARGER_CONTROL_1, new_value);
        if (status != GLOVE_STATUS_OK)
        {
            return status;
        }
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_1, &readback);
    if (status == GLOVE_STATUS_OK)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] watchdog off ctrl1=0x%02X->0x%02X\r\n",
               (unsigned int)value,
               (unsigned int)readback);
        if ((readback & BQ25622_CHARGER_CONTROL_1_WATCHDOG_MASK) != 0U)
        {
            return GLOVE_STATUS_ERROR;
        }
    }

    return status;
}

GloveStatus_t Bq25622_EnableExternalIlim(const Bq25622Handle_t *handle)
{
    uint8_t value;
    uint8_t new_value;
    uint8_t readback;
    GloveStatus_t status;

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_4, &value);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    new_value = (uint8_t)(value | BQ25622_CHARGER_CONTROL_4_EN_EXTILIM_MASK);

    if (new_value != value)
    {
        status = Bq25622_WriteRegister8(handle, BQ25622_REG_CHARGER_CONTROL_4, new_value);
        if (status != GLOVE_STATUS_OK)
        {
            return status;
        }
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_4, &readback);
    if (status == GLOVE_STATUS_OK)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] external ILIM on ctrl4=0x%02X->0x%02X ext_ilim=%u\r\n",
               (unsigned int)value,
               (unsigned int)readback,
               (unsigned int)((readback & BQ25622_CHARGER_CONTROL_4_EN_EXTILIM_MASK) != 0U));
        if ((readback & BQ25622_CHARGER_CONTROL_4_EN_EXTILIM_MASK) == 0U)
        {
            return GLOVE_STATUS_ERROR;
        }
    }

    return status;
}

GloveStatus_t Bq25622_DisableExternalIlim(const Bq25622Handle_t *handle)
{
    uint8_t value;
    uint8_t new_value;
    uint8_t readback;
    GloveStatus_t status;

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_4, &value);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    new_value = (uint8_t)(value & (uint8_t)(~BQ25622_CHARGER_CONTROL_4_EN_EXTILIM_MASK));

    if (new_value != value)
    {
        status = Bq25622_WriteRegister8(handle, BQ25622_REG_CHARGER_CONTROL_4, new_value);
        if (status != GLOVE_STATUS_OK)
        {
            return status;
        }
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_4, &readback);
    if (status == GLOVE_STATUS_OK)
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] external ILIM off ctrl4=0x%02X->0x%02X ext_ilim=%u\r\n",
               (unsigned int)value,
               (unsigned int)readback,
               (unsigned int)((readback & BQ25622_CHARGER_CONTROL_4_EN_EXTILIM_MASK) != 0U));
    }

    return status;
}

GloveStatus_t Bq25622_SetInputCurrentLimitMa(const Bq25622Handle_t *handle,
                                             uint16_t current_ma)
{
    uint16_t input_current_code;
    uint16_t reg_value;
    uint16_t readback_reg_value;
    uint16_t readback_input_current_code;
    uint16_t readback_current_ma;
    uint8_t low_byte;
    uint8_t high_byte;
    GloveStatus_t status;

    if ((current_ma < BQ25622_INPUT_CURRENT_MIN_MA) ||
        (current_ma > BQ25622_INPUT_CURRENT_MAX_MA) ||
        ((current_ma % BQ25622_INPUT_CURRENT_STEP_MA) != 0U))
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] invalid input current=%u mA, range=%u-%u mA, step=%u mA\r\n",
               (unsigned int)current_ma,
               (unsigned int)BQ25622_INPUT_CURRENT_MIN_MA,
               (unsigned int)BQ25622_INPUT_CURRENT_MAX_MA,
               (unsigned int)BQ25622_INPUT_CURRENT_STEP_MA);
        return GLOVE_STATUS_INVALID_PARAM;
    }

    input_current_code = (uint16_t)(current_ma / BQ25622_INPUT_CURRENT_CODE_LSB_MA);
    reg_value = (uint16_t)(input_current_code << BQ25622_INPUT_CURRENT_REG_SHIFT);
    low_byte = (uint8_t)(reg_value & 0xFFU);
    high_byte = (uint8_t)((reg_value >> 8) & 0xFFU);

    BQ25622_DEBUG_PRINTF("[BQ25622] input current target=%u mA reg=0x%04X data={0x%02X,0x%02X}\r\n",
           (unsigned int)current_ma,
           (unsigned int)reg_value,
           (unsigned int)low_byte,
           (unsigned int)high_byte);

    status = Bq25622_WriteRegister16(handle, BQ25622_REG_INPUT_CURRENT_LIMIT, reg_value);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister16(handle,
                                    BQ25622_REG_INPUT_CURRENT_LIMIT,
                                    &readback_reg_value);
    if (status == GLOVE_STATUS_OK)
    {
        readback_input_current_code =
            (uint16_t)((readback_reg_value >> BQ25622_INPUT_CURRENT_REG_SHIFT) &
                       BQ25622_INPUT_CURRENT_FIELD_MASK);
        readback_current_ma =
            (uint16_t)(readback_input_current_code * BQ25622_INPUT_CURRENT_CODE_LSB_MA);

        BQ25622_DEBUG_PRINTF("[BQ25622] input current readback reg=0x%04X current=%u mA\r\n",
               (unsigned int)readback_reg_value,
               (unsigned int)readback_current_ma);
        if (readback_current_ma != current_ma)
        {
            return GLOVE_STATUS_ERROR;
        }
    }

    return status;
}

GloveStatus_t Bq25622_SetChargeVoltageLimitMv(const Bq25622Handle_t *handle,
                                              uint16_t voltage_mv)
{
    uint16_t charge_voltage_code;
    uint16_t reg_value;
    uint16_t readback_reg_value;
    uint16_t readback_charge_voltage_code;
    uint16_t readback_voltage_mv;
    uint8_t low_byte;
    uint8_t high_byte;
    GloveStatus_t status;

    if ((voltage_mv < BQ25622_CHARGE_VOLTAGE_MIN_MV) ||
        (voltage_mv > BQ25622_CHARGE_VOLTAGE_MAX_MV) ||
        ((voltage_mv % BQ25622_CHARGE_VOLTAGE_STEP_MV) != 0U))
    {
        BQ25622_DEBUG_PRINTF("[BQ25622] invalid charge voltage=%u mV, range=%u-%u mV, step=%u mV\r\n",
               (unsigned int)voltage_mv,
               (unsigned int)BQ25622_CHARGE_VOLTAGE_MIN_MV,
               (unsigned int)BQ25622_CHARGE_VOLTAGE_MAX_MV,
               (unsigned int)BQ25622_CHARGE_VOLTAGE_STEP_MV);
        return GLOVE_STATUS_INVALID_PARAM;
    }

    charge_voltage_code = (uint16_t)(voltage_mv / BQ25622_CHARGE_VOLTAGE_CODE_LSB_MV);
    reg_value = (uint16_t)(charge_voltage_code << BQ25622_CHARGE_VOLTAGE_REG_SHIFT);
    low_byte = (uint8_t)(reg_value & 0xFFU);
    high_byte = (uint8_t)((reg_value >> 8) & 0xFFU);

    BQ25622_DEBUG_PRINTF("[BQ25622] charge voltage target=%u mV reg=0x%04X data={0x%02X,0x%02X}\r\n",
           (unsigned int)voltage_mv,
           (unsigned int)reg_value,
           (unsigned int)low_byte,
           (unsigned int)high_byte);

    status = Bq25622_WriteRegister16(handle, BQ25622_REG_CHARGE_VOLTAGE_LIMIT, reg_value);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister16(handle,
                                    BQ25622_REG_CHARGE_VOLTAGE_LIMIT,
                                    &readback_reg_value);
    if (status == GLOVE_STATUS_OK)
    {
        readback_charge_voltage_code =
            (uint16_t)((readback_reg_value >> BQ25622_CHARGE_VOLTAGE_REG_SHIFT) &
                       BQ25622_CHARGE_VOLTAGE_FIELD_MASK);
        readback_voltage_mv =
            (uint16_t)(readback_charge_voltage_code * BQ25622_CHARGE_VOLTAGE_CODE_LSB_MV);

        BQ25622_DEBUG_PRINTF("[BQ25622] charge voltage readback reg=0x%04X voltage=%u mV\r\n",
               (unsigned int)readback_reg_value,
               (unsigned int)readback_voltage_mv);
        if (readback_voltage_mv != voltage_mv)
        {
            return GLOVE_STATUS_ERROR;
        }
    }

    return status;
}

GloveStatus_t Bq25622_ReadChargeCurrentLimitMa(const Bq25622Handle_t *handle,
                                               uint16_t *current_ma)
{
    uint16_t reg_value;
    uint16_t charge_current_code;
    GloveStatus_t status;

    if (current_ma == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = Bq25622_ReadRegister16(handle, BQ25622_REG_CHARGE_CURRENT_LIMIT, &reg_value);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    charge_current_code = (uint16_t)((reg_value >> BQ25622_CHARGE_CURRENT_REG_SHIFT) &
                                     BQ25622_CHARGE_CURRENT_FIELD_MASK);
    *current_ma = (uint16_t)(charge_current_code * BQ25622_CHARGE_CURRENT_CODE_LSB_MA);

    return GLOVE_STATUS_OK;
}

GloveStatus_t Bq25622_PrintChargeStatus(const Bq25622Handle_t *handle)
{
    uint8_t status0;
    uint8_t status1;
    uint8_t fault0;
    GloveStatus_t status;

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_STATUS_0, &status0);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_STATUS_1, &status1);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_FAULT_STATUS_0, &fault0);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    BQ25622_DEBUG_PRINTF("[BQ25622] status STAT0=0x%02X treg=%u vsys=%u iindpm_or_ilim=%u vindpm=%u safety=%u wd=%u STAT1=0x%02X chg=%u vbus=%u FAULT0=0x%02X ts=%u vbus_fault=%u bat_fault=%u sys_fault=%u tshut=%u\r\n",
           (unsigned int)status0,
           (unsigned int)((status0 >> 5) & 0x01U),
           (unsigned int)((status0 >> 4) & 0x01U),
           (unsigned int)((status0 >> 3) & 0x01U),
           (unsigned int)((status0 >> 2) & 0x01U),
           (unsigned int)((status0 >> 1) & 0x01U),
           (unsigned int)(status0 & 0x01U),
           (unsigned int)status1,
           (unsigned int)((status1 >> 3) & 0x03U),
           (unsigned int)(status1 & 0x07U),
           (unsigned int)fault0,
           (unsigned int)(fault0 & 0x07U),
           (unsigned int)((fault0 >> 7) & 0x01U),
           (unsigned int)((fault0 >> 6) & 0x01U),
           (unsigned int)((fault0 >> 5) & 0x01U),
           (unsigned int)((fault0 >> 3) & 0x01U));

    return GLOVE_STATUS_OK;
}

GloveStatus_t Bq25622_DumpDebugRegisters(const Bq25622Handle_t *handle)
{
    uint16_t ichg_reg;
    uint16_t iindpm_reg;
    uint8_t ctrl1;
    uint8_t ctrl4;
    uint8_t status0;
    uint8_t status1;
    uint8_t fault0;
    uint16_t ichg_code;
    uint16_t iindpm_code;
    GloveStatus_t status;

    status = Bq25622_ReadRegister16(handle, BQ25622_REG_CHARGE_CURRENT_LIMIT, &ichg_reg);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister16(handle, BQ25622_REG_INPUT_CURRENT_LIMIT, &iindpm_reg);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_1, &ctrl1);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_CONTROL_4, &ctrl4);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_STATUS_0, &status0);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_CHARGER_STATUS_1, &status1);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = Bq25622_ReadRegister8(handle, BQ25622_REG_FAULT_STATUS_0, &fault0);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    ichg_code = (uint16_t)((ichg_reg >> BQ25622_CHARGE_CURRENT_REG_SHIFT) &
                           BQ25622_CHARGE_CURRENT_FIELD_MASK);
    iindpm_code = (uint16_t)((iindpm_reg >> BQ25622_INPUT_CURRENT_REG_SHIFT) &
                             BQ25622_INPUT_CURRENT_FIELD_MASK);

    BQ25622_DEBUG_PRINTF("[BQ25622] dump ICHG_REG=0x%04X ICHG=%u mA IINDPM_REG=0x%04X IINDPM=%u mA CTRL1=0x%02X CTRL4=0x%02X ext_ilim=%u\r\n",
           (unsigned int)ichg_reg,
           (unsigned int)(ichg_code * BQ25622_CHARGE_CURRENT_CODE_LSB_MA),
           (unsigned int)iindpm_reg,
           (unsigned int)(iindpm_code * BQ25622_INPUT_CURRENT_CODE_LSB_MA),
           (unsigned int)ctrl1,
           (unsigned int)ctrl4,
           (unsigned int)((ctrl4 & BQ25622_CHARGER_CONTROL_4_EN_EXTILIM_MASK) != 0U));

    BQ25622_DEBUG_PRINTF("[BQ25622] dump STAT0=0x%02X treg=%u vsys=%u iindpm_or_ilim=%u vindpm=%u safety=%u wd=%u STAT1=0x%02X chg=%u vbus=%u FAULT0=0x%02X ts=%u vbus_fault=%u bat_fault=%u sys_fault=%u tshut=%u\r\n",
           (unsigned int)status0,
           (unsigned int)((status0 >> 5) & 0x01U),
           (unsigned int)((status0 >> 4) & 0x01U),
           (unsigned int)((status0 >> 3) & 0x01U),
           (unsigned int)((status0 >> 2) & 0x01U),
           (unsigned int)((status0 >> 1) & 0x01U),
           (unsigned int)(status0 & 0x01U),
           (unsigned int)status1,
           (unsigned int)((status1 >> 3) & 0x03U),
           (unsigned int)(status1 & 0x07U),
           (unsigned int)fault0,
           (unsigned int)(fault0 & 0x07U),
           (unsigned int)((fault0 >> 7) & 0x01U),
           (unsigned int)((fault0 >> 6) & 0x01U),
           (unsigned int)((fault0 >> 5) & 0x01U),
           (unsigned int)((fault0 >> 3) & 0x01U));

    return GLOVE_STATUS_OK;
}
