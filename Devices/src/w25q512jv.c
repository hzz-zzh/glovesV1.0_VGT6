#include "w25q512jv.h"

#include <stddef.h>

#include "main.h"
#include "stm32h5xx_hal.h"

#define W25Q512JV_CMD_WRITE_ENABLE          (0x06U)
#define W25Q512JV_CMD_READ_STATUS_REG1      (0x05U)
#define W25Q512JV_CMD_READ_DATA_4B          (0x13U)
#define W25Q512JV_CMD_PAGE_PROGRAM_4B       (0x12U)
#define W25Q512JV_CMD_SECTOR_ERASE_4B       (0x21U)
#define W25Q512JV_CMD_READ_JEDEC_ID         (0x9FU)

#define W25Q512JV_STATUS_BUSY_MASK          (0x01U)
#define W25Q512JV_COMMAND_TIMEOUT_MS        (100U)
#define W25Q512JV_POLL_INTERVAL_CYCLES      (0x10U)

static GloveStatus_t W25q512jv_MapHalStatus(HAL_StatusTypeDef status)
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

static uint32_t W25q512jv_UseDefaultTimeout(uint32_t timeout_ms)
{
    return (timeout_ms == 0U) ? W25Q512JV_DEFAULT_TIMEOUT_MS : timeout_ms;
}

static GloveStatus_t W25q512jv_CheckRange(uint32_t address, uint32_t size)
{
    if (size == 0U)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if (address >= W25Q512JV_SIZE_BYTES)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    if (size > (W25Q512JV_SIZE_BYTES - address))
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    return GLOVE_STATUS_OK;
}

static void W25q512jv_FillBaseCommand(XSPI_RegularCmdTypeDef *cmd)
{
    cmd->OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
    cmd->IOSelect = HAL_XSPI_SELECT_IO_3_0;
    cmd->Instruction = 0U;
    cmd->InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    cmd->InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS;
    cmd->InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
    cmd->Address = 0U;
    cmd->AddressMode = HAL_XSPI_ADDRESS_NONE;
    cmd->AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    cmd->AddressDTRMode = HAL_XSPI_ADDRESS_DTR_DISABLE;
    cmd->AlternateBytes = 0U;
    cmd->AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
    cmd->AlternateBytesWidth = HAL_XSPI_ALT_BYTES_8_BITS;
    cmd->AlternateBytesDTRMode = HAL_XSPI_ALT_BYTES_DTR_DISABLE;
    cmd->DataMode = HAL_XSPI_DATA_NONE;
    cmd->DataLength = 1U;
    cmd->DataDTRMode = HAL_XSPI_DATA_DTR_DISABLE;
    cmd->DummyCycles = 0U;
    cmd->DQSMode = HAL_XSPI_DQS_DISABLE;
}

static GloveStatus_t W25q512jv_SendSimpleCommand(uint32_t instruction)
{
    XSPI_RegularCmdTypeDef cmd;

    W25q512jv_FillBaseCommand(&cmd);
    cmd.Instruction = instruction;

    return W25q512jv_MapHalStatus(HAL_XSPI_Command(&hxspi1,
                                                   &cmd,
                                                   W25Q512JV_COMMAND_TIMEOUT_MS));
}

static GloveStatus_t W25q512jv_WriteEnable(void)
{
    return W25q512jv_SendSimpleCommand(W25Q512JV_CMD_WRITE_ENABLE);
}

GloveStatus_t W25q512jv_WaitReady(uint32_t timeout_ms)
{
    XSPI_RegularCmdTypeDef cmd;
    XSPI_AutoPollingTypeDef polling;
    HAL_StatusTypeDef hal_status;

    W25q512jv_FillBaseCommand(&cmd);
    cmd.Instruction = W25Q512JV_CMD_READ_STATUS_REG1;
    cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength = 1U;

    polling.MatchValue = 0U;
    polling.MatchMask = W25Q512JV_STATUS_BUSY_MASK;
    polling.MatchMode = HAL_XSPI_MATCH_MODE_AND;
    polling.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;
    polling.IntervalTime = W25Q512JV_POLL_INTERVAL_CYCLES;

    hal_status = HAL_XSPI_Command(&hxspi1, &cmd, W25Q512JV_COMMAND_TIMEOUT_MS);
    if (hal_status != HAL_OK)
    {
        return W25q512jv_MapHalStatus(hal_status);
    }

    hal_status = HAL_XSPI_AutoPolling(&hxspi1,
                                      &polling,
                                      W25q512jv_UseDefaultTimeout(timeout_ms));
    return W25q512jv_MapHalStatus(hal_status);
}

GloveStatus_t W25q512jv_ReadJedecId(W25q512jvId_t *id)
{
    XSPI_RegularCmdTypeDef cmd;
    uint8_t data[3];
    HAL_StatusTypeDef hal_status;

    if (id == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    W25q512jv_FillBaseCommand(&cmd);
    cmd.Instruction = W25Q512JV_CMD_READ_JEDEC_ID;
    cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength = sizeof(data);

    hal_status = HAL_XSPI_Command(&hxspi1, &cmd, W25Q512JV_COMMAND_TIMEOUT_MS);
    if (hal_status != HAL_OK)
    {
        return W25q512jv_MapHalStatus(hal_status);
    }

    hal_status = HAL_XSPI_Receive(&hxspi1, data, W25Q512JV_COMMAND_TIMEOUT_MS);
    if (hal_status != HAL_OK)
    {
        return W25q512jv_MapHalStatus(hal_status);
    }

    id->manufacturer_id = data[0];
    id->memory_type = data[1];
    id->capacity = data[2];
    id->reserved = 0U;
    id->jedec_id = ((uint32_t)data[0] << 16) |
                   ((uint32_t)data[1] << 8) |
                   (uint32_t)data[2];

    return GLOVE_STATUS_OK;
}

GloveStatus_t W25q512jv_Init(void)
{
    W25q512jvId_t id;
    GloveStatus_t status;

    status = W25q512jv_WaitReady(W25Q512JV_DEFAULT_TIMEOUT_MS);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = W25q512jv_ReadJedecId(&id);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    if (id.jedec_id != W25Q512JV_JEDEC_ID)
    {
        return GLOVE_STATUS_NOT_READY;
    }

    return GLOVE_STATUS_OK;
}

GloveStatus_t W25q512jv_Read(uint32_t address,
                             uint8_t *data,
                             uint32_t size,
                             uint32_t timeout_ms)
{
    XSPI_RegularCmdTypeDef cmd;
    HAL_StatusTypeDef hal_status;
    GloveStatus_t status;

    if (data == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = W25q512jv_CheckRange(address, size);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    W25q512jv_FillBaseCommand(&cmd);
    cmd.Instruction = W25Q512JV_CMD_READ_DATA_4B;
    cmd.Address = address;
    cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength = size;

    hal_status = HAL_XSPI_Command(&hxspi1,
                                  &cmd,
                                  W25q512jv_UseDefaultTimeout(timeout_ms));
    if (hal_status != HAL_OK)
    {
        return W25q512jv_MapHalStatus(hal_status);
    }

    hal_status = HAL_XSPI_Receive(&hxspi1,
                                  data,
                                  W25q512jv_UseDefaultTimeout(timeout_ms));
    return W25q512jv_MapHalStatus(hal_status);
}

GloveStatus_t W25q512jv_PageProgram(uint32_t address,
                                    const uint8_t *data,
                                    uint32_t size,
                                    uint32_t timeout_ms)
{
    XSPI_RegularCmdTypeDef cmd;
    HAL_StatusTypeDef hal_status;
    GloveStatus_t status;
    uint32_t page_remaining;

    if (data == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = W25q512jv_CheckRange(address, size);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    page_remaining = W25Q512JV_PAGE_SIZE - (address % W25Q512JV_PAGE_SIZE);
    if (size > page_remaining)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = W25q512jv_WriteEnable();
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    W25q512jv_FillBaseCommand(&cmd);
    cmd.Instruction = W25Q512JV_CMD_PAGE_PROGRAM_4B;
    cmd.Address = address;
    cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
    cmd.DataMode = HAL_XSPI_DATA_1_LINE;
    cmd.DataLength = size;

    hal_status = HAL_XSPI_Command(&hxspi1,
                                  &cmd,
                                  W25q512jv_UseDefaultTimeout(timeout_ms));
    if (hal_status != HAL_OK)
    {
        return W25q512jv_MapHalStatus(hal_status);
    }

    hal_status = HAL_XSPI_Transmit(&hxspi1,
                                   data,
                                   W25q512jv_UseDefaultTimeout(timeout_ms));
    if (hal_status != HAL_OK)
    {
        return W25q512jv_MapHalStatus(hal_status);
    }

    return W25q512jv_WaitReady((timeout_ms == 0U) ?
                               W25Q512JV_PROGRAM_TIMEOUT_MS :
                               timeout_ms);
}

GloveStatus_t W25q512jv_Write(uint32_t address,
                              const uint8_t *data,
                              uint32_t size,
                              uint32_t timeout_ms)
{
    GloveStatus_t status;
    uint32_t offset = 0U;
    uint32_t chunk_size;
    uint32_t page_remaining;

    if (data == NULL)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = W25q512jv_CheckRange(address, size);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    while (offset < size)
    {
        page_remaining = W25Q512JV_PAGE_SIZE -
                         ((address + offset) % W25Q512JV_PAGE_SIZE);
        chunk_size = size - offset;
        if (chunk_size > page_remaining)
        {
            chunk_size = page_remaining;
        }

        status = W25q512jv_PageProgram(address + offset,
                                       &data[offset],
                                       chunk_size,
                                       timeout_ms);
        if (status != GLOVE_STATUS_OK)
        {
            return status;
        }

        offset += chunk_size;
    }

    return GLOVE_STATUS_OK;
}

GloveStatus_t W25q512jv_EraseSector(uint32_t address,
                                    uint32_t timeout_ms)
{
    XSPI_RegularCmdTypeDef cmd;
    HAL_StatusTypeDef hal_status;
    GloveStatus_t status;

    if ((address % W25Q512JV_SECTOR_SIZE) != 0U)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    status = W25q512jv_CheckRange(address, W25Q512JV_SECTOR_SIZE);
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    status = W25q512jv_WriteEnable();
    if (status != GLOVE_STATUS_OK)
    {
        return status;
    }

    W25q512jv_FillBaseCommand(&cmd);
    cmd.Instruction = W25Q512JV_CMD_SECTOR_ERASE_4B;
    cmd.Address = address;
    cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
    cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;

    hal_status = HAL_XSPI_Command(&hxspi1,
                                  &cmd,
                                  W25q512jv_UseDefaultTimeout(timeout_ms));
    if (hal_status != HAL_OK)
    {
        return W25q512jv_MapHalStatus(hal_status);
    }

    return W25q512jv_WaitReady((timeout_ms == 0U) ?
                               W25Q512JV_ERASE_TIMEOUT_MS :
                               timeout_ms);
}
