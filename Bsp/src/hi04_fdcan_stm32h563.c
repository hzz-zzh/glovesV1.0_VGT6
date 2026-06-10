#include "hi04_fdcan_stm32h563.h"

#include <string.h>

#include "stm32h5xx_hal.h"

static uint32_t dlc_to_hal(uint8_t dlc)
{
    static const uint32_t map[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
    };
    return (dlc <= 8u) ? map[dlc] : FDCAN_DLC_BYTES_8;
}

static uint8_t hal_to_dlc(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0: return 0u;
    case FDCAN_DLC_BYTES_1: return 1u;
    case FDCAN_DLC_BYTES_2: return 2u;
    case FDCAN_DLC_BYTES_3: return 3u;
    case FDCAN_DLC_BYTES_4: return 4u;
    case FDCAN_DLC_BYTES_5: return 5u;
    case FDCAN_DLC_BYTES_6: return 6u;
    case FDCAN_DLC_BYTES_7: return 7u;
    default: return 8u;
    }
}

bool hi04_fdcan_stm32h563_send(void *ctx, const hi04_can_frame_t *frame)
{
    hi04_fdcan_stm32h563_t *port = (hi04_fdcan_stm32h563_t *)ctx;
    if (port == 0 || port->hfdcan == 0 || frame == 0 || frame->dlc > 8u) return false;

    FDCAN_TxHeaderTypeDef tx;
    memset(&tx, 0, sizeof(tx));
    tx.Identifier = frame->id & HI04_CAN_EFF_MASK;
    tx.IdType = (frame->id & HI04_CAN_EFF_FLAG) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = dlc_to_hal(frame->dlc);
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0u;
    return HAL_FDCAN_AddMessageToTxFifoQ(port->hfdcan, &tx, (uint8_t *)frame->data) == HAL_OK;
}

bool hi04_fdcan_stm32h563_read_fifo0(hi04_fdcan_stm32h563_t *port,
                                     hi04_can_frame_t *frame)
{
    if (port == 0 || port->hfdcan == 0 || frame == 0) return false;

    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    if (HAL_FDCAN_GetRxMessage(port->hfdcan, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->id = rx.Identifier;
    if (rx.IdType == FDCAN_EXTENDED_ID) frame->id |= HI04_CAN_EFF_FLAG;
    frame->dlc = hal_to_dlc(rx.DataLength);
    memcpy(frame->data, data, frame->dlc);
    frame->timestamp_us = (port->time_us != 0) ? port->time_us() : 0u;
    return true;
}
