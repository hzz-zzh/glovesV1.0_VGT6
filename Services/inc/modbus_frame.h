#ifndef __MODBUS_FRAME_H__
#define __MODBUS_FRAME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"

#define MODBUS_BROADCAST_ADDR          0x00U
#define MODBUS_MIN_RTU_FRAME_LEN       4U
#define MODBUS_READ_REQ_LEN            8U
#define MODBUS_MAX_READ_REG_COUNT      125U

#define MB_FC_READ_HOLDING_REGS        0x03U
#define MB_FC_WRITE_SINGLE_REG         0x06U
#define MB_FC_WRITE_MULTIPLE_REGS      0x10U

#define MB_EX_ILLEGAL_FUNCTION         0x01U
#define MB_EX_ILLEGAL_DATA_ADDRESS     0x02U
#define MB_EX_ILLEGAL_DATA_VALUE       0x03U

typedef enum
{
  MODBUS_RESULT_NO_RESPONSE = 0,
  MODBUS_RESULT_RESPONSE_READY,
  MODBUS_RESULT_FRAME_ERROR
} ModbusResult_t;

void Modbus_SetSlaveAddress(uint8_t address);
uint8_t Modbus_GetSlaveAddress(void);
uint16_t Modbus_Crc16(const uint8_t *data, uint16_t len);
void Modbus_UpdateFullFrameSnapshot(const GloveFullFrame_t *frame);

ModbusResult_t Modbus_ProcessRequest(const uint8_t *rx_buf,
                                     uint16_t rx_len,
                                     uint8_t *tx_buf,
                                     uint16_t tx_buf_size,
                                     uint16_t *tx_len);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_FRAME_H__ */
