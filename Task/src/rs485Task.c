#include "rs485Task.h"

#include "RS485_uasrt.h"
#include "cmsis_os2.h"
#include "data_manager.h"
#include "modbus_frame.h"
#include "modbus_time_sync.h"
#include "system_health.h"

#define RS485_TASK_EVT_RX_FRAME (1UL << 0)
#define RS485_TASK_EVT_TX_EVENT (1UL << 1)
#define RS485_TASK_POLL_TIMEOUT_MS (10U)
#define RS485_TASK_FULL_DRAIN_LIMIT (4U)
#define RS485_TASK_HEALTH_RECOVERY_TX (10U)

static osThreadId_t rs485_task_id = NULL;

static volatile uint32_t rs485_task_rx_events = 0U;
static volatile uint32_t rs485_task_tx_events = 0U;
static volatile uint32_t rs485_task_full_frames = 0U;

static void RS485_TaskNotify(uint32_t flags)
{
  if (rs485_task_id != NULL)
  {
    (void)osThreadFlagsSet(rs485_task_id, flags);
  }
}

void RS485_TaskNotifyRxFrame(void)
{
  rs485_task_rx_events++;
  RS485_TaskNotify(RS485_TASK_EVT_RX_FRAME);
}

void RS485_TaskNotifyTxComplete(void)
{
  rs485_task_tx_events++;
  RS485_TaskNotify(RS485_TASK_EVT_TX_EVENT);
}

void RS485_TaskNotifyTxError(void)
{
  RS485_TaskNotify(RS485_TASK_EVT_TX_EVENT);
}

void RS485_TaskGetEventCounts(uint32_t *rx_event_count, uint32_t *tx_event_count)
{
  if (rx_event_count != NULL)
  {
    *rx_event_count = rs485_task_rx_events;
  }

  if (tx_event_count != NULL)
  {
    *tx_event_count = rs485_task_tx_events;
  }
}

void RS485_TaskGetStats(Rs485TaskStats_t *stats)
{
  if (stats == NULL)
  {
    return;
  }

  stats->rx_event_count = rs485_task_rx_events;
  stats->tx_event_count = rs485_task_tx_events;
  stats->full_frame_count = rs485_task_full_frames;
}

static uint32_t RS485_TaskMsToTicks(uint32_t timeout_ms)
{
  uint64_t ticks = ((uint64_t)timeout_ms * (uint64_t)osKernelGetTickFreq() + 999ULL) / 1000ULL;

  if ((timeout_ms > 0U) && (ticks == 0ULL))
  {
    ticks = 1ULL;
  }

  return (ticks > 0xFFFFFFFEULL) ? 0xFFFFFFFEUL : (uint32_t)ticks;
}

static void RS485_TaskDrainFullFrames(void)
{
  GloveFullFrameBlock_t *full = NULL;
  GloveStatus_t status;
  uint32_t drained = 0U;

  while (drained < RS485_TASK_FULL_DRAIN_LIMIT)
  {
    status = DataManager_GetFullFrame(DATA_CONSUMER_RS485, &full, 0U);
    if (status != GLOVE_STATUS_OK)
    {
      break;
    }

    Modbus_UpdateFullFrameSnapshot(&full->frame);
    (void)DataManager_ReleaseFullFrame(full);
    full = NULL;
    drained++;
    rs485_task_full_frames++;
  }
}

static void RS485_TaskServiceHealth(void)
{
  static RS485_StatusTypeDef previous;
  static uint8_t recovery_tx_count;
  RS485_StatusTypeDef current;
  uint8_t new_error;

  RS485_GetStatus(&current);
  SystemHealth_SetRs485UartDetail((uint16_t)(current.last_uart_error & 0xFFFFU));
  new_error = ((current.rx_overwrite != previous.rx_overwrite) ||
               (current.errors != previous.errors) ||
               (current.tx_send_fail != previous.tx_send_fail)) ? 1U : 0U;

  if (current.rx_overwrite != previous.rx_overwrite)
  {
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_RS485_RX_OVERWRITE,
                          SYSTEM_ERROR_RS485_RX_OVERWRITE,
                          SYSTEM_HEALTH_SOURCE_RS485,
                          0U,
                          1U);
  }
  if (current.errors != previous.errors)
  {
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_RS485_UART_ERROR,
                          SYSTEM_ERROR_RS485_UART,
                          SYSTEM_HEALTH_SOURCE_RS485,
                          (uint16_t)(current.last_uart_error & 0xFFFFU),
                          1U);
  }
  if (current.tx_send_fail != previous.tx_send_fail)
  {
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_RS485_TX_FAILED,
                          SYSTEM_ERROR_RS485_TX,
                          SYSTEM_HEALTH_SOURCE_RS485,
                          0U,
                          1U);
  }

  if (new_error != 0U)
  {
    recovery_tx_count = 0U;
    SystemHealth_SetSensorReady(SYSTEM_SENSOR_READY_RS485, 0U);
  }
  else if (current.tx_done != previous.tx_done)
  {
    uint32_t completed = current.tx_done - previous.tx_done;
    recovery_tx_count = (completed >= RS485_TASK_HEALTH_RECOVERY_TX) ?
                        RS485_TASK_HEALTH_RECOVERY_TX :
                        (uint8_t)(recovery_tx_count + completed);
    if (recovery_tx_count >= RS485_TASK_HEALTH_RECOVERY_TX)
    {
      recovery_tx_count = RS485_TASK_HEALTH_RECOVERY_TX;
      SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_RS485_RX_OVERWRITE,
                            SYSTEM_ERROR_NONE,
                            SYSTEM_HEALTH_SOURCE_RS485,
                            0U,
                            0U);
      SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_RS485_UART_ERROR,
                            SYSTEM_ERROR_NONE,
                            SYSTEM_HEALTH_SOURCE_RS485,
                            0U,
                            0U);
      SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_RS485_TX_FAILED,
                            SYSTEM_ERROR_NONE,
                            SYSTEM_HEALTH_SOURCE_RS485,
                            0U,
                            0U);
      SystemHealth_SetSensorReady(SYSTEM_SENSOR_READY_RS485, 1U);
    }
  }

  if (ModbusTimeSync_GetLastSyncUtcUs() != 0ULL)
  {
    uint8_t synced = ModbusTimeSync_IsSynced();
    SystemHealth_SetFault(SYSTEM_HEALTH_FLAG_TIME_UNSYNCED,
                          SYSTEM_ERROR_TIME_UNSYNCED,
                          SYSTEM_HEALTH_SOURCE_TIME_SYNC,
                          0U,
                          (synced == 0U) ? 1U : 0U);
    SystemHealth_SetSensorReady(SYSTEM_SENSOR_READY_TIME_SYNC, synced);
  }
  previous = current;
}

void Rs485Task(void *argument)
{
  uint32_t flags;
  uint32_t init_failure_count = 0U;

  (void)argument;
  rs485_task_id = osThreadGetId();

  (void)ModbusTimeSync_Init();
  while (RS485_Init() != HAL_OK)
  {
    init_failure_count++;
    osDelay(100U);
  }
  if (init_failure_count != 0U)
  {
    SystemHealth_RecordEvent(SYSTEM_ERROR_RS485_UART,
                             SYSTEM_HEALTH_SOURCE_RS485,
                             0U);
  }
  SystemHealth_SetSensorReady(SYSTEM_SENSOR_READY_RS485, 1U);

  for (;;)
  {
    RS485_TaskDrainFullFrames();
    RS485_TaskServiceHealth();

    flags = osThreadFlagsWait(RS485_TASK_EVT_RX_FRAME | RS485_TASK_EVT_TX_EVENT,
                              osFlagsWaitAny,
                              RS485_TaskMsToTicks(RS485_TASK_POLL_TIMEOUT_MS));
    if ((flags & osFlagsError) == 0U)
    {
      if ((flags & RS485_TASK_EVT_TX_EVENT) != 0U)
      {
        RS485_ProcessTxEvent();
      }

      if ((flags & RS485_TASK_EVT_RX_FRAME) != 0U)
      {
        RS485_TaskDrainFullFrames();
        RS485_ProcessRxFrame();
      }
    }
  }
}
