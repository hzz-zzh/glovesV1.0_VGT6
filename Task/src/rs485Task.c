#include "rs485Task.h"

#include "RS485_uasrt.h"
#include "cmsis_os2.h"
#include "data_manager.h"
#include "modbus_frame.h"
#include "modbus_time_sync.h"

#define RS485_TASK_EVT_RX_FRAME (1UL << 0)
#define RS485_TASK_EVT_TX_EVENT (1UL << 1)
#define RS485_TASK_POLL_TIMEOUT_MS (10U)
#define RS485_TASK_FULL_DRAIN_LIMIT (4U)

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

void Rs485Task(void *argument)
{
  uint32_t flags;

  (void)argument;
  rs485_task_id = osThreadGetId();

  (void)ModbusTimeSync_Init();
  while (RS485_Init() != HAL_OK)
  {
    osDelay(100U);
  }

  for (;;)
  {
    RS485_TaskDrainFullFrames();

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
        RS485_ProcessRxFrame();
      }
    }
  }
}
