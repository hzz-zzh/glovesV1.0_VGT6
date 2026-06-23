#ifndef RS485_TASK_H
#define RS485_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t rx_event_count;
  uint32_t tx_event_count;
  uint32_t full_frame_count;
} Rs485TaskStats_t;

void Rs485Task(void *argument);

void RS485_TaskNotifyRxFrame(void);
void RS485_TaskNotifyTxComplete(void);
void RS485_TaskNotifyTxError(void);
void RS485_TaskGetEventCounts(uint32_t *rx_event_count, uint32_t *tx_event_count);
void RS485_TaskGetStats(Rs485TaskStats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* RS485_TASK_H */
