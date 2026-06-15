#ifndef RS485_TASK_H
#define RS485_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Rs485Task(void *argument);

void RS485_TaskNotifyRxFrame(void);
void RS485_TaskNotifyTxComplete(void);
void RS485_TaskNotifyTxError(void);
void RS485_TaskGetEventCounts(uint32_t *rx_event_count, uint32_t *tx_event_count);

#ifdef __cplusplus
}
#endif

#endif /* RS485_TASK_H */
