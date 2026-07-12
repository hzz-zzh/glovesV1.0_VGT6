#ifndef IMU_CAN_TASK_H
#define IMU_CAN_TASK_H

#include <stdint.h>

#define IMU_CAN_TASK_DEBUG_BUS_COUNT (2U)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t rx_irq_count;
    uint32_t rx_frame_count;
    uint32_t parsed_frame_count;
    uint32_t published_count;
    uint32_t publish_drop_count;
    uint32_t init_error_count;
    uint32_t last_error;
    uint32_t unparsed_frame_count;
    uint32_t rejected_node_count;
    uint32_t last_rx_id;
    uint32_t last_rx_is_extended;
    uint32_t last_rx_dlc;
    uint8_t last_rx_data[8];
    uint32_t node_last_rx_id;
    uint32_t node_last_rx_dlc;
    uint8_t node_last_rx_data[8];
    uint32_t cfg_phase;
    uint32_t cfg_target_new_id;
    uint32_t cfg_tx_count;
    uint32_t cfg_last_tx_id;
    uint32_t cfg_last_tx_dlc;
    uint8_t cfg_last_tx_data[8];
    uint32_t cfg_reply_count;
    uint32_t cfg_last_reply_id;
    uint32_t cfg_last_reply_dlc;
    uint8_t cfg_last_reply_data[8];
    uint32_t cfg_verified_node_mask;
    uint32_t cfg_failed_node_mask;
    uint32_t cfg_retry_count;
    uint32_t cfg_step_ack_count[6];
    uint32_t cfg_step_ack_id[6];
    uint8_t cfg_step_ack_data[6][8];
    uint32_t cfg_step_tx_id[6];
    uint8_t cfg_step_tx_data[6][8];
    uint32_t bus_started_mask;
    uint32_t bus_state[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t bus_tx_error_count[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t bus_rx_error_count[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t bus_rx_error_passive[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t bus_last_error_code[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t bus_data_last_error_code[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t bus_activity[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t bus_error_passive[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t bus_warning[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t bus_off[IMU_CAN_TASK_DEBUG_BUS_COUNT];
    uint32_t first_valid_node_id;
    uint32_t first_valid_seen_mask;
    int32_t accel_x_mg;
    int32_t accel_y_mg;
    int32_t accel_z_mg;
    int32_t gyro_x_mdps;
    int32_t gyro_y_mdps;
    int32_t gyro_z_mdps;
    uint32_t quat_source;
    int32_t quat_w_1e4;
    int32_t quat_x_1e4;
    int32_t quat_y_1e4;
    int32_t quat_z_1e4;
} ImuCanTaskDebugSnapshot_t;

void ImuCanTask(void *argument);
void ImuCanTask_GetDebugSnapshot(ImuCanTaskDebugSnapshot_t *snapshot);
void ImuCanTask_GetDebugSnapshotForNode(uint32_t node_id,
                                        ImuCanTaskDebugSnapshot_t *snapshot);
uint32_t ImuCanTask_GetFirstNodeId(void);
uint32_t ImuCanTask_GetNodeCount(void);
uint16_t ImuCanTask_GetFreshMask(void);
void ImuCanTask_SetAcquisitionEnabled(uint8_t enabled);
uint8_t ImuCanTask_IsAcquisitionPaused(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CAN_TASK_H */
