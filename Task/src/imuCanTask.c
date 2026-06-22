#include "imuCanTask.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "app_data.h"
#include "app_freertos.h"
#include "acq_sync.h"
#include "cmsis_os2.h"
#include "data_manager.h"
#include "glove_hand_config.h"
#include "hi04_driver.h"
#include "hi04_fdcan_stm32h563.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

#define IMU_CAN_TASK_RX_FLAG                    (1UL << 0)

#define IMU_CAN_TASK_BUS_COUNT                  (2U)
#define IMU_CAN_TASK_MAX_NODES_PER_BUS          (8U)
#define IMU_CAN_TASK_LOGICAL_FIRST_NODE_ID      (1U)

#define IMU_CAN_TASK_BUS1_CAN_HANDLE            (&hfdcan1)
#define IMU_CAN_TASK_BUS1_FIRST_NODE_ID         (0x11U)
#define IMU_CAN_TASK_BUS1_NODE_COUNT            (8U)
#define IMU_CAN_TASK_BUS1_CONFIG_NODE_COUNT     (8U)

#define IMU_CAN_TASK_BUS2_CAN_HANDLE            (&hfdcan2)
#define IMU_CAN_TASK_BUS2_FIRST_NODE_ID         (0x11U)
#define IMU_CAN_TASK_BUS2_NODE_COUNT            (8U)
#define IMU_CAN_TASK_BUS2_CONFIG_NODE_COUNT     (8U)

#define IMU_CAN_TASK_SET_IMU_BUS_INDEX          (0U)

#define IMU_CAN_TASK_PUBLISH_PERIOD_MS          (10U)
#define IMU_CAN_TASK_QUEUE_TIMEOUT_MS           (0U)
#define IMU_CAN_TASK_FRAME_FLAGS_REQUIRED       (HI04_SEEN_ACCEL | HI04_SEEN_GYRO)
#define IMU_CAN_TASK_G_TO_MPS2                  (9.80665f)
#define IMU_CAN_TASK_DEG_TO_RAD                 (0.01745329251994329577f)
#define IMU_CAN_TASK_J1939_PRIORITY             (3U)
#define IMU_CAN_TASK_J1939_HOST_ADDR            (0x55U)
#define IMU_CAN_TASK_J1939_CONFIG_PF            (0xEFU)
#define IMU_CAN_TASK_J1939_OUTPUT_ACCEL         (0x0134U)
#define IMU_CAN_TASK_J1939_OUTPUT_GYRO          (0x0137U)
#define IMU_CAN_TASK_J1939_OUTPUT_RPY           (0x013DU)
#define IMU_CAN_TASK_J1939_OUTPUT_YAW           (0x0141U)
#define IMU_CAN_TASK_J1939_OUTPUT_QUAT          (0x0146U)
#define IMU_CAN_TASK_J1939_REG_CONTROL          (0x0000U)
#define IMU_CAN_TASK_J1939_REG_OUTPUT_EN        (0x009DU)
#define IMU_CAN_TASK_J1939_REG_NODE_ID          (0x009CU)
#define IMU_CAN_TASK_ENABLE_J1939_CONFIG        (0U)
#define IMU_CAN_TASK_OUTPUT_PERIOD_MS           (10U)
#define IMU_CAN_TASK_CLEAR_J1939_PERIODS_ONCE   (0U)
#define IMU_CAN_TASK_CONFIG_J1939_SYNC_ONCE     (1U)
#define IMU_CAN_TASK_J1939_SYNC_TRIGGER_VAL     (0x8000UL)
#define IMU_CAN_TASK_ENABLE_SET_IMU_ID          (0U)
#define IMU_CAN_TASK_SET_IMU_OLD_ID             (8U)
#define IMU_CAN_TASK_SET_IMU_NEW_ID             (3U)
#define IMU_CAN_TASK_ID_CONFIG_LOOP_MS          (500U)
#define IMU_CAN_TASK_ENABLE_PUBLISH             (1U)
#define IMU_CAN_TASK_QUAT_SOURCE_NONE           (0U)
#define IMU_CAN_TASK_QUAT_SOURCE_RAW            (1U)
#define IMU_CAN_TASK_IRQ_PRIORITY               (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
#define IMU_CAN_TASK_IRQ_SUBPRIORITY            (0U)

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t first_node_id;
    uint8_t node_count;
    uint8_t config_node_count;
} ImuCanTaskBusConfig_t;

typedef struct
{
    const ImuCanTaskBusConfig_t *config;
    hi04_fdcan_stm32h563_t port;
    hi04_device_t devices[IMU_CAN_TASK_MAX_NODES_PER_BUS];
    uint32_t node_last_rx_id[IMU_CAN_TASK_MAX_NODES_PER_BUS];
    uint32_t node_last_rx_dlc[IMU_CAN_TASK_MAX_NODES_PER_BUS];
    uint8_t node_last_rx_data[IMU_CAN_TASK_MAX_NODES_PER_BUS][8];
    uint32_t node_sync_seq[IMU_CAN_TASK_MAX_NODES_PER_BUS];
    GloveTimestampUs_t node_sync_timestamp_us[IMU_CAN_TASK_MAX_NODES_PER_BUS];
    uint32_t node_sync_seen_mask[IMU_CAN_TASK_MAX_NODES_PER_BUS];
    uint8_t node_sync_valid[IMU_CAN_TASK_MAX_NODES_PER_BUS];
    bool fdcan_started;
} ImuCanTaskBusRuntime_t;

typedef struct
{
    volatile uint32_t rx_irq_count;
    volatile uint32_t rx_frame_count;
    volatile uint32_t parsed_frame_count;
    volatile uint32_t published_count;
    volatile uint32_t publish_drop_count;
    volatile uint32_t init_error_count;
    volatile uint32_t last_error;
    volatile uint32_t unparsed_frame_count;
    volatile uint32_t rejected_node_count;
    volatile uint32_t last_rx_id;
    volatile uint32_t last_rx_is_extended;
    volatile uint32_t last_rx_dlc;
    uint8_t last_rx_data[8];
    volatile uint32_t cfg_phase;
    volatile uint32_t cfg_target_new_id;
    volatile uint32_t cfg_tx_count;
    volatile uint32_t cfg_last_tx_id;
    volatile uint32_t cfg_last_tx_dlc;
    uint8_t cfg_last_tx_data[8];
    volatile uint32_t cfg_reply_count;
    volatile uint32_t cfg_last_reply_id;
    volatile uint32_t cfg_last_reply_dlc;
    uint8_t cfg_last_reply_data[8];
    volatile uint32_t cfg_active_step;
    uint32_t cfg_step_ack_count[6];
    uint32_t cfg_step_ack_id[6];
    uint8_t cfg_step_ack_data[6][8];
    uint32_t cfg_step_tx_id[6];
    uint8_t cfg_step_tx_data[6][8];
} ImuCanTaskStats_t;

static const ImuCanTaskBusConfig_t s_bus_configs[IMU_CAN_TASK_BUS_COUNT] =
{
    { IMU_CAN_TASK_BUS1_CAN_HANDLE, IMU_CAN_TASK_BUS1_FIRST_NODE_ID,
      IMU_CAN_TASK_BUS1_NODE_COUNT, IMU_CAN_TASK_BUS1_CONFIG_NODE_COUNT },
    { IMU_CAN_TASK_BUS2_CAN_HANDLE, IMU_CAN_TASK_BUS2_FIRST_NODE_ID,
      IMU_CAN_TASK_BUS2_NODE_COUNT, IMU_CAN_TASK_BUS2_CONFIG_NODE_COUNT }
};

static ImuCanTaskBusRuntime_t s_buses[IMU_CAN_TASK_BUS_COUNT];
static ImuCanTaskStats_t s_imu_can_stats;
static uint32_t s_sensor_seq;

static const uint8_t s_left_bus1_output_index[IMU_CAN_TASK_MAX_NODES_PER_BUS] =
{
    1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U
};

static const uint8_t s_left_bus2_output_index[IMU_CAN_TASK_MAX_NODES_PER_BUS] =
{
    13U, 14U, 15U, 10U, 11U, 12U, 0U, 9U
};

static const uint8_t s_right_bus1_output_index[IMU_CAN_TASK_MAX_NODES_PER_BUS] =
{
    13U, 14U, 15U, 10U, 11U, 12U, 7U, 8U
};

static const uint8_t s_right_bus2_output_index[IMU_CAN_TASK_MAX_NODES_PER_BUS] =
{
    1U, 2U, 3U, 4U, 5U, 6U, 0U, 9U
};

static const uint8_t *const s_left_output_maps[IMU_CAN_TASK_BUS_COUNT] =
{
    s_left_bus1_output_index,
    s_left_bus2_output_index
};

static const uint8_t *const s_right_output_maps[IMU_CAN_TASK_BUS_COUNT] =
{
    s_right_bus1_output_index,
    s_right_bus2_output_index
};

static uint64_t ImuCanTask_TimeUs(void)
{
    return (uint64_t)HAL_GetTick() * 1000ULL;
}

static uint32_t ImuCanTask_TotalLogicalNodeCount(void)
{
    return GLOVE_IMU_COUNT;
}

static uint32_t ImuCanTask_SeenBitForMsg(hi04_msg_type_t msg_type)
{
    switch (msg_type)
    {
        case HI04_MSG_ACCEL:
            return HI04_SEEN_ACCEL;
        case HI04_MSG_GYRO:
            return HI04_SEEN_GYRO;
        case HI04_MSG_MAG:
            return HI04_SEEN_MAG;
        case HI04_MSG_QUAT:
            return HI04_SEEN_QUAT;
        case HI04_MSG_PITCH_ROLL:
            return HI04_SEEN_PITCH_ROLL;
        case HI04_MSG_YAW:
            return HI04_SEEN_YAW;
        case HI04_MSG_ENV:
            return HI04_SEEN_ENV;
        case HI04_MSG_TIME:
            return HI04_SEEN_TIME;
        case HI04_MSG_CANFD0:
            return HI04_SEEN_CANFD0;
        default:
            return 0U;
    }
}

static const uint8_t *const *ImuCanTask_GetOutputMaps(void)
{
    return (GloveHandConfig_GetHandSide() == GLOVE_HAND_RIGHT) ?
           s_right_output_maps :
           s_left_output_maps;
}

static bool ImuCanTask_GetBusIndex(const ImuCanTaskBusRuntime_t *bus,
                                   uint32_t *bus_index)
{
    if ((bus == NULL) || (bus_index == NULL))
    {
        return false;
    }

    for (uint32_t i = 0U; i < IMU_CAN_TASK_BUS_COUNT; i++)
    {
        if (bus == &s_buses[i])
        {
            *bus_index = i;
            return true;
        }
    }

    return false;
}

static bool ImuCanTask_BusOutputIndex(const ImuCanTaskBusRuntime_t *bus,
                                      uint32_t local_index,
                                      uint32_t *output_index)
{
    uint32_t bus_index;
    const uint8_t *const *maps = ImuCanTask_GetOutputMaps();
    uint8_t mapped_index;

    if ((bus == NULL) || (bus->config == NULL) || (output_index == NULL) ||
        (bus->config->node_count > IMU_CAN_TASK_MAX_NODES_PER_BUS) ||
        (local_index >= bus->config->node_count) ||
        (local_index >= IMU_CAN_TASK_MAX_NODES_PER_BUS) ||
        (ImuCanTask_GetBusIndex(bus, &bus_index) == false) ||
        (bus_index >= IMU_CAN_TASK_BUS_COUNT) ||
        (maps[bus_index] == NULL))
    {
        return false;
    }

    mapped_index = maps[bus_index][local_index];
    if (mapped_index >= GLOVE_IMU_COUNT)
    {
        return false;
    }

    *output_index = (uint32_t)mapped_index;
    return true;
}

static uint32_t ImuCanTask_BusLogicalNodeId(const ImuCanTaskBusRuntime_t *bus,
                                            uint32_t local_index)
{
    uint32_t output_index;

    if (ImuCanTask_BusOutputIndex(bus, local_index, &output_index) == false)
    {
        return 0U;
    }

    return IMU_CAN_TASK_LOGICAL_FIRST_NODE_ID + output_index;
}

static bool ImuCanTask_ValidateBusMap(uint32_t bus_index, uint8_t node_count)
{
    if ((bus_index >= IMU_CAN_TASK_BUS_COUNT) ||
        (node_count > IMU_CAN_TASK_MAX_NODES_PER_BUS) ||
        (s_left_output_maps[bus_index] == NULL) ||
        (s_right_output_maps[bus_index] == NULL))
    {
        return false;
    }

    for (uint32_t local_i = 0U; local_i < node_count; local_i++)
    {
        if ((s_left_output_maps[bus_index][local_i] >= GLOVE_IMU_COUNT) ||
            (s_right_output_maps[bus_index][local_i] >= GLOVE_IMU_COUNT))
        {
            return false;
        }
    }

    return true;
}

static bool ImuCanTask_NodeToLocalIndex(const ImuCanTaskBusRuntime_t *bus,
                                        uint8_t node_id,
                                        uint32_t *index)
{
    if ((bus == NULL) || (bus->config == NULL) || (index == NULL) ||
        (node_id < bus->config->first_node_id) ||
        (node_id >= (uint8_t)(bus->config->first_node_id + bus->config->node_count)) ||
        (bus->config->node_count > IMU_CAN_TASK_MAX_NODES_PER_BUS))
    {
        return false;
    }

    *index = (uint32_t)node_id - bus->config->first_node_id;
    return true;
}

static bool ImuCanTask_LogicalNodeToBus(uint32_t logical_node_id,
                                        ImuCanTaskBusRuntime_t **bus_out,
                                        uint32_t *local_index_out)
{
    uint32_t output_index;

    if ((bus_out == NULL) || (local_index_out == NULL) ||
        (logical_node_id < IMU_CAN_TASK_LOGICAL_FIRST_NODE_ID))
    {
        return false;
    }

    output_index = logical_node_id - IMU_CAN_TASK_LOGICAL_FIRST_NODE_ID;
    if (output_index >= GLOVE_IMU_COUNT)
    {
        return false;
    }

    for (uint32_t bus_i = 0U; bus_i < IMU_CAN_TASK_BUS_COUNT; bus_i++)
    {
        ImuCanTaskBusRuntime_t *bus = &s_buses[bus_i];
        if ((bus->config == NULL) ||
            (bus->config->node_count > IMU_CAN_TASK_MAX_NODES_PER_BUS))
        {
            continue;
        }

        uint32_t count = bus->config->node_count;
        for (uint32_t local_i = 0U; local_i < count; local_i++)
        {
            uint32_t mapped_index;
            if ((ImuCanTask_BusOutputIndex(bus, local_i, &mapped_index) == true) &&
                (mapped_index == output_index))
            {
                *bus_out = bus;
                *local_index_out = local_i;
                return true;
            }
        }
    }

    return false;
}

static ImuCanTaskBusRuntime_t *ImuCanTask_FindBusByHandle(FDCAN_HandleTypeDef *hfdcan)
{
    for (uint32_t i = 0U; i < IMU_CAN_TASK_BUS_COUNT; i++)
    {
        if (s_buses[i].port.hfdcan == hfdcan)
        {
            return &s_buses[i];
        }
    }

    return NULL;
}

static void ImuCanTask_EnableFdcanIrq(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL)
    {
        return;
    }

    if (hfdcan->Instance == FDCAN1)
    {
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,
                             IMU_CAN_TASK_IRQ_PRIORITY,
                             IMU_CAN_TASK_IRQ_SUBPRIORITY);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
#if defined(FDCAN2) && defined(FDCAN2_IT0_IRQn)
    else if (hfdcan->Instance == FDCAN2)
    {
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn,
                             IMU_CAN_TASK_IRQ_PRIORITY,
                             IMU_CAN_TASK_IRQ_SUBPRIORITY);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    }
#endif
}

static bool ImuCanTask_ConfigFdcan(ImuCanTaskBusRuntime_t *bus)
{
    FDCAN_FilterTypeDef filter;

    if ((bus == NULL) || (bus->port.hfdcan == NULL))
    {
        s_imu_can_stats.last_error = 1U;
        return false;
    }

    (void)memset(&filter, 0, sizeof(filter));
    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_ACCEPT_IN_RX_FIFO0;
    filter.FilterID1 = 0x00000000UL;
    filter.FilterID2 = 0x00000000UL;

    if (HAL_FDCAN_ConfigFilter(bus->port.hfdcan, &filter) != HAL_OK)
    {
        s_imu_can_stats.last_error = 1U;
        return false;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(bus->port.hfdcan,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        s_imu_can_stats.last_error = 2U;
        return false;
    }

    if (HAL_FDCAN_Start(bus->port.hfdcan) != HAL_OK)
    {
        s_imu_can_stats.last_error = 3U;
        return false;
    }

    if (HAL_FDCAN_ConfigInterruptLines(bus->port.hfdcan,
                                       FDCAN_IT_GROUP_RX_FIFO0,
                                       FDCAN_INTERRUPT_LINE0) != HAL_OK)
    {
        s_imu_can_stats.last_error = 4U;
        return false;
    }

    if (HAL_FDCAN_ActivateNotification(bus->port.hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK)
    {
        s_imu_can_stats.last_error = 5U;
        return false;
    }

    ImuCanTask_EnableFdcanIrq(bus->port.hfdcan);
    bus->fdcan_started = true;
    return true;
}

static void ImuCanTask_UpdateLastRx(const hi04_can_frame_t *frame)
{
    uint32_t raw_id;

    if (frame == NULL)
    {
        return;
    }

    raw_id = frame->id & HI04_CAN_EFF_MASK;
    s_imu_can_stats.last_rx_id = raw_id;
    s_imu_can_stats.last_rx_is_extended = ((frame->id & HI04_CAN_EFF_FLAG) != 0U) ? 1U : 0U;
    s_imu_can_stats.last_rx_dlc = frame->dlc;
    (void)memset(s_imu_can_stats.last_rx_data, 0, sizeof(s_imu_can_stats.last_rx_data));
    if (frame->dlc <= sizeof(s_imu_can_stats.last_rx_data))
    {
        (void)memcpy(s_imu_can_stats.last_rx_data, frame->data, frame->dlc);
    }

    if ((((raw_id >> 16) & 0xFFU) == IMU_CAN_TASK_J1939_CONFIG_PF) &&
        (((raw_id >> 8) & 0xFFU) == IMU_CAN_TASK_J1939_HOST_ADDR))
    {
        uint32_t step = s_imu_can_stats.cfg_active_step;

        s_imu_can_stats.cfg_reply_count++;
        s_imu_can_stats.cfg_last_reply_id = raw_id;
        s_imu_can_stats.cfg_last_reply_dlc = frame->dlc;
        (void)memset(s_imu_can_stats.cfg_last_reply_data, 0,
                     sizeof(s_imu_can_stats.cfg_last_reply_data));
        if (frame->dlc <= sizeof(s_imu_can_stats.cfg_last_reply_data))
        {
            (void)memcpy(s_imu_can_stats.cfg_last_reply_data,
                         frame->data,
                         frame->dlc);
        }

        if ((step > 0U) && (step < 6U))
        {
            s_imu_can_stats.cfg_step_ack_count[step]++;
            s_imu_can_stats.cfg_step_ack_id[step] = raw_id;
            (void)memset(s_imu_can_stats.cfg_step_ack_data[step], 0,
                         sizeof(s_imu_can_stats.cfg_step_ack_data[step]));
            if (frame->dlc <= sizeof(s_imu_can_stats.cfg_step_ack_data[step]))
            {
                (void)memcpy(s_imu_can_stats.cfg_step_ack_data[step],
                             frame->data,
                             frame->dlc);
            }
        }
    }
}

static void ImuCanTask_DrainRxFifoForConfig(ImuCanTaskBusRuntime_t *bus)
{
    hi04_can_frame_t frame;

    if (bus == NULL)
    {
        return;
    }

    while (hi04_fdcan_stm32h563_read_fifo0(&bus->port, &frame))
    {
        s_imu_can_stats.rx_frame_count++;
        ImuCanTask_UpdateLastRx(&frame);
    }
}

static void ImuCanTask_DrainAllRxFifosForConfig(void)
{
    for (uint32_t i = 0U; i < IMU_CAN_TASK_BUS_COUNT; i++)
    {
        ImuCanTask_DrainRxFifoForConfig(&s_buses[i]);
    }
}

static void ImuCanTask_ConfigDelayMs(uint32_t delay_ms)
{
    uint32_t start_ms = HAL_GetTick();

    do
    {
        ImuCanTask_DrainAllRxFifosForConfig();
        osDelay(10U);
    } while ((HAL_GetTick() - start_ms) < delay_ms);
}

static bool ImuCanTask_SendJ1939ConfigU32(ImuCanTaskBusRuntime_t *bus,
                                          uint8_t dest_node_id,
                                          uint16_t reg_addr,
                                          uint32_t value)
{
    hi04_can_frame_t frame;
    uint32_t start_ms = HAL_GetTick();

    if ((bus == NULL) || (bus->fdcan_started == false))
    {
        return false;
    }

    while (HAL_FDCAN_GetTxFifoFreeLevel(bus->port.hfdcan) == 0U)
    {
        if ((HAL_GetTick() - start_ms) >= 20U)
        {
            return false;
        }
        osDelay(1U);
    }

    (void)memset(&frame, 0, sizeof(frame));
    frame.id = HI04_CAN_EFF_FLAG |
               ((uint32_t)IMU_CAN_TASK_J1939_PRIORITY << 26) |
               ((uint32_t)IMU_CAN_TASK_J1939_CONFIG_PF << 16) |
               ((uint32_t)dest_node_id << 8) |
               IMU_CAN_TASK_J1939_HOST_ADDR;
    frame.dlc = 8U;
    frame.data[0] = (uint8_t)(reg_addr & 0xFFU);
    frame.data[1] = (uint8_t)((reg_addr >> 8) & 0xFFU);
    frame.data[2] = 0x06U;
    frame.data[3] = 0x00U;
    frame.data[4] = (uint8_t)(value & 0xFFU);
    frame.data[5] = (uint8_t)((value >> 8) & 0xFFU);
    frame.data[6] = (uint8_t)((value >> 16) & 0xFFU);
    frame.data[7] = (uint8_t)((value >> 24) & 0xFFU);

    s_imu_can_stats.cfg_last_tx_id = frame.id & HI04_CAN_EFF_MASK;
    s_imu_can_stats.cfg_last_tx_dlc = frame.dlc;
    (void)memcpy(s_imu_can_stats.cfg_last_tx_data,
                 frame.data,
                 sizeof(s_imu_can_stats.cfg_last_tx_data));
    if ((s_imu_can_stats.cfg_active_step > 0U) && (s_imu_can_stats.cfg_active_step < 6U))
    {
        uint32_t step = s_imu_can_stats.cfg_active_step;
        s_imu_can_stats.cfg_step_tx_id[step] = frame.id & HI04_CAN_EFF_MASK;
        (void)memcpy(s_imu_can_stats.cfg_step_tx_data[step],
                     frame.data,
                     sizeof(s_imu_can_stats.cfg_step_tx_data[step]));
    }

    if (hi04_fdcan_stm32h563_send(&bus->port, &frame) == false)
    {
        return false;
    }

    s_imu_can_stats.cfg_tx_count++;
    ImuCanTask_ConfigDelayMs(50U);
    return true;
}

static bool ImuCanTask_SendJ1939OutputPeriod(ImuCanTaskBusRuntime_t *bus,
                                             uint8_t node_id,
                                             uint16_t output_addr,
                                             uint16_t period_ms)
{
    return ImuCanTask_SendJ1939ConfigU32(bus, node_id, output_addr, (uint32_t)period_ms);
}

static bool ImuCanTask_SendJ1939SetNodeId(ImuCanTaskBusRuntime_t *bus,
                                          uint8_t old_node_id,
                                          uint8_t new_node_id)
{
    return ImuCanTask_SendJ1939ConfigU32(bus,
                                         old_node_id,
                                         IMU_CAN_TASK_J1939_REG_NODE_ID,
                                         (uint32_t)new_node_id);
}

static ImuCanTaskBusRuntime_t *ImuCanTask_GetIdConfigBus(void)
{
    if (IMU_CAN_TASK_SET_IMU_BUS_INDEX >= IMU_CAN_TASK_BUS_COUNT)
    {
        return NULL;
    }
    return &s_buses[IMU_CAN_TASK_SET_IMU_BUS_INDEX];
}

static void ImuCanTask_SetJ1939NodeIdOnce(void)
{
#if IMU_CAN_TASK_ENABLE_SET_IMU_ID
    ImuCanTaskBusRuntime_t *bus = ImuCanTask_GetIdConfigBus();

    s_imu_can_stats.cfg_phase = 1U;
    if ((bus == NULL) || (bus->fdcan_started == false))
    {
        s_imu_can_stats.cfg_phase = 90U;
        return;
    }

    if ((IMU_CAN_TASK_SET_IMU_NEW_ID < 1U) || (IMU_CAN_TASK_SET_IMU_NEW_ID > 127U))
    {
        s_imu_can_stats.last_error = 20U;
        s_imu_can_stats.cfg_phase = 91U;
        return;
    }

    s_imu_can_stats.cfg_phase = 2U;
    s_imu_can_stats.cfg_active_step = 1U;
    if (ImuCanTask_SendJ1939SetNodeId(bus,
                                      IMU_CAN_TASK_SET_IMU_OLD_ID,
                                      IMU_CAN_TASK_SET_IMU_NEW_ID) == false)
    {
        s_imu_can_stats.last_error = 21U;
        s_imu_can_stats.cfg_phase = 92U;
        return;
    }

    s_imu_can_stats.cfg_active_step = 0U;
    s_imu_can_stats.cfg_phase = 20U;
#else
    (void)ImuCanTask_GetIdConfigBus;
    (void)ImuCanTask_SendJ1939SetNodeId;
#endif
}

static void ImuCanTask_LoopJ1939NodeIdConfig(void)
{
#if IMU_CAN_TASK_ENABLE_SET_IMU_ID
    ImuCanTaskBusRuntime_t *bus = ImuCanTask_GetIdConfigBus();

    if ((bus == NULL) || (bus->fdcan_started == false))
    {
        s_imu_can_stats.cfg_phase = 90U;
        return;
    }

    for (;;)
    {
        s_imu_can_stats.cfg_phase = 31U;
        s_imu_can_stats.cfg_active_step = 1U;
        if (ImuCanTask_SendJ1939SetNodeId(bus,
                                          IMU_CAN_TASK_SET_IMU_OLD_ID,
                                          IMU_CAN_TASK_SET_IMU_NEW_ID) == false)
        {
            s_imu_can_stats.last_error = 21U;
        }

        ImuCanTask_ConfigDelayMs(IMU_CAN_TASK_ID_CONFIG_LOOP_MS);
    }
#endif
}

static void ImuCanTask_ConfigJ1939Outputs(ImuCanTaskBusRuntime_t *bus, uint8_t node_id)
{
#if IMU_CAN_TASK_ENABLE_J1939_CONFIG
    if ((bus == NULL) || (bus->fdcan_started == false))
    {
        return;
    }

    if (ImuCanTask_SendJ1939ConfigU32(bus,
                                      node_id,
                                      IMU_CAN_TASK_J1939_REG_OUTPUT_EN,
                                      1UL) == false)
    {
        s_imu_can_stats.last_error = 10U;
    }
    if (ImuCanTask_SendJ1939OutputPeriod(bus,
                                         node_id,
                                         IMU_CAN_TASK_J1939_OUTPUT_ACCEL,
                                         IMU_CAN_TASK_OUTPUT_PERIOD_MS) == false)
    {
        s_imu_can_stats.last_error = 11U;
    }
    if (ImuCanTask_SendJ1939OutputPeriod(bus,
                                         node_id,
                                         IMU_CAN_TASK_J1939_OUTPUT_GYRO,
                                         IMU_CAN_TASK_OUTPUT_PERIOD_MS) == false)
    {
        s_imu_can_stats.last_error = 12U;
    }
    if (ImuCanTask_SendJ1939OutputPeriod(bus,
                                         node_id,
                                         IMU_CAN_TASK_J1939_OUTPUT_QUAT,
                                         IMU_CAN_TASK_OUTPUT_PERIOD_MS) == false)
    {
        s_imu_can_stats.last_error = 13U;
    }
    (void)ImuCanTask_SendJ1939OutputPeriod(bus, node_id, IMU_CAN_TASK_J1939_OUTPUT_RPY, 0U);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus, node_id, IMU_CAN_TASK_J1939_OUTPUT_YAW, 0U);
#else
    (void)bus;
    (void)node_id;
#endif
}

static void ImuCanTask_ClearJ1939PeriodsOnce(ImuCanTaskBusRuntime_t *bus, uint8_t node_id)
{
#if IMU_CAN_TASK_CLEAR_J1939_PERIODS_ONCE
    if ((bus == NULL) || (bus->fdcan_started == false))
    {
        return;
    }

    (void)ImuCanTask_SendJ1939OutputPeriod(bus, node_id, IMU_CAN_TASK_J1939_OUTPUT_ACCEL, 0U);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus, node_id, IMU_CAN_TASK_J1939_OUTPUT_GYRO, 0U);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus, node_id, IMU_CAN_TASK_J1939_OUTPUT_QUAT, 0U);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus, node_id, IMU_CAN_TASK_J1939_OUTPUT_RPY, 0U);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus, node_id, IMU_CAN_TASK_J1939_OUTPUT_YAW, 0U);
    (void)ImuCanTask_SendJ1939ConfigU32(bus, node_id, IMU_CAN_TASK_J1939_REG_CONTROL, 0UL);
#else
    (void)bus;
    (void)node_id;
#endif
}

static void ImuCanTask_ConfigJ1939SyncOnce(ImuCanTaskBusRuntime_t *bus, uint8_t node_id)
{
#if IMU_CAN_TASK_CONFIG_J1939_SYNC_ONCE
    if ((bus == NULL) || (bus->fdcan_started == false))
    {
        return;
    }

    s_imu_can_stats.cfg_active_step = node_id;
    (void)ImuCanTask_SendJ1939ConfigU32(bus, node_id, IMU_CAN_TASK_J1939_REG_OUTPUT_EN, 1UL);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus,
                                           node_id,
                                           IMU_CAN_TASK_J1939_OUTPUT_ACCEL,
                                           IMU_CAN_TASK_J1939_SYNC_TRIGGER_VAL);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus,
                                           node_id,
                                           IMU_CAN_TASK_J1939_OUTPUT_GYRO,
                                           IMU_CAN_TASK_J1939_SYNC_TRIGGER_VAL);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus,
                                           node_id,
                                           IMU_CAN_TASK_J1939_OUTPUT_QUAT,
                                           IMU_CAN_TASK_J1939_SYNC_TRIGGER_VAL);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus, node_id, IMU_CAN_TASK_J1939_OUTPUT_RPY, 0U);
    (void)ImuCanTask_SendJ1939OutputPeriod(bus, node_id, IMU_CAN_TASK_J1939_OUTPUT_YAW, 0U);
    (void)ImuCanTask_SendJ1939ConfigU32(bus, node_id, IMU_CAN_TASK_J1939_REG_CONTROL, 0UL);
    s_imu_can_stats.cfg_active_step = 0U;
#else
    (void)bus;
    (void)node_id;
#endif
}

static void ImuCanTask_InitHi04Devices(ImuCanTaskBusRuntime_t *bus)
{
    hi04_bus_t hi04_bus;

    if ((bus == NULL) || (bus->config == NULL))
    {
        return;
    }

    hi04_bus.send = hi04_fdcan_stm32h563_send;
    hi04_bus.ctx = &bus->port;

    for (uint32_t i = 0U; i < bus->config->node_count; i++)
    {
        uint8_t node_id = (uint8_t)(bus->config->first_node_id + i);
        hi04_device_init(&bus->devices[i], hi04_bus, node_id);

        if (i < bus->config->config_node_count)
        {
            ImuCanTask_ConfigJ1939Outputs(bus, node_id);
            ImuCanTask_ClearJ1939PeriodsOnce(bus, node_id);
            ImuCanTask_ConfigJ1939SyncOnce(bus, node_id);
        }
    }
}

static void ImuCanTask_ProcessFrame(ImuCanTaskBusRuntime_t *bus,
                                    const hi04_can_frame_t *frame)
{
    uint32_t index;
    uint8_t node_id;
    hi04_msg_type_t msg_type;

    if ((bus == NULL) || (frame == NULL) || ((frame->id & HI04_CAN_EFF_FLAG) == 0U))
    {
        return;
    }

    node_id = hi04_can_extract_node_id(frame->id);
    if (ImuCanTask_NodeToLocalIndex(bus, node_id, &index) == false)
    {
        s_imu_can_stats.rejected_node_count++;
        return;
    }

    if (hi04_device_process_frame(&bus->devices[index], frame, &msg_type))
    {
        AcqSyncSnapshot_t sync;

        bus->node_last_rx_id[index] = frame->id & HI04_CAN_EFF_MASK;
        bus->node_last_rx_dlc[index] = frame->dlc;
        (void)memset(bus->node_last_rx_data[index], 0,
                     sizeof(bus->node_last_rx_data[index]));
        if (frame->dlc <= sizeof(bus->node_last_rx_data[index]))
        {
            (void)memcpy(bus->node_last_rx_data[index], frame->data, frame->dlc);
        }
        if (AcqSync_GetLatest(&sync) != 0U)
        {
            if (bus->node_sync_seq[index] != sync.seq)
            {
                bus->node_sync_seen_mask[index] = 0U;
            }
            bus->node_sync_seq[index] = sync.seq;
            bus->node_sync_timestamp_us[index] = sync.timestamp_us;
            bus->node_sync_seen_mask[index] |= ImuCanTask_SeenBitForMsg(msg_type);
            bus->node_sync_valid[index] = 1U;
        }
        s_imu_can_stats.parsed_frame_count++;
    }
    else
    {
        s_imu_can_stats.unparsed_frame_count++;
    }
}

static void ImuCanTask_DrainRxFifo(ImuCanTaskBusRuntime_t *bus)
{
    hi04_can_frame_t frame;

    if (bus == NULL)
    {
        return;
    }

    while (hi04_fdcan_stm32h563_read_fifo0(&bus->port, &frame))
    {
        s_imu_can_stats.rx_frame_count++;
        ImuCanTask_UpdateLastRx(&frame);
        ImuCanTask_ProcessFrame(bus, &frame);
    }
}

static void ImuCanTask_DrainAllRxFifos(void)
{
    for (uint32_t i = 0U; i < IMU_CAN_TASK_BUS_COUNT; i++)
    {
        ImuCanTask_DrainRxFifo(&s_buses[i]);
    }
}

static void ImuCanTask_GetQuaternion(const hi04_sample_t *sample,
                                     uint32_t seen_mask,
                                     float *qw,
                                     float *qx,
                                     float *qy,
                                     float *qz,
                                     uint32_t *source)
{
    if ((sample == NULL) || (qw == NULL) || (qx == NULL) ||
        (qy == NULL) || (qz == NULL))
    {
        return;
    }

    if (source != NULL)
    {
        *source = IMU_CAN_TASK_QUAT_SOURCE_NONE;
    }

    if ((seen_mask & HI04_SEEN_QUAT) != 0U)
    {
        *qw = sample->quat_w;
        *qx = sample->quat_x;
        *qy = sample->quat_y;
        *qz = sample->quat_z;
        if (source != NULL)
        {
            *source = IMU_CAN_TASK_QUAT_SOURCE_RAW;
        }
        return;
    }

    *qw = 0.0f;
    *qx = 0.0f;
    *qy = 0.0f;
    *qz = 0.0f;
}

static void ImuCanTask_FillOneImu(GloveImuSensorData_t *data,
                                  uint32_t index,
                                  uint32_t seen_mask,
                                  const hi04_sample_t *sample)
{
    float qw;
    float qx;
    float qy;
    float qz;

    data->imu[index].accel_mps2.x = sample->acc_x * IMU_CAN_TASK_G_TO_MPS2;
    data->imu[index].accel_mps2.y = sample->acc_y * IMU_CAN_TASK_G_TO_MPS2;
    data->imu[index].accel_mps2.z = sample->acc_z * IMU_CAN_TASK_G_TO_MPS2;
    data->imu[index].gyro_radps.x = sample->gyr_x * IMU_CAN_TASK_DEG_TO_RAD;
    data->imu[index].gyro_radps.y = sample->gyr_y * IMU_CAN_TASK_DEG_TO_RAD;
    data->imu[index].gyro_radps.z = sample->gyr_z * IMU_CAN_TASK_DEG_TO_RAD;

    ImuCanTask_GetQuaternion(sample, seen_mask, &qw, &qx, &qy, &qz, NULL);
    data->quat[index].w = qw;
    data->quat[index].x = qx;
    data->quat[index].y = qy;
    data->quat[index].z = qz;
}

static void ImuCanTask_PublishSnapshot(void)
{
    GloveImuSensorBlock_t *block;
    AcqSyncSnapshot_t sync;
    GloveTimestampUs_t block_sync_timestamp_us = 0ULL;
    uint32_t block_sync_seq = 0U;
    uint8_t any_valid = 0U;
    uint8_t any_quat_valid = 0U;

    block = DataManager_AllocImuSensor();
    if (block == NULL)
    {
        s_imu_can_stats.publish_drop_count++;
        return;
    }

    for (uint32_t bus_i = 0U; bus_i < IMU_CAN_TASK_BUS_COUNT; bus_i++)
    {
        ImuCanTaskBusRuntime_t *bus = &s_buses[bus_i];

        for (uint32_t local_i = 0U; local_i < bus->config->node_count; local_i++)
        {
            uint32_t out_i;
            if (ImuCanTask_BusOutputIndex(bus, local_i, &out_i) == false)
            {
                continue;
            }

            if ((bus->node_sync_valid[local_i] != 0U) &&
                (bus->node_sync_seq[local_i] >= block_sync_seq))
            {
                block_sync_seq = bus->node_sync_seq[local_i];
                block_sync_timestamp_us = bus->node_sync_timestamp_us[local_i];
            }
        }
    }

    for (uint32_t bus_i = 0U; bus_i < IMU_CAN_TASK_BUS_COUNT; bus_i++)
    {
        ImuCanTaskBusRuntime_t *bus = &s_buses[bus_i];

        for (uint32_t local_i = 0U; local_i < bus->config->node_count; local_i++)
        {
            uint32_t out_i;
            uint32_t seen_mask;

            if (ImuCanTask_BusOutputIndex(bus, local_i, &out_i) == false)
            {
                continue;
            }

            if (block_sync_seq != 0U)
            {
                if ((bus->node_sync_valid[local_i] == 0U) ||
                    (bus->node_sync_seq[local_i] != block_sync_seq))
                {
                    continue;
                }
                seen_mask = bus->node_sync_seen_mask[local_i];
            }
            else
            {
                seen_mask = bus->devices[local_i].seen_mask;
            }

            if ((seen_mask & IMU_CAN_TASK_FRAME_FLAGS_REQUIRED) ==
                IMU_CAN_TASK_FRAME_FLAGS_REQUIRED)
            {
                ImuCanTask_FillOneImu(&block->data,
                                       out_i,
                                       seen_mask,
                                       &bus->devices[local_i].latest);
                if ((seen_mask & HI04_SEEN_QUAT) != 0U)
                {
                    block->data.valid_flags |= GLOVE_FRAME_VALID_IMU_BIT(out_i);
                    any_quat_valid = 1U;
                }
                any_valid = 1U;
            }
        }
    }

    if (any_valid != 0U)
    {
        if (block_sync_seq != 0U)
        {
            block->data.sensor_seq = block_sync_seq;
            block->data.timestamp_us = block_sync_timestamp_us;
        }
        else if (AcqSync_GetLatest(&sync) != 0U)
        {
            block->data.sensor_seq = sync.seq;
            block->data.timestamp_us = sync.timestamp_us;
        }
        else
        {
            block->data.sensor_seq = s_sensor_seq++;
            block->data.timestamp_us = (GloveTimestampUs_t)ImuCanTask_TimeUs();
        }

        block->data.valid_flags |= GLOVE_FRAME_FLAG_IMU_VALID;
        if (any_quat_valid != 0U)
        {
            block->data.valid_flags |= GLOVE_FRAME_FLAG_QUAT_VALID;
        }

        if (DataManager_PublishImuSensor(block, IMU_CAN_TASK_QUEUE_TIMEOUT_MS) ==
            GLOVE_STATUS_OK)
        {
            s_imu_can_stats.published_count++;
            return;
        }
        s_imu_can_stats.publish_drop_count++;
    }

    (void)DataManager_ReleaseImuSensor(block);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    ImuCanTaskBusRuntime_t *bus = ImuCanTask_FindBusByHandle(hfdcan);

    if ((bus != NULL) &&
        ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) &&
        (imuCanTaskHandle != NULL))
    {
        s_imu_can_stats.rx_irq_count++;
        (void)osThreadFlagsSet(imuCanTaskHandle, IMU_CAN_TASK_RX_FLAG);
    }
}

static void ImuCanTask_CopyStatsToSnapshot(ImuCanTaskDebugSnapshot_t *snapshot)
{
    (void)memset(snapshot, 0, sizeof(*snapshot));
    snapshot->rx_irq_count = s_imu_can_stats.rx_irq_count;
    snapshot->rx_frame_count = s_imu_can_stats.rx_frame_count;
    snapshot->parsed_frame_count = s_imu_can_stats.parsed_frame_count;
    snapshot->published_count = s_imu_can_stats.published_count;
    snapshot->publish_drop_count = s_imu_can_stats.publish_drop_count;
    snapshot->init_error_count = s_imu_can_stats.init_error_count;
    snapshot->last_error = s_imu_can_stats.last_error;
    snapshot->unparsed_frame_count = s_imu_can_stats.unparsed_frame_count;
    snapshot->rejected_node_count = s_imu_can_stats.rejected_node_count;
    snapshot->last_rx_id = s_imu_can_stats.last_rx_id;
    snapshot->last_rx_is_extended = s_imu_can_stats.last_rx_is_extended;
    snapshot->last_rx_dlc = s_imu_can_stats.last_rx_dlc;
    (void)memcpy(snapshot->last_rx_data,
                 s_imu_can_stats.last_rx_data,
                 sizeof(snapshot->last_rx_data));
    snapshot->cfg_phase = s_imu_can_stats.cfg_phase;
    snapshot->cfg_target_new_id = IMU_CAN_TASK_SET_IMU_NEW_ID;
    snapshot->cfg_tx_count = s_imu_can_stats.cfg_tx_count;
    snapshot->cfg_last_tx_id = s_imu_can_stats.cfg_last_tx_id;
    snapshot->cfg_last_tx_dlc = s_imu_can_stats.cfg_last_tx_dlc;
    (void)memcpy(snapshot->cfg_last_tx_data,
                 s_imu_can_stats.cfg_last_tx_data,
                 sizeof(snapshot->cfg_last_tx_data));
    snapshot->cfg_reply_count = s_imu_can_stats.cfg_reply_count;
    snapshot->cfg_last_reply_id = s_imu_can_stats.cfg_last_reply_id;
    snapshot->cfg_last_reply_dlc = s_imu_can_stats.cfg_last_reply_dlc;
    (void)memcpy(snapshot->cfg_last_reply_data,
                 s_imu_can_stats.cfg_last_reply_data,
                 sizeof(snapshot->cfg_last_reply_data));

    for (uint32_t i = 0U; i < 6U; i++)
    {
        snapshot->cfg_step_ack_count[i] = s_imu_can_stats.cfg_step_ack_count[i];
        snapshot->cfg_step_ack_id[i] = s_imu_can_stats.cfg_step_ack_id[i];
        (void)memcpy(snapshot->cfg_step_ack_data[i],
                     s_imu_can_stats.cfg_step_ack_data[i],
                     sizeof(snapshot->cfg_step_ack_data[i]));
        snapshot->cfg_step_tx_id[i] = s_imu_can_stats.cfg_step_tx_id[i];
        (void)memcpy(snapshot->cfg_step_tx_data[i],
                     s_imu_can_stats.cfg_step_tx_data[i],
                     sizeof(snapshot->cfg_step_tx_data[i]));
    }
}

static void ImuCanTask_FillDebugFromDevice(ImuCanTaskDebugSnapshot_t *snapshot,
                                           ImuCanTaskBusRuntime_t *bus,
                                           uint32_t local_index)
{
    const hi04_sample_t *sample = &bus->devices[local_index].latest;
    float qw;
    float qx;
    float qy;
    float qz;
    uint32_t quat_source;

    ImuCanTask_GetQuaternion(sample,
                             bus->devices[local_index].seen_mask,
                             &qw,
                             &qx,
                             &qy,
                             &qz,
                             &quat_source);

    snapshot->first_valid_node_id = ImuCanTask_BusLogicalNodeId(bus, local_index);
    snapshot->first_valid_seen_mask = bus->devices[local_index].seen_mask;
    snapshot->node_last_rx_id = bus->node_last_rx_id[local_index];
    snapshot->node_last_rx_dlc = bus->node_last_rx_dlc[local_index];
    (void)memcpy(snapshot->node_last_rx_data,
                 bus->node_last_rx_data[local_index],
                 sizeof(snapshot->node_last_rx_data));
    snapshot->accel_x_mg = (int32_t)(sample->acc_x * 1000.0f);
    snapshot->accel_y_mg = (int32_t)(sample->acc_y * 1000.0f);
    snapshot->accel_z_mg = (int32_t)(sample->acc_z * 1000.0f);
    snapshot->gyro_x_mdps = (int32_t)(sample->gyr_x * 1000.0f);
    snapshot->gyro_y_mdps = (int32_t)(sample->gyr_y * 1000.0f);
    snapshot->gyro_z_mdps = (int32_t)(sample->gyr_z * 1000.0f);
    snapshot->quat_source = quat_source;
    snapshot->quat_w_1e4 = (int32_t)(qw * 10000.0f);
    snapshot->quat_x_1e4 = (int32_t)(qx * 10000.0f);
    snapshot->quat_y_1e4 = (int32_t)(qy * 10000.0f);
    snapshot->quat_z_1e4 = (int32_t)(qz * 10000.0f);
}

void ImuCanTask_GetDebugSnapshot(ImuCanTaskDebugSnapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    ImuCanTask_CopyStatsToSnapshot(snapshot);

    for (uint32_t bus_i = 0U; bus_i < IMU_CAN_TASK_BUS_COUNT; bus_i++)
    {
        ImuCanTaskBusRuntime_t *bus = &s_buses[bus_i];
        if (bus->config == NULL)
        {
            continue;
        }

        for (uint32_t local_i = 0U; local_i < bus->config->node_count; local_i++)
        {
            if ((bus->devices[local_i].seen_mask & IMU_CAN_TASK_FRAME_FLAGS_REQUIRED) ==
                IMU_CAN_TASK_FRAME_FLAGS_REQUIRED)
            {
                ImuCanTask_FillDebugFromDevice(snapshot, bus, local_i);
                return;
            }
        }
    }
}

void ImuCanTask_GetDebugSnapshotForNode(uint32_t node_id,
                                        ImuCanTaskDebugSnapshot_t *snapshot)
{
    ImuCanTaskBusRuntime_t *bus;
    uint32_t local_index;

    if (snapshot == NULL)
    {
        return;
    }

    ImuCanTask_CopyStatsToSnapshot(snapshot);

    if (ImuCanTask_LogicalNodeToBus(node_id, &bus, &local_index) == false)
    {
        return;
    }

    if (bus->devices[local_index].seen_mask != 0U)
    {
        ImuCanTask_FillDebugFromDevice(snapshot, bus, local_index);
    }
}

uint32_t ImuCanTask_GetFirstNodeId(void)
{
    return IMU_CAN_TASK_LOGICAL_FIRST_NODE_ID;
}

uint32_t ImuCanTask_GetNodeCount(void)
{
    return ImuCanTask_TotalLogicalNodeCount();
}

void ImuCanTask(void *argument)
{
    uint32_t last_publish_ms;

    (void)argument;
    (void)ImuCanTask_LoopJ1939NodeIdConfig;

    for (uint32_t i = 0U; i < IMU_CAN_TASK_BUS_COUNT; i++)
    {
        s_buses[i].config = &s_bus_configs[i];
        s_buses[i].port.hfdcan = s_bus_configs[i].hfdcan;
        s_buses[i].port.time_us = ImuCanTask_TimeUs;

        if ((s_bus_configs[i].node_count > IMU_CAN_TASK_MAX_NODES_PER_BUS) ||
            (ImuCanTask_ValidateBusMap(i, s_bus_configs[i].node_count) == false))
        {
            s_imu_can_stats.init_error_count++;
            s_imu_can_stats.last_error = 80U + i;
            continue;
        }

        if (ImuCanTask_ConfigFdcan(&s_buses[i]) == false)
        {
            s_imu_can_stats.init_error_count++;
        }
    }

#if IMU_CAN_TASK_ENABLE_SET_IMU_ID
    ImuCanTask_LoopJ1939NodeIdConfig();
#else
    ImuCanTask_SetJ1939NodeIdOnce();
#endif

    for (uint32_t i = 0U; i < IMU_CAN_TASK_BUS_COUNT; i++)
    {
        ImuCanTask_InitHi04Devices(&s_buses[i]);
    }

    last_publish_ms = HAL_GetTick();

    for (;;)
    {
        (void)osThreadFlagsWait(IMU_CAN_TASK_RX_FLAG,
                                osFlagsWaitAny,
                                IMU_CAN_TASK_PUBLISH_PERIOD_MS);
        ImuCanTask_DrainAllRxFifos();

#if IMU_CAN_TASK_ENABLE_PUBLISH
        if ((HAL_GetTick() - last_publish_ms) >= IMU_CAN_TASK_PUBLISH_PERIOD_MS)
        {
            ImuCanTask_PublishSnapshot();
            last_publish_ms = HAL_GetTick();
        }
#else
        (void)last_publish_ms;
#endif
    }
}
