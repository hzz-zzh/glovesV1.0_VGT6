#ifndef HI04_CAN_H
#define HI04_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HI04_CAN_EFF_FLAG 0x80000000UL
#define HI04_CAN_SFF_MASK 0x000007FFUL
#define HI04_CAN_EFF_MASK 0x1FFFFFFFUL

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[64];
    uint64_t timestamp_us;
} hi04_can_frame_t;

typedef enum {
    HI04_MSG_NONE = 0,
    HI04_MSG_ACCEL,
    HI04_MSG_GYRO,
    HI04_MSG_MAG,
    HI04_MSG_QUAT,
    HI04_MSG_PITCH_ROLL,
    HI04_MSG_YAW,
    HI04_MSG_ENV,
    HI04_MSG_TIME,
    HI04_MSG_CANFD0,
    HI04_MSG_UNKNOWN
} hi04_msg_type_t;

typedef struct {
    uint8_t node_id;
    uint64_t timestamp_us;
    uint16_t main_status;
    uint32_t system_time_ms;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    float roll;
    float pitch;
    float yaw;
    float temperature;
    float pressure;
    uint8_t utc_year;
    uint8_t utc_month;
    uint8_t utc_day;
    uint8_t utc_hour;
    uint8_t utc_minute;
    uint8_t utc_second;
    uint16_t utc_millisecond;
    uint32_t utc_ms_of_day;
} hi04_sample_t;
//40-82-97
typedef struct {
    bool valid;
    hi04_msg_type_t type;
    hi04_sample_t sample;
} hi04_parse_result_t;

uint8_t hi04_can_extract_node_id(uint32_t can_id);

#ifdef __cplusplus
}
#endif

#endif
