#include "hi04_driver.h"

#include <string.h>

bool hi04_device_send_frame(hi04_device_t *dev, const hi04_can_frame_t *frame)
{
    return dev != 0 && dev->bus.send != 0 && dev->bus.send(dev->bus.ctx, frame);
}

void hi04_device_init(hi04_device_t *dev,
                      hi04_bus_t bus,
                      uint8_t node_id)
{
    if (dev == 0) return;
    memset(dev, 0, sizeof(*dev));
    dev->bus = bus;
    dev->node_id = node_id;
}

static uint32_t seen_bit(hi04_msg_type_t type)
{
    switch (type) {
    case HI04_MSG_ACCEL: return HI04_SEEN_ACCEL;
    case HI04_MSG_GYRO: return HI04_SEEN_GYRO;
    case HI04_MSG_MAG: return HI04_SEEN_MAG;
    case HI04_MSG_QUAT: return HI04_SEEN_QUAT;
    case HI04_MSG_PITCH_ROLL: return HI04_SEEN_PITCH_ROLL;
    case HI04_MSG_YAW: return HI04_SEEN_YAW;
    case HI04_MSG_ENV: return HI04_SEEN_ENV;
    case HI04_MSG_TIME: return HI04_SEEN_TIME;
    case HI04_MSG_CANFD0: return HI04_SEEN_CANFD0;
    default: return 0u;
    }
}

static void merge_sample(hi04_sample_t *dst, const hi04_sample_t *src, hi04_msg_type_t type)
{
    dst->node_id = src->node_id;
    dst->timestamp_us = src->timestamp_us;
    switch (type) {
    case HI04_MSG_ACCEL:
        dst->acc_x = src->acc_x; dst->acc_y = src->acc_y; dst->acc_z = src->acc_z;
        break;
    case HI04_MSG_GYRO:
        dst->gyr_x = src->gyr_x; dst->gyr_y = src->gyr_y; dst->gyr_z = src->gyr_z;
        break;
    case HI04_MSG_MAG:
        dst->mag_x = src->mag_x; dst->mag_y = src->mag_y; dst->mag_z = src->mag_z;
        break;
    case HI04_MSG_QUAT:
        dst->quat_w = src->quat_w; dst->quat_x = src->quat_x;
        dst->quat_y = src->quat_y; dst->quat_z = src->quat_z;
        break;
    case HI04_MSG_PITCH_ROLL:
        dst->roll = src->roll; dst->pitch = src->pitch;
        break;
    case HI04_MSG_YAW:
        dst->yaw = src->yaw;
        break;
    case HI04_MSG_ENV:
        dst->temperature = src->temperature; dst->pressure = src->pressure;
        break;
    case HI04_MSG_TIME:
        dst->utc_year = src->utc_year; dst->utc_month = src->utc_month; dst->utc_day = src->utc_day;
        dst->utc_hour = src->utc_hour; dst->utc_minute = src->utc_minute; dst->utc_second = src->utc_second;
        dst->utc_millisecond = src->utc_millisecond; dst->utc_ms_of_day = src->utc_ms_of_day;
        break;
    case HI04_MSG_CANFD0:
        *dst = *src;
        break;
    default:
        break;
    }
}

bool hi04_device_process_frame(hi04_device_t *dev,
                               const hi04_can_frame_t *frame,
                               hi04_msg_type_t *out_type)
{
    if (dev == 0 || frame == 0) return false;
    if (hi04_can_extract_node_id(frame->id) != dev->node_id) return false;

    hi04_parse_result_t parsed = hi04_j1939_parse(frame);
    if (!parsed.valid) return false;

    merge_sample(&dev->latest, &parsed.sample, parsed.type);
    dev->seen_mask |= seen_bit(parsed.type);
    if (out_type != 0) *out_type = parsed.type;
    return true;
}
