#include "hi04_j1939.h"

#include <string.h>

static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t le_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] |
                     ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

//瑙ｆ瀽鍑芥暟
hi04_parse_result_t hi04_j1939_parse(const hi04_can_frame_t *frame)
{
    hi04_parse_result_t result;
    memset(&result, 0, sizeof(result));
    result.type = HI04_MSG_UNKNOWN;

    if (frame == 0) {
        return result;
    }

    if ((frame->id & HI04_CAN_EFF_FLAG) != 0u) {
        const uint32_t id = frame->id & HI04_CAN_EFF_MASK;
        const uint8_t pf = (uint8_t)((id >> 16) & 0xFFu);
        const uint8_t ps = (uint8_t)((id >> 8) & 0xFFu);
        hi04_sample_t *s = &result.sample;

        if (pf != 0xFFu || frame->dlc < 8u) {
            return result;
        }

        s->node_id = hi04_can_extract_node_id(frame->id);
        s->timestamp_us = frame->timestamp_us;

        switch (ps) {
        case 0x34u:
            s->acc_x = le_i16(&frame->data[0]) * 0.00048828f;
            s->acc_y = le_i16(&frame->data[2]) * 0.00048828f;
            s->acc_z = le_i16(&frame->data[4]) * 0.00048828f;
            result.type = HI04_MSG_ACCEL;
            break;
        case 0x37u:
            s->gyr_x = le_i16(&frame->data[0]) * 0.061035f;
            s->gyr_y = le_i16(&frame->data[2]) * 0.061035f;
            s->gyr_z = le_i16(&frame->data[4]) * 0.061035f;
            result.type = HI04_MSG_GYRO;
            break;
        case 0x3Du:
            s->roll = le_i32(&frame->data[0]) * 0.001f;
            s->pitch = le_i32(&frame->data[4]) * 0.001f;
            result.type = HI04_MSG_PITCH_ROLL;
            break;
        case 0x41u:
            s->yaw = le_i32(&frame->data[4]) * 0.001f;
            result.type = HI04_MSG_YAW;
            break;
        case 0x46u:
            s->quat_w = le_i16(&frame->data[0]) * 0.0001f;
            s->quat_x = le_i16(&frame->data[2]) * 0.0001f;
            s->quat_y = le_i16(&frame->data[4]) * 0.0001f;
            s->quat_z = le_i16(&frame->data[6]) * 0.0001f;
            result.type = HI04_MSG_QUAT;
            break;
        default:
            return result;
        }

        result.valid = true;
        return result;
    }

    return result;
}
