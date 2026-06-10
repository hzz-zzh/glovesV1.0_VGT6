#ifndef HI04_J1939_H
#define HI04_J1939_H

#include "hi04_can.h"

#ifdef __cplusplus
extern "C" {
#endif

hi04_parse_result_t hi04_j1939_parse(const hi04_can_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif
