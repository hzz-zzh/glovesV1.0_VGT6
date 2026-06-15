#ifndef GLOVE_HAND_CONFIG_H
#define GLOVE_HAND_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_data.h"

void GloveHandConfig_InitFromGpio(void);
GloveHandSide_t GloveHandConfig_GetHandSide(void);
GloveStatus_t GloveHandConfig_SetHandSide(GloveHandSide_t hand_side);

#ifdef __cplusplus
}
#endif

#endif /* GLOVE_HAND_CONFIG_H */
