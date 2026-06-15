#include "glove_hand_config.h"

#include "main.h"

static volatile GloveHandSide_t s_hand_side = GLOVE_HAND_LEFT;

static uint8_t GloveHandConfig_IsValidHandSide(GloveHandSide_t hand_side)
{
    return ((hand_side == GLOVE_HAND_LEFT) || (hand_side == GLOVE_HAND_RIGHT)) ? 1U : 0U;
}

void GloveHandConfig_InitFromGpio(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(L_R_HAND_FLAG_GPIO_Port, L_R_HAND_FLAG_Pin);

    s_hand_side = (state == GPIO_PIN_SET) ? GLOVE_HAND_RIGHT : GLOVE_HAND_LEFT;
}

GloveHandSide_t GloveHandConfig_GetHandSide(void)
{
    return s_hand_side;
}

GloveStatus_t GloveHandConfig_SetHandSide(GloveHandSide_t hand_side)
{
    if (GloveHandConfig_IsValidHandSide(hand_side) == 0U)
    {
        return GLOVE_STATUS_INVALID_PARAM;
    }

    s_hand_side = hand_side;
    return GLOVE_STATUS_OK;
}
