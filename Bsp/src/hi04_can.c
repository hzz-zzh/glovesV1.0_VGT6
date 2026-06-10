#include "hi04_can.h"

uint8_t hi04_can_extract_node_id(uint32_t can_id)  //读取CAN ID中的节点ID部分 COB-ID = 功能码基地址 + Node-ID
{
    return (uint8_t)((can_id & HI04_CAN_EFF_FLAG) ? 
    (can_id & 0xFFu) : 
    (can_id & 0x7Fu));
}
