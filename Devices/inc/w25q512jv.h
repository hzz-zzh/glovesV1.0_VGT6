#ifndef W25Q512JV_H
#define W25Q512JV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "app_data.h"

#define W25Q512JV_SIZE_BYTES                (0x04000000UL)
#define W25Q512JV_SECTOR_SIZE              (4096UL)
#define W25Q512JV_PAGE_SIZE                (256UL)
#define W25Q512JV_JEDEC_ID                 (0xEF4020UL)
#define W25Q512JV_DEFAULT_TIMEOUT_MS       (1000UL)
#define W25Q512JV_ERASE_TIMEOUT_MS         (1000UL)
#define W25Q512JV_PROGRAM_TIMEOUT_MS       (100UL)

typedef struct
{
    uint32_t jedec_id;
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity;
    uint8_t reserved;
} W25q512jvId_t;

GloveStatus_t W25q512jv_Init(void);
GloveStatus_t W25q512jv_ReadJedecId(W25q512jvId_t *id);
GloveStatus_t W25q512jv_Read(uint32_t address,
                             uint8_t *data,
                             uint32_t size,
                             uint32_t timeout_ms);
GloveStatus_t W25q512jv_PageProgram(uint32_t address,
                                    const uint8_t *data,
                                    uint32_t size,
                                    uint32_t timeout_ms);
GloveStatus_t W25q512jv_Write(uint32_t address,
                              const uint8_t *data,
                              uint32_t size,
                              uint32_t timeout_ms);
GloveStatus_t W25q512jv_EraseSector(uint32_t address,
                                    uint32_t timeout_ms);
GloveStatus_t W25q512jv_WaitReady(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* W25Q512JV_H */
