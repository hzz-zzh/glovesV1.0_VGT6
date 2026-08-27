#include "ff.h"
#include "diskio.h"

#include "cmsis_os2.h"
#include "main.h"
#include "modbus_time_sync.h"
#include "sd_diskio.h"
#include <stdint.h>
#include <string.h>

#define SD_DISK_PDRV               (0U)
#define SD_DISK_BLOCK_SIZE         (512U)
#define SD_DISK_TIMEOUT_MS         (5000U)

typedef enum
{
  SD_DMA_TRANSFER_IDLE = 0U,
  SD_DMA_TRANSFER_PENDING,
  SD_DMA_TRANSFER_COMPLETE,
  SD_DMA_TRANSFER_ERROR
} SdDmaTransferState_t;

static volatile DSTATUS sd_status = STA_NOINIT;
volatile uint32_t sd_disk_last_hal_status = 0U;
volatile uint32_t sd_disk_last_hal_error = 0U;
volatile uint32_t sd_disk_last_result = RES_OK;
volatile uint32_t sd_disk_init_count = 0U;
volatile uint32_t sd_disk_last_state_before = 0U;
volatile uint32_t sd_disk_last_state_after = 0U;
volatile uint32_t sd_disk_last_card_state = 0U;

static osSemaphoreId_t sd_dma_sem;
static volatile SdDmaTransferState_t sd_dma_transfer_state = SD_DMA_TRANSFER_IDLE;
static __ALIGNED(4) uint8_t sd_scratch[SD_DISK_BLOCK_SIZE];

static DRESULT SdDisk_SetResult(DRESULT result, HAL_StatusTypeDef hal_status)
{
  sd_disk_last_result = (uint32_t)result;
  sd_disk_last_hal_status = (uint32_t)hal_status;
  sd_disk_last_hal_error = HAL_SD_GetError(&hsd1);

  return result;
}

static DRESULT SdDisk_SetError(HAL_StatusTypeDef hal_status)
{
  (void)HAL_SD_Abort(&hsd1);
  sd_status = STA_NOINIT;
  return SdDisk_SetResult(RES_ERROR, hal_status);
}

static void SdDisk_EnsureSemaphore(void)
{
  if (sd_dma_sem == NULL)
  {
    const osSemaphoreAttr_t attr = {
      .name = "sdDmaSem"
    };

    sd_dma_sem = osSemaphoreNew(1U, 0U, &attr);
  }
}

static void SdDisk_ClearSemaphore(void)
{
  SdDisk_EnsureSemaphore();
  if (sd_dma_sem != NULL)
  {
    while (osSemaphoreAcquire(sd_dma_sem, 0U) == osOK)
    {
    }
  }
  /* 每次启动DMA前重置结果，避免复用上一笔传输的完成状态。 */
  sd_dma_transfer_state = SD_DMA_TRANSFER_PENDING;
}

void SdDisk_Deinitialize(void)
{
  /* 文件系统卸载后失效底层状态，保证重新插卡时执行完整初始化。 */
  sd_status = STA_NOINIT;

  if (sd_dma_transfer_state == SD_DMA_TRANSFER_PENDING)
  {
    (void)HAL_SD_Abort(&hsd1);
  }
  if (hsd1.State != HAL_SD_STATE_RESET)
  {
    (void)HAL_SD_DeInit(&hsd1);
  }

  sd_dma_transfer_state = SD_DMA_TRANSFER_IDLE;
  if (sd_dma_sem != NULL)
  {
    while (osSemaphoreAcquire(sd_dma_sem, 0U) == osOK)
    {
    }
  }
}

static DRESULT SdDisk_WaitReady(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      return RES_ERROR;
    }
    osDelay(1U);
  }

  return RES_OK;
}

static HAL_StatusTypeDef SdDisk_WaitDma(void)
{
  DRESULT ready_result;

  SdDisk_EnsureSemaphore();
  if (sd_dma_sem == NULL)
  {
    return HAL_ERROR;
  }

  if (osSemaphoreAcquire(sd_dma_sem, SD_DISK_TIMEOUT_MS) != osOK)
  {
    return HAL_TIMEOUT;
  }

  if (sd_dma_transfer_state != SD_DMA_TRANSFER_COMPLETE)
  {
    return HAL_ERROR;
  }

  ready_result = SdDisk_WaitReady(SD_DISK_TIMEOUT_MS);
  return (ready_result == RES_OK) ? HAL_OK : HAL_TIMEOUT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
  HAL_StatusTypeDef hal_status;

  sd_disk_init_count++;
  sd_disk_last_state_before = hsd1.State;
  sd_disk_last_card_state = 0xFFFFFFFFU;

  if (pdrv != SD_DISK_PDRV)
  {
    sd_disk_last_result = RES_PARERR;
    return STA_NOINIT;
  }

  SdDisk_EnsureSemaphore();

  if (hsd1.State == HAL_SD_STATE_READY)
  {
    const HAL_SD_CardStateTypeDef card_state = HAL_SD_GetCardState(&hsd1);
    sd_disk_last_card_state = (uint32_t)card_state;
    if (card_state == HAL_SD_CARD_TRANSFER)
    {
      sd_status = 0U;
      sd_disk_last_state_after = hsd1.State;
      (void)SdDisk_SetResult(RES_OK, HAL_OK);
      return sd_status;
    }
    (void)HAL_SD_DeInit(&hsd1);
  }

  hal_status = HAL_SD_Init(&hsd1);
  sd_disk_last_state_after = hsd1.State;
  sd_disk_last_hal_status = (uint32_t)hal_status;
  sd_disk_last_hal_error = HAL_SD_GetError(&hsd1);

  if (hal_status == HAL_OK)
  {
    sd_status = 0U;
    sd_disk_last_result = RES_OK;
  }
  else
  {
    sd_status = STA_NOINIT;
    sd_disk_last_result = RES_NOTRDY;
  }

  return sd_status;
}

DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv != SD_DISK_PDRV)
  {
    return STA_NOINIT;
  }

  return sd_status;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef wait_status;

  if ((pdrv != SD_DISK_PDRV) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((sd_status & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  /* FatFs传入对齐缓存时直接使用多块DMA，避免逐扇区启动DMA的额外开销。 */
  if ((((uintptr_t)buff) & 0x03U) == 0U)
  {
    SdDisk_ClearSemaphore();
    status = HAL_SD_ReadBlocks_DMA(&hsd1, buff, (uint32_t)sector, count);
    if (status != HAL_OK)
    {
      return SdDisk_SetError(status);
    }
    wait_status = SdDisk_WaitDma();
    if (wait_status != HAL_OK)
    {
      return SdDisk_SetError(wait_status);
    }
    return SdDisk_SetResult(RES_OK, HAL_OK);
  }

  for (UINT index = 0U; index < count; index++)
  {
    BYTE *read_buffer = &buff[index * SD_DISK_BLOCK_SIZE];

    if ((((uintptr_t)read_buffer) & 0x03U) != 0U)
    {
      read_buffer = sd_scratch;
    }

    SdDisk_ClearSemaphore();
    status = HAL_SD_ReadBlocks_DMA(&hsd1,
                                   read_buffer,
                                   (uint32_t)sector + index,
                                   1U);
    if (status != HAL_OK)
    {
      return SdDisk_SetError(status);
    }
    wait_status = SdDisk_WaitDma();
    if (wait_status != HAL_OK)
    {
      return SdDisk_SetError(wait_status);
    }

    if (read_buffer == sd_scratch)
    {
      memcpy(&buff[index * SD_DISK_BLOCK_SIZE], sd_scratch, SD_DISK_BLOCK_SIZE);
    }
  }

  return SdDisk_SetResult(RES_OK, HAL_OK);
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
  HAL_StatusTypeDef status;
  HAL_StatusTypeDef wait_status;

  if ((pdrv != SD_DISK_PDRV) || (buff == NULL) || (count == 0U))
  {
    return RES_PARERR;
  }

  if ((sd_status & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  /* 日志批缓存按4字节对齐，可一次提交连续扇区给SDMMC。 */
  if ((((uintptr_t)buff) & 0x03U) == 0U)
  {
    SdDisk_ClearSemaphore();
    status = HAL_SD_WriteBlocks_DMA(&hsd1,
                                    (uint8_t *)buff,
                                    (uint32_t)sector,
                                    count);
    if (status != HAL_OK)
    {
      return SdDisk_SetError(status);
    }
    wait_status = SdDisk_WaitDma();
    if (wait_status != HAL_OK)
    {
      return SdDisk_SetError(wait_status);
    }
    return SdDisk_SetResult(RES_OK, HAL_OK);
  }

  for (UINT index = 0U; index < count; index++)
  {
    const BYTE *write_buffer = &buff[index * SD_DISK_BLOCK_SIZE];

    if ((((uintptr_t)write_buffer) & 0x03U) != 0U)
    {
      memcpy(sd_scratch, write_buffer, SD_DISK_BLOCK_SIZE);
      write_buffer = sd_scratch;
    }

    SdDisk_ClearSemaphore();
    status = HAL_SD_WriteBlocks_DMA(&hsd1,
                                    write_buffer,
                                    (uint32_t)sector + index,
                                    1U);
    if (status != HAL_OK)
    {
      return SdDisk_SetError(status);
    }
    wait_status = SdDisk_WaitDma();
    if (wait_status != HAL_OK)
    {
      return SdDisk_SetError(wait_status);
    }
  }

  return SdDisk_SetResult(RES_OK, HAL_OK);
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  HAL_SD_CardInfoTypeDef card_info;

  if (pdrv != SD_DISK_PDRV)
  {
    return RES_PARERR;
  }

  if ((sd_status & STA_NOINIT) != 0U)
  {
    return RES_NOTRDY;
  }

  switch (cmd)
  {
    case CTRL_SYNC:
      return SdDisk_WaitReady(SD_DISK_TIMEOUT_MS);

    case GET_SECTOR_COUNT:
      if (buff == NULL)
      {
        return RES_PARERR;
      }
      if (HAL_SD_GetCardInfo(&hsd1, &card_info) != HAL_OK)
      {
        return RES_ERROR;
      }
      *(LBA_t *)buff = (LBA_t)card_info.LogBlockNbr;
      return RES_OK;

    case GET_SECTOR_SIZE:
      if (buff == NULL)
      {
        return RES_PARERR;
      }
      *(WORD *)buff = SD_DISK_BLOCK_SIZE;
      return RES_OK;

    case GET_BLOCK_SIZE:
      if (buff == NULL)
      {
        return RES_PARERR;
      }
      *(DWORD *)buff = 1U;
      return RES_OK;

    default:
      return RES_PARERR;
  }
}

static uint8_t SdDisk_IsLeapYear(uint32_t year)
{
  return (((year % 4U) == 0U) &&
          (((year % 100U) != 0U) || ((year % 400U) == 0U))) ? 1U : 0U;
}

static uint8_t SdDisk_DaysInMonth(uint32_t year, uint8_t month)
{
  static const uint8_t days[12] = {
    31U, 28U, 31U, 30U, 31U, 30U,
    31U, 31U, 30U, 31U, 30U, 31U
  };

  if ((month == 2U) && (SdDisk_IsLeapYear(year) != 0U))
  {
    return 29U;
  }
  return days[month - 1U];
}

static DWORD SdDisk_UtcToFatTime(uint64_t utc_timestamp_us)
{
  uint64_t seconds = utc_timestamp_us / 1000000ULL;
  uint32_t days = (uint32_t)(seconds / 86400ULL);
  uint32_t seconds_of_day = (uint32_t)(seconds % 86400ULL);
  uint32_t year = 1970U;
  uint8_t month = 1U;
  uint8_t day;
  uint8_t month_days;

  while (year < 2108U)
  {
    const uint16_t year_days = (SdDisk_IsLeapYear(year) != 0U) ? 366U : 365U;
    if (days < year_days)
    {
      break;
    }
    days -= year_days;
    year++;
  }
  if ((year < 1980U) || (year > 2107U))
  {
    return 0U;
  }

  while (month <= 12U)
  {
    month_days = SdDisk_DaysInMonth(year, month);
    if (days < month_days)
    {
      break;
    }
    days -= month_days;
    month++;
  }
  day = (uint8_t)(days + 1U);

  return ((DWORD)(year - 1980U) << 25) |
         ((DWORD)month << 21) |
         ((DWORD)day << 16) |
         ((DWORD)(seconds_of_day / 3600U) << 11) |
         ((DWORD)((seconds_of_day % 3600U) / 60U) << 5) |
         ((DWORD)((seconds_of_day % 60U) / 2U));
}

DWORD get_fattime(void)
{
  static DWORD last_valid_time = ((DWORD)(2026U - 1980U) << 25) |
                                 ((DWORD)1U << 21) |
                                 ((DWORD)1U << 16);
  DWORD current_time;

  if (ModbusTimeSync_IsSynced() != 0U)
  {
    current_time = SdDisk_UtcToFatTime(ModbusTimeSync_GetUtcTimestampUs());
    if (current_time != 0U)
    {
      last_valid_time = current_time;
    }
  }

  /* FatFs没有时区字段，统一写UTC；开始录制前强制要求时间已同步。 */
  return last_valid_time;
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  if ((hsd != NULL) && (hsd->Instance == SDMMC1))
  {
    if (sd_dma_transfer_state == SD_DMA_TRANSFER_PENDING)
    {
      sd_dma_transfer_state = SD_DMA_TRANSFER_COMPLETE;
    }
    if (sd_dma_sem != NULL)
    {
      (void)osSemaphoreRelease(sd_dma_sem);
    }
  }
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  if ((hsd != NULL) && (hsd->Instance == SDMMC1))
  {
    if (sd_dma_transfer_state == SD_DMA_TRANSFER_PENDING)
    {
      sd_dma_transfer_state = SD_DMA_TRANSFER_COMPLETE;
    }
    if (sd_dma_sem != NULL)
    {
      (void)osSemaphoreRelease(sd_dma_sem);
    }
  }
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  if ((hsd != NULL) && (hsd->Instance == SDMMC1))
  {
    /* 错误状态先于信号量发布，等待任务被唤醒后可以区分成功和失败。 */
    sd_dma_transfer_state = SD_DMA_TRANSFER_ERROR;
    if (sd_dma_sem != NULL)
    {
      (void)osSemaphoreRelease(sd_dma_sem);
    }
  }
}
