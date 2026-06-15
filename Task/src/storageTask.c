#include "storageTask.h"

#include "cmsis_os2.h"
#include "data_manager.h"

#define STORAGE_PLACEHOLDER_GET_TIMEOUT_MS (100U)

void StorageTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    GloveFullFrameBlock_t *full = NULL;
    if (DataManager_GetFullFrame(DATA_CONSUMER_STORAGE,
                                 &full,
                                 STORAGE_PLACEHOLDER_GET_TIMEOUT_MS) == GLOVE_STATUS_OK)
    {
      (void)DataManager_ReleaseFullFrame(full);
    }
  }
}
