#ifndef SD_DISKIO_H
#define SD_DISKIO_H

#ifdef __cplusplus
extern "C" {
#endif

/* 结束一次介质会话，使下一次挂载重新初始化SD卡。 */
void SdDisk_Deinitialize(void);

#ifdef __cplusplus
}
#endif

#endif /* SD_DISKIO_H */
