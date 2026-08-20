#ifndef APP_VERSION_H
#define APP_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 固件版本采用 V主版本.次版本.修订版本 格式。
 * 发布新固件时只需在此处修改，串口日志和通信接口会同步更新。
 */
#define GLOVE_FW_VERSION_MAJOR             2
#define GLOVE_FW_VERSION_MINOR             1
#define GLOVE_FW_VERSION_PATCH             0

#define GLOVE_FW_VERSION_STRINGIFY_INNER(value)  #value
#define GLOVE_FW_VERSION_STRINGIFY(value)        GLOVE_FW_VERSION_STRINGIFY_INNER(value)

#define GLOVE_FW_VERSION_STRING            \
    "V" GLOVE_FW_VERSION_STRINGIFY(GLOVE_FW_VERSION_MAJOR) "." \
    GLOVE_FW_VERSION_STRINGIFY(GLOVE_FW_VERSION_MINOR) "." \
    GLOVE_FW_VERSION_STRINGIFY(GLOVE_FW_VERSION_PATCH)

#ifdef __cplusplus
}
#endif

#endif /* APP_VERSION_H */
