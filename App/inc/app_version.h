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

/*
    版本说明
    V2.1.0： 
        1. 485通讯频率提升至 200Hz（固件支持，且上位机可以实现）
        2. IMU CAN通讯 默认通讯速度为1Mbps
        3. 传感器采样频率提高到 200Hz
        4. 解决一些已知问题

    V1.0.0:
         1. 初始版本 
*/

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
