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
#define GLOVE_FW_VERSION_MINOR             2
#define GLOVE_FW_VERSION_PATCH             2

/*
    版本说明
    V2.2.2：
        1. 修复SD DMA错误中断可能被误判为传输完成的问题
        2. 485录制命令改为异步执行，文件就绪后才开启Storage数据投递
        3. 移除固定64MiB预分配，避免启动阻塞和异常断电空白尾部
        4. SD日志解析器增加帧号连续性、重复帧和时间戳倒退检查

    V2.2.1：
        1. Storage队列扩展到128帧并增加FullFrame内存池余量
        2. 增加SD日志批量写入和写入耗时诊断

    V2.2.0：
        1. 增加485控制SD卡开始和停止录制
        2. SD日志升级为V2格式，补充诊断字段并优化批量写入
        3. 文件时间改为上位机同步后的实际UTC时间

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
