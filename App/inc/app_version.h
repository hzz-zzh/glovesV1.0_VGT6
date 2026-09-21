#ifndef APP_VERSION_H
#define APP_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 固件版本采用 V主版本.次版本.修订版本 格式。
 * 发布新固件时只需在此处修改，串口日志和通信接口会同步更新。
 */
#define GLOVE_FW_VERSION_MAJOR             3
#define GLOVE_FW_VERSION_MINOR             1
#define GLOVE_FW_VERSION_PATCH             2

/*
    版本说明
    V3.1.2（2026-09-17，相对V3.1.1）：
        1. 增加命令启动的125Hz独立调试采集，复用真实IMU、触觉、组帧和算法链路
        2. 正常PPS模式不变；调试屏蔽PPS硬件复位，真实输入仅用于状态监测
        3. 切换等待脉冲下降沿及20ms排空，按采样序号边界丢弃跨模式旧帧
        4. 增加10秒会话及续期接口，停止、超时或复位后恢复正常PPS模式
        5. 调试UTC无效、485绝对时间字段为0，拒绝UTC写入和SD录制
        6. Modbus升级到2.2，增加调试命令、只读状态区和调试位，Schema仍为3
        7. 监控工具增加独立采集按钮和自动续期，协议统一记录一次发布变化

    V3.1.1（2026-09-17，相对V3.1.0）：
        1. PPS同步采样由200Hz改为125Hz，每秒125点、标称间隔8ms，末点脉冲结束后停表
        2. 统一采样率配置、TIM2默认参数、动态周期换算及诊断阈值，同步监控工具和SD速率说明
        3. UTC采用首次及失步恢复后一次性对时，连续正常PPS自动推进整秒并校正本地计时频率
        4. 保留主机主动校时及报文重试，修复重复UTC写入参与频率积分的问题
        5. 第0帧打时间戳前先处理PPS，修复更新/触发中断回调顺序问题
        6. UTC有效性随采样帧传递，485和SD不再误标首次及恢复对时前的旧帧
        7. 组帧先匹配采样序号，避免未对时的零时间戳造成错帧配对
        8. PPS超时检测原子读取时间并在停采前复查，避免边沿抢占造成误判丢失
        9. RS485发布配置统一为4Mbps，同步CubeMX、监控工具和协议；接口布局保持不变

    V3.1.0：
        1. PPS硬件触发每秒200点采样窗口，PPS丢失后停采而通信保持正常
        2. Modbus协议升级到2.1、快照升级到Schema 3，增加PPS/采样状态和帧龄
        3. TIM5改为自由运行时间基准，UTC写入不再清零本地时钟

    V3.0.0：
        1. Modbus协议升级到2.0，移除已退役的0x0060~0x0071状态区
        2. 传感器快照升级到Schema 2，以快照有效和UTC有效标志替代旧元数据
        3. 增加协议版本、能力、左右手配置和MCU唯一标识寄存器
        4. 系统管理任务精简为健康监测任务，移除历史兼容状态

    V2.3.0：
        1. 适配外部直接供电的新硬件，移除电量计和充电管理芯片逻辑
        2. 移除按键控制外设电源及外设断电恢复流程
        3. 触觉数据均值滤波窗口3帧
        4. 默认关闭调试串口、采集诊断和触觉数据流输出

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
