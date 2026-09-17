# 数据采集手套嵌入式软件任务设计

当前硬件由外部电源直接供电。历史硬件资料归档在 [旧版硬件说明](docs/legacy/POWER_MANAGEMENT_USER_GUIDE.md)，不适用于当前硬件和Modbus 2.1协议。

RS485寄存器、统一健康状态和历史错误清除命令见 [Modbus485协议](MODBUS485_PROTOCOL.md)。

固件版本统一定义在 `App/inc/app_version.h`，当前版本为 `V3.1.1`。

当前发布配置：125Hz PPS同步采样、标称8ms间隔，RS485采用4000000 bit/s、8N1；Modbus/Health协议版本为`0x0201`，传感器快照Schema为`3`。首次对时及PPS失步恢复后需写入对应UTC整秒，连续正常PPS期间无需每秒写UTC。

本文档说明当前 FreeRTOS 工程中的任务划分、任务之间的数据流关系，以及推荐的任务优先级设置。

## 文档更新日志

- `2026-09-17 / V3.1.0 → V3.1.1`：统一更新125Hz PPS采样、8ms标称间隔和112.5Hz诊断阈值；UTC改为首次及失步恢复后一次性对时，修复第0帧时间戳、逐帧UTC有效性、组帧匹配及PPS超时边沿竞态；发布通信配置统一为4Mbps，同步CubeMX、监控工具和SD速率说明。Modbus/Health版本及快照Schema不变，协议文档修订由2.1升至2.2。

## 1. 总体任务列表

当前系统建议包含以下任务：

```text
SystemHealthTask       系统健康监测任务
TimeSyncTask           时间同步任务
ImuCanTask             IMU 数据获取任务
TouchAdcTask           触觉数据获取任务
FrameAssemblerTask     合帧任务
AttitudeTask           姿态计算任务
Rs485Task              485 通讯任务
StorageTask            SD 卡存储任务
TestTask               测试任务，调试阶段使用
```

其中 `TestTask` 只用于早期验证数据管理器和任务流程，正式运行时可以关闭或降低优先级。

---

## 2. 总体数据流

推荐的数据流如下：

```text
TimeSyncTask / 硬件定时器
    -> 提供统一 timestamp

ImuCanTask
    -> GloveImuSensorData_t
    -> DataManager_PublishImuSensor

TouchAdcTask
    -> GloveTouchSensorData_t
    -> DataManager_PublishTouchSensor

FrameAssemblerTask
    -> DataManager_GetImuSensor
    -> DataManager_GetTouchSensor
    -> GloveRawFrame_t
    -> DataManager_PublishRawFrame

AttitudeTask
    -> DataManager_GetRawFrame
    -> 姿态解算和关节角计算
    -> GloveFullFrame_t
    -> DataManager_PublishFullFrame

Rs485Task
    -> DataManager_GetFullFrame(DATA_CONSUMER_RS485)
    -> 协议打包和发送

StorageTask
    -> DataManager_GetFullFrame(DATA_CONSUMER_STORAGE)
    -> 写入 SD 卡缓存和文件

SystemHealthTask
    -> 汇总错误状态 看门狗和运行健康信息
```

---

## 3. 任务职责说明

### 3.1 SystemHealthTask

职责：

```text
错误码维护
任务健康检查
看门狗状态汇总
DataManager 统计信息监控
系统运行状态上报
```

周期：

```text
10 ms
```

该任务不应该执行耗时数据处理，也不应该阻塞采集任务。

---

### 3.2 TimeSyncTask

职责：

```text
维护系统统一时间基准
处理外部时间同步命令
校准本地时间戳
为采集任务提供统一 timestamp
```

建议提供统一接口：

```c
uint32_t AppTime_GetUs(void);
uint32_t AppTime_GetMs(void);
```
---

### 3.3 ImuCanTask

```text
通过 CAN 获取 16 路 IMU 数据
整理六轴数据和四元数
填充 GloveImuSensorData_t
调用 DataManager_PublishImuSensor
```

---

### 3.4 TouchAdcTask

```text
触发 ADC 或 DMA 采集
读取 68 点触觉阵列数据
填充 GloveTouchSensorData_t
调用 DataManager_PublishTouchSensor
```

---

### 3.5 FrameAssemblerTask

```text
获取 ImuSensorData
获取 TouchSensorData
根据 timestamp 或 seq 判断是否可以合帧
生成 GloveRawFrame_t
调用 DataManager_PublishRawFrame
释放 ImuSensorData 和 TouchSensorData
```

后续需要在这里处理：

```text
IMU 和触觉采样频率不一致
时间戳差值过大
某一路传感器超时
丢帧统计
使用最近一帧还是等待新帧
```

---

### 3.6 AttitudeTask

```text
获取 GloveRawFrame_t
进行 IMU 姿态解算
计算 21 自由度关节角 deg
生成 GloveProcessedFrame_t
合成 GloveFullFrame_t
调用 DataManager_PublishFullFrame
释放 RawFrame
```

```text
算法任务可以占用较多 CPU
但不能阻塞 IMU 和触觉采集
算法耗时需要持续统计
```
---

### 3.7 Rs485Task

职责：

```text
获取 FullFrame
打包原始数据和算法结果
计算 CRC
控制 RS485 方向引脚
通过 UART DMA 或非阻塞方式发送
发送完成后释放 FullFrame
```
---

### 3.8 StorageTask

```text
获取 FullFrame
写入 RAM 缓存
缓存到达阈值后写入 SD 卡
维护文件状态
处理写入错误
释放 FullFrame
```
---

### 3.9 TestTask

正式系统中可以关闭该任务，或者设置为最低优先级。

---

## 4. 推荐优先级设置


| 任务 | 推荐优先级 | 原因 |
|---|---:|---|
| TimeSyncTask | `osPriorityHigh` | 时间同步影响所有数据时间戳 |
| ImuCanTask | `osPriorityAboveNormal` | IMU 数据实时性最高 |
| TouchAdcTask | `osPriorityAboveNormal` | 触觉采集需要及时完成 |
| FrameAssemblerTask | `osPriorityNormal` | 需要及时合成 RawFrame |
| AttitudeTask | `osPriorityNormal` | 算法耗时较大 不能压制采集 |
| Rs485Task | `osPriorityBelowNormal` | 通讯重要但可由队列缓存削峰 |
| StorageTask | `osPriorityLow` | SD 写入抖动大 不应影响采集 |
| SystemHealthTask | `osPriorityLow` | 低频健康监测任务 |
| TestTask | `osPriorityLow` | 仅调试阶段使用 |

---

## 5. 数据采集调试输出

当前量产配置已关闭全部调试输出。需要检查触觉原始值时，可将 `APP_BUILD_PRODUCTION` 设为 `0`，并将 `APP_ENABLE_DEBUG_UART_OUTPUT`、`APP_ENABLE_UART_DEBUG_TASK` 和 `APP_ENABLE_TOUCH_1_30_STREAM` 设为 `1`。调试串口为 `USART2`（`PD5/TX`、`PD6/RX`）、`921600 baud`、`8N1`，每个触觉采样周期输出第1～30点（数组下标0～29）：

```text
[TOUCH_01_30] value1,value2,...,value30
```

目标输出频率约为 `125 Hz`，标称采样间隔为`8 ms`；每个PPS采样窗口包含第0～124点，末点脉冲结束后停表等待下一PPS。采样率由`App/inc/app_config.h`中的`GLOVE_SENSOR_SAMPLE_RATE_HZ`配置。30个数依次对应第1～30点。当前全部68个触觉点均采用3帧滑动均值滤波，串口输出与发布给后续任务的数据一致。启动、IMU、完整帧、健康状态及其他触觉点输出均已暂时关闭。

如需恢复完整采集诊断，将 `APP_ENABLE_TOUCH_1_30_STREAM` 设为 `0`，将 `APP_ENABLE_ACQUISITION_DEBUG` 设为 `1`。完整诊断输出格式如下：

```text
[ACQ] status=OK ...
[HEALTH] state=... current_err=... source=... target=...
[RATE] imu_pub=... touch_pub=... raw=... full=... processed=...
[IMU_SAMPLE] node=... acc_mg=(...) gyro_mdps=(...) quat_1e4=(...)
[IMU01] ... 至 [IMU16] ...
[FULL] ...
```

`[ACQ] status=OK` 表示 IMU、触觉、合帧和算法输出均达到至少 `112.5 Hz`（标称采样率的90%），16 个 IMU 在本周期内都有新数据，且没有新增丢帧、超时或关键健康告警。正常情况下重点检查：

- `fresh=0xffff`，表示 16 个 IMU 均有新数据；
- `new_err=0`、`health=0x00000000`、`uart_drop=0`；
- `[RATE]` 各级速率稳定在目标 `125 Hz` 附近；
- `[IMU_SAMPLE]` 的加速度、角速度和四元数会随手套运动变化；
- `[IMU01]` 至 `[IMU16]` 可逐路检查 IMU 数据。

若出现 `status=WARN`，结合 `imu_ok`、`touch_ok`、`pipe_ok`、`fresh`、`new_err` 和 `health` 判断异常所在。`uart_drop` 非零表示调试信息来不及发送，不等同于传感器丢帧。

联调完成后，在 `App/inc/app_config.h` 中恢复量产配置：将 `APP_BUILD_PRODUCTION` 设为 `1`，并将所有 `APP_ENABLE_*DEBUG*` 和 `APP_ENABLE_TOUCH_1_30_STREAM` 调试开关设为 `0`。
