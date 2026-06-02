# 数据采集手套嵌入式软件任务设计

本文档说明当前 FreeRTOS 工程中的任务划分、任务之间的数据流关系，以及推荐的任务优先级设置。

## 1. 总体任务列表

当前系统建议包含以下任务：

```text
SystemManagerTask      系统管理任务
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

SystemManagerTask
    -> 电量检测 状态灯 错误状态 看门狗 运行统计
```

---

## 3. 任务职责说明

### 3.1 SystemManagerTask

职责：

```text
电量检测
状态灯显示
错误码维护
任务健康检查
看门狗喂狗
DataManager 统计信息监控
系统运行状态上报
```

周期：

```text
300 ms 
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
读取 81 点触觉阵列数据
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
计算 21 自由度关节角
计算关节角速度
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
| SystemManagerTask | `osPriorityLow` | 低频状态管理任务 |
| TestTask | `osPriorityLow` | 仅调试阶段使用 |
