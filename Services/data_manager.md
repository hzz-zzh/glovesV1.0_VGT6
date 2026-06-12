# DataManager 数据管理器设计说明

## 1. 设计目标

- 避免 FreeRTOS 队列反复拷贝大结构体
- 保证数据块生命周期清晰可控
- 保证原始数据和算法结果可以正确绑定
- 为后续 SD 存储、RS485 通讯、算法处理提供统一的数据接口

`DataManager` 不负责具体业务逻辑，例如 CAN 采集、ADC 采集、姿态解算、SD 写入、485 打包等。它只负责数据块的分配、发布、接收、释放和统计。

---

## 2. 总体数据流

当前设计的数据流如下：

```text
IMU_CAN_Task
    -> ImuSensorData
    -> DataManager_PublishImuSensor()
    -> FrameAssemblerTask

Touch_ADC_Task
    -> TouchSensorData
    -> DataManager_PublishTouchSensor()
    -> FrameAssemblerTask

FrameAssemblerTask
    -> ImuSensorData + TouchSensorData
    -> RawFrame
    -> DataManager_PublishRawFrame()
    -> AlgorithmTask

AlgorithmTask
    -> RawFrame
    -> ProcessedFrame
    -> FullFrame = RawFrame + ProcessedFrame
    -> DataManager_PublishFullFrame()
    -> StorageTask / RS485Task

StorageTask
    -> DataManager_GetFullFrame(DATA_CONSUMER_STORAGE)
    -> 写入 SD 卡

RS485Task
    -> DataManager_GetFullFrame(DATA_CONSUMER_RS485)
    -> 打包发送
```

也可以简化理解为：

```text
SensorData -> RawFrame -> FullFrame
```

其中：

```text
SensorData  是单类传感器采集结果
RawFrame    是同一帧的完整原始数据
FullFrame   是原始数据 + 算法结果
```

---

## 3. 为什么需要 SensorData

手套系统中 IMU 和触觉数据来自不同任务：

- `IMU_CAN_Task` 负责采集 16 路 IMU 数据
- `Touch_ADC_Task` 负责采集 81 点触觉阵列数据

这两个任务的采样时刻、采样耗时和执行周期可能不同，因此不建议让它们直接写同一个 `RawFrame`。

更稳妥的方式是：

```text
IMU_CAN_Task 只生产 ImuSensorData
Touch_ADC_Task 只生产 TouchSensorData
FrameAssemblerTask 负责时间对齐和合帧
```

这样可以避免：

- 两个任务同时写同一块内存
- RawFrame 半新半旧
- 算法任务读到未完成数据
- 采集超时后内存释放混乱

---

## 4. 数据类型职责

### 4.1 ImuSensorData

`GloveImuSensorData_t` 表示一次 IMU 采集结果。

典型内容包括：

- `sensor_seq`：IMU 采集序号
- `timestamp_us`：IMU 数据时间戳
- `valid_flags`：有效数据标志
- `imu[16]`：16 个 IMU 的六轴数据
- `quat[16]`：16 个 IMU 的四元数

生产者：

```text
IMU_CAN_Task
```

消费者：

```text
FrameAssemblerTask
```

---

### 4.2 TouchSensorData

`GloveTouchSensorData_t` 表示一次触觉阵列采集结果。

典型内容包括：

- `sensor_seq`：触觉采集序号
- `timestamp_us`：触觉数据时间戳
- `valid_flags`：有效数据标志
- `touch[81]`：81 点触觉数据

生产者：

```text
Touch_ADC_Task
```

消费者：

```text
FrameAssemblerTask
```

---

### 4.3 RawFrame

`GloveRawFrame_t` 表示一帧完整原始数据。

它由 `FrameAssemblerTask` 根据 `ImuSensorData` 和 `TouchSensorData` 合成。

典型内容包括：

- `frame_id`：系统统一帧号
- `timestamp_us`：合帧后的统一时间戳
- `valid_flags`：原始数据有效标志
- `imu[16]`
- `quat[16]`
- `touch[81]`

生产者：

```text
FrameAssemblerTask
```

消费者：

```text
AlgorithmTask
```

---

### 4.4 ProcessedFrame

`GloveProcessedFrame_t` 表示算法处理结果。

典型内容包括：

- `frame_id`
- `timestamp_us`
- `imu_attitude[16]`
- `joint_angle_deg[21]`

生产者：

```text
AlgorithmTask
```

注意：`ProcessedFrame` 通常不单独通过 `DataManager` 发布，而是和 `RawFrame` 合成为 `FullFrame`。

---

### 4.5 FullFrame

`GloveFullFrame_t` 表示最终用于存储和通讯的完整数据帧。

它包含：

```text
FullFrame = RawFrame + ProcessedFrame
```

生产者：

```text
AlgorithmTask
```

消费者：

```text
StorageTask
RS485Task
```

这样可以保证 SD 卡和 RS485 拿到的数据一定是同一帧的原始数据和对应算法结果，不会出现原始数据和算法结果错位。

---

## 5. 内存管理方式

`DataManager` 不使用动态堆内存，而是使用固定内存池。

每类数据都有独立内存池：

```text
ImuSensorData   -> imu_sensor_pool
TouchSensorData -> touch_sensor_pool
RawFrame        -> raw_pool
FullFrame       -> full_pool
```

内存池在初始化时一次性创建固定数量的数据块，运行过程中只做分配和归还。

优点：

- 分配时间确定
- 不产生堆碎片
- 内存占用可控
- 适合实时嵌入式系统

---

## 6. 队列传递方式

FreeRTOS/CMSIS-RTOS 队列中只传递指针，不传递完整结构体。

例如：

```c
osMessageQueuePut(queue, &ptr, 0U, timeout);
```

这样可以避免几百字节甚至上千字节的数据帧在队列中反复拷贝。

实际数据仍然保存在内存池的数据块中。

---

## 7. 引用计数设计

每个数据块都带有 `ref_count`。

它表示当前还有多少个持有者正在使用该数据块。

规则：

- `Alloc` 成功后，生产者持有 1 个引用
- `Publish` 成功投递给一个消费者后，消费者获得 1 个引用
- `Publish` 结束后，生产者释放自己的引用
- 消费者 `Get` 成功后，使用完必须调用对应的 `Release`
- 当 `ref_count` 变为 0 时，数据块归还内存池

以 `FullFrame` 为例：

```text
AllocFullFrame 后：
    ref_count = 1    // AlgorithmTask 持有

发布给 Storage 成功：
    ref_count = 2

发布给 RS485 成功：
    ref_count = 3

发布者释放：
    ref_count = 2    // Storage 和 RS485 各持有 1 个引用

Storage 释放：
    ref_count = 1

RS485 释放：
    ref_count = 0    // 归还 full_pool
```

因此，只有最后一个消费者释放后，数据块才会真正回到内存池。

---

## 8. 初始化方式

`DataManager_Init()` 会创建消息队列，因此需要在 `osKernelInitialize()` 之后调用。

推荐位置：

```c
osKernelInitialize();

if (DataManager_Init() != GLOVE_STATUS_OK)
{
    Error_Handler();
}

MX_FREERTOS_Init();
osKernelStart();
```

不要放在 `MX_ICACHE_Init()` 后面直接调用，因为那时 RTOS kernel 还没有初始化，`osMessageQueueNew()` 可能无法安全工作。

---

## 9. 使用方式

### 9.1 IMU 采集任务

```c
GloveImuSensorBlock_t *imu = DataManager_AllocImuSensor();

if (imu != NULL)
{
    imu->data.sensor_seq = seq;
    imu->data.timestamp_us = timestamp_us;
    imu->data.valid_flags = GLOVE_FRAME_FLAG_IMU_VALID |
                            GLOVE_FRAME_FLAG_QUAT_VALID;

    // 填充 imu->data.imu 和 imu->data.quat

    DataManager_PublishImuSensor(imu, 0U);
}
```

`Publish` 返回后，生产者不再拥有 `imu`，不能继续访问。

---

### 9.2 触觉采集任务

```c
GloveTouchSensorBlock_t *touch = DataManager_AllocTouchSensor();

if (touch != NULL)
{
    touch->data.sensor_seq = seq;
    touch->data.timestamp_us = timestamp_us;
    touch->data.valid_flags = GLOVE_FRAME_FLAG_TOUCH_VALID;

    // 填充 touch->data.touch

    DataManager_PublishTouchSensor(touch, 0U);
}
```

---

### 9.3 合帧任务

```c
GloveImuSensorBlock_t *imu = NULL;
GloveTouchSensorBlock_t *touch = NULL;

if (DataManager_GetImuSensor(&imu, 10U) == GLOVE_STATUS_OK &&
    DataManager_GetTouchSensor(&touch, 10U) == GLOVE_STATUS_OK)
{
    GloveRawFrameBlock_t *raw = DataManager_AllocRawFrame();

    if (raw != NULL)
    {
        AppData_BuildRawFrameFromSensors(&raw->frame,
                                         frame_id,
                                         imu->data.timestamp_us,
                                         &imu->data,
                                         &touch->data);

        DataManager_PublishRawFrame(raw, 0U);
    }

    DataManager_ReleaseImuSensor(imu);
    DataManager_ReleaseTouchSensor(touch);
}
```

实际工程中，合帧任务还需要处理时间戳对齐、超时和丢帧策略。

---

### 9.4 算法任务

```c
GloveRawFrameBlock_t *raw = NULL;

if (DataManager_GetRawFrame(DATA_CONSUMER_ALGORITHM, &raw, 10U) == GLOVE_STATUS_OK)
{
    GloveProcessedFrame_t processed;
    AppData_ClearProcessedFrame(&processed);

    processed.frame_id = raw->frame.frame_id;
    processed.timestamp_us = raw->frame.timestamp_us;
    processed.valid_flags = GLOVE_FRAME_FLAG_ALGORITHM_VALID;

    // 姿态解算和关节角计算

    GloveFullFrameBlock_t *full = DataManager_AllocFullFrame();

    if (full != NULL)
    {
        AppData_BuildFullFrame(&full->frame, &raw->frame, &processed);
        DataManager_PublishFullFrame(full, 0U);
    }

    DataManager_ReleaseRawFrame(raw);
}
```

---

### 9.5 SD 存储任务

```c
GloveFullFrameBlock_t *full = NULL;

if (DataManager_GetFullFrame(DATA_CONSUMER_STORAGE, &full, 10U) == GLOVE_STATUS_OK)
{
    // 写 full->frame.raw
    // 写 full->frame.processed

    DataManager_ReleaseFullFrame(full);
}
```

---

### 9.6 RS485 通讯任务

```c
GloveFullFrameBlock_t *full = NULL;

if (DataManager_GetFullFrame(DATA_CONSUMER_RS485, &full, 10U) == GLOVE_STATUS_OK)
{
    // 打包 full->frame.raw
    // 打包 full->frame.processed

    DataManager_ReleaseFullFrame(full);
}
```

---

## 10. 使用规则

必须遵守以下规则：

```text
1. 业务任务不要直接调用 FramePool_Alloc / FramePool_Free
2. 每次 Alloc 成功后，要么 Publish，要么 Release
3. Publish 返回后，生产者不能继续访问该数据块
4. 每次 Get 成功后，消费者必须调用一次对应 Release
5. RawFrame 只给 AlgorithmTask 消费
6. FullFrame 只给 StorageTask 和 RS485Task 消费
7. SD 和 RS485 不直接消费 RawFrame
8. 时间同步和合帧策略放在 FrameAssemblerTask 中实现
```

---

## 11. 调试建议

可以通过 `DataManager_GetStats()` 观察运行状态。

重点关注：

```text
pool_alloc_failures
queue_send_failures
imu_sensor_dropped
touch_sensor_dropped
raw_frames_dropped
full_frames_dropped
```

如果 `pool_alloc_failures` 增加，说明某类内存池容量可能不足，或者某个消费者没有及时释放数据块。

如果 `queue_send_failures` 增加，说明下游任务处理太慢或队列深度不足。

如果 `full_frames_dropped` 增加，说明 SD 或 RS485 消费链路存在阻塞风险。

---

## 12. 后续扩展方向

后续可以在当前设计上继续扩展：

- 为 FrameAssemblerTask 增加时间戳匹配策略
- 为不同传感器增加超时和丢帧统计
- 为 SD 写入增加缓存队列或块缓冲
- 为 RS485 增加协议打包层
- 为 DataManager 增加运行时诊断接口
- 根据实测帧率调整内存池大小和队列深度
