# Glove Modbus485 协议

本文档描述当前固件开放给主站的Modbus RTU寄存器和自定义功能码。寄存器地址均为16位Holding Register地址。

- 文档修订：`1.3`
- Health协议版本：`0x0102`（主版本1，次版本2）
- 固件版本：`V1.0.0`
- 新增：`0x000E..0x0010` 固件版本只读寄存器
- 兼容性：标准`0x03/0x06/0x10`和自定义`0x41`帧格式保持不变

## 1. 通讯基础

- 物理层：RS485
- 协议：Modbus RTU
- 从站地址：默认 `0x01`
- 串口格式：`8N1`
- 当前源码波特率：`3000000`
- 支持功能码：
  - `0x03`：Read Holding Registers
  - `0x06`：Write Single Register
  - `0x10`：Write Multiple Registers
- 单次读寄存器数量：`1..125`
- `0x10` 单次写寄存器数量：`1..123`
- `0x06` 单次写寄存器数量：固定 `1`

## 2. 数据格式

### U16

单个 Modbus register，寄存器内按 Modbus 标准高字节在前。

### U32 / U64

分别占2个或4个寄存器，低16位寄存器在前；每个寄存器内部仍按Modbus标准高字节在前。

### ROS Time

占 4 个寄存器，低 16 位寄存器在前：

| offset | 含义 |
|---:|---|
| `+0` | sec bit15..0 |
| `+1` | sec bit31..16 |
| `+2` | nanosec bit15..0 |
| `+3` | nanosec bit31..16 |

`nanosec` 必须小于 `1000000000`。

### float32

占 2 个寄存器，低 16 位寄存器在前：

| offset | 含义 |
|---:|---|
| `+0` | float bit15..0 |
| `+1` | float bit31..16 |

示例：`1.0f = 0x3F800000`，寄存器顺序为 `0x0000, 0x3F80`。

## 3. 可读寄存器

### 3.1 基础状态区

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0000` | 1 | U16 | 从站地址，默认 `1` |
| `0x0001` | 1 | U16 | 波特率代码，当前返回 `0` |
| `0x0002` | 4 | ROS Time | 当前 UTC 时间 |
| `0x0006` | 4 | ROS Time | 本地运行时间，按 sec/nanosec 表示 |
| `0x000A` | 4 | ROS Time | 最近一次主站同步的 UTC 时间 |

#### 3.1.1 固件版本信息区

固件版本采用 `V主版本.次版本.修订版本` 格式。三个寄存器均为只读，
当前版本为 `V1.0.0`。

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x000E` | 1 | U16 | 固件主版本，当前为 `1` |
| `0x000F` | 1 | U16 | 固件次版本，当前为 `0` |
| `0x0010` | 1 | U16 | 固件修订版本，当前为 `0` |

### 3.2 命令状态区

`0x0020..0x0022`可写，`0x0023..0x0025`只读。命令采用应用层ACK，Modbus写应答成功只表示寄存器写入成功，主站仍需读取ACK区判断命令结果。

| 地址 | 数量 | 访问 | 含义 |
|---:|---:|---|---|
| `0x0020` | 1 | R/W | command；执行后自动清零 |
| `0x0021` | 1 | R/W | command param；执行后自动清零 |
| `0x0022` | 1 | R/W | command seq；保留最近写入值 |
| `0x0023` | 1 | R | command ack |
| `0x0024` | 1 | R | command ack seq |
| `0x0025` | 1 | R | command error detail |
| `0x0026..0x003E` | 25 | U16 | reserved，当前读 `0` |

ACK定义：`0x0000=idle`、`0x0001=ok`、`0x0002=busy`、`0x8001=unknown command`、`0x8002=invalid param`、`0x8003=state denied`、`0x8004=failed`。

error detail定义：`0x0000=none`、`0x0001=invalid param`、`0x0002=duplicate seq`、`0x0003=state denied`、`0x0004=resource not ready`、`0x0005=start failed`、`0x0006=stop failed`、`0x0007=timeout`。相同`seq`的重发不会重复执行命令，而是保留上一笔ACK和error结果。当前历史清除命令只会产生`none`或`invalid param`。

### 3.3 系统状态区

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0040` | 1 | U16 | system_state，与Health overall state一致 |
| `0x0041` | 1 | U16 | work_mode，当前`0=normal` |
| `0x0042` | 1 | U16 | log_state：`0=idle`、`1=recording` |
| `0x0043` | 1 | U16 | sd_state：`0=not ready`、`1=ready` |
| `0x0044` | 1 | U16 | sensor_state，IMU 有效位，bit0 对应 IMU0 |
| `0x0045` | 1 | U16 | comm_state：`1=ok`、`2=degraded` |
| `0x0046` | 1 | U16 | MCU复位原因位图 |
| `0x0047` | 1 | U16 | MCU独立看门狗状态位图 |
| `0x0048` | 2 | float32 | board temperature，当前占位 `25.0` |

`0x0046`：bit0=引脚复位、bit1=上电或欠压复位、bit2=软件复位、bit3=IWDG复位、bit4=WWDG复位、bit5=低功耗复位。多个原因可能同时存在。

`0x0047`：bit0=IWDG已经启动、bit1=已经执行硬件刷新、bit2=IWDG配置回读警告。正常稳定运行时通常为 `0x0003`；若为 `0x0007`，表示预分频或重装值回读不一致，但任务仍会每100毫秒刷新硬件狗。

### 3.4 统一健康状态区

范围：`0x004A..0x005F`，共22个寄存器。32位量均为低16位寄存器在前。

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x004A` | 1 | U16 | Health协议版本，当前`0x0102` |
| `0x004B` | 1 | U16 | overall state |
| `0x004C` | 2 | U32 | active flags，低字在前 |
| `0x004E` | 1 | U16 | current error |
| `0x004F` | 1 | U16 | current source |
| `0x0050` | 1 | U16 | current target |
| `0x0051` | 1 | U16 | recovery stage |
| `0x0052` | 1 | U16 | 高8位=attempt limit，低8位=attempt |
| `0x0053` | 1 | U16 | last error |
| `0x0054` | 1 | U16 | last source |
| `0x0055` | 1 | U16 | last target |
| `0x0056` | 2 | U32 | error sequence，低字在前 |
| `0x0058` | 2 | U32 | error count，低字在前 |
| `0x005A` | 2 | U32 | last error uptime，单位ms，低字在前 |
| `0x005C` | 1 | U16 | live IMU mask，bit0对应IMU0 |
| `0x005D` | 1 | U16 | sensor ready flags |
| `0x005E` | 1 | U16 | FullFrame age，单位ms；`0xFFFF`表示不可用 |
| `0x005F` | 1 | U16 | 最近一次HAL UART错误位 |

overall state：`0=INIT`、`1=OK`、`2=WARNING`、`3=DEGRADED`、`4=RECOVERING`、`5=FAULT`、`6=OFF`、`7=LOCKOUT`。

source：`0=none`、`1=IMU`、`2=CAN1`、`3=CAN2`、`4=touch`、`5=pipeline`、`6=power`、`7=battery`、`8=charger`、`9=watchdog`、`10=RS485`、`11=time sync`、`12=calibration`、`13=storage`。

recovery stage：`0=none`、`1=loss confirm`、`2=node config`、`3=node verify`、`4=bus reinit`、`5=bus config`、`6=bus verify`、`7=safe stop`、`8=power-off hold`、`9=power start`、`10=sensor wait`、`11=frame verify`、`12=failed`。

sensor ready flags：bit0=全部IMU、bit1=触觉、bit2=FullFrame、bit3=关节、bit4=电源、bit5=时间同步、bit6=RS485。

active flags：

| 位 | 含义 | 位 | 含义 |
|---:|---|---:|---|
| 0 | IMU部分失效 | 14 | BQ通信异常 |
| 1 | 全部IMU失效 | 15 | 电量计通信异常 |
| 2 | 触觉无效 | 16 | 电压读数不一致 |
| 3 | FullFrame过期 | 17 | 温度限制 |
| 4 | 关节无效 | 18 | 充电故障 |
| 5 | CAN1 error-passive | 19 | 看门狗警告 |
| 6 | CAN1 bus-off | 20 | 时间未同步/失步 |
| 7 | CAN2 error-passive | 21 | 校准错误 |
| 8 | CAN2 bus-off | 22 | RS485接收覆盖 |
| 9 | IMU配置失败 | 23 | RS485 UART错误 |
| 10 | CAN重初始化失败 | 24 | RS485发送失败 |
| 11 | 外设电源恢复失败 | 25 | 队列压力 |
| 12 | 低电量 | 26 | 内存池耗尽 |
| 13 | 严重低电量 | 27 | SD错误 |

错误码：

| 范围/值 | 含义 |
|---:|---|
| `0x1001` | IMU节点数据过期，target为逻辑节点 |
| `0x1002` | IMU节点配置失败，target为逻辑节点 |
| `0x2001` | CAN error-passive，target为总线号 |
| `0x2002` | CAN bus-off，target为总线号 |
| `0x2003` | CAN重新初始化失败 |
| `0x2004` | CAN恢复验证失败 |
| `0x3001` | 触觉同步超时 |
| `0x3002` | 触觉ADC DMA超时 |
| `0x3003` | 触觉ADC DMA错误 |
| `0x4001` | FullFrame过期 |
| `0x4002` | IMU/触觉时间戳不匹配 |
| `0x4003` | 数据队列满 |
| `0x4004` | 数据池耗尽 |
| `0x4005` | 关节算法输入无效 |
| `0x5001` | 采集暂停超时 |
| `0x5002` | 同步启动失败 |
| `0x5003` | 外设恢复超时 |
| `0x6001` | 电池低电量 |
| `0x6002` | 电池严重低电量 |
| `0x6003` | BQ25622通信失败 |
| `0x6004` | MAX17043通信失败 |
| `0x6005` | 两路电压读数不一致 |
| `0x6006` | 充电温度限制 |
| `0x6007` | 充电故障 |
| `0x7001` | 看门狗配置警告 |
| `0x8001` | RS485接收帧覆盖 |
| `0x8002` | RS485 UART错误，target为HAL UART错误位 |
| `0x8003` | RS485发送失败 |
| `0x8004` | 时间同步丢失 |
| `0x9001` | 校准被拒绝，target为条目编号 |
| `0xA001` | SD错误，target为SD错误码 |

`0x005F` UART错误位：bit0=PE、bit1=NE、bit2=FE、bit3=ORE、bit4=DMA、bit5=RTO。

`current_*`和active flags表示当前仍存在的问题，经过验证恢复后自动清除；`last_*`、sequence、count和last uptime为历史信息，直到主站执行历史清除命令或设备复位。

### 3.5 电源状态区

电源状态区由 `SystemManagerTask` 的实时快照提供。float32 继续采用本协议统一的双寄存器字节序。

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0060` | 2 | float32 | battery voltage，单位 V |
| `0x0062` | 2 | float32 | battery current，单位 A，充电为正、放电为负 |
| `0x0064` | 2 | float32 | battery SOC，单位 %，当前为未校准估算值 |
| `0x0066` | 1 | U16 | system power state |
| `0x0067` | 1 | U16 | charger state |
| `0x0068` | 1 | U16 | power status flags |
| `0x0069` | 1 | U16 | charger/local fault code |
| `0x006A` | 2 | float32 | VBUS voltage，单位 V |
| `0x006C` | 2 | float32 | input current，单位 A |
| `0x006E` | 1 | U16 | BQ25622 diagnostic：高8位为诊断阶段，低8位为驱动状态 |
| `0x006F` | 1 | U16 | 最近一次BQ充电事件：低8位为Charger Flag 0，高8位为Charger Flag 1 |
| `0x0070` | 1 | U16 | 最近一次BQ故障事件：低8位为Fault Flag 0，高8位保留 |
| `0x0071` | 1 | U16 | BQ INT下降沿累计次数低16位，溢出后回绕 |

system power state：`0=INIT`、`1=ON_NORMAL`、`2=ON_LOW`、`3=USER_OFF`、`4=LOW_BAT_LOCKOUT`、`5=STOPPING`、`6=RECOVERING`、`7=RECOVERY_FAULT`。

charger state：`0=UNKNOWN`、`1=NO_INPUT`、`2=IDLE`、`3=CC`、`4=CV`、`5=TOPOFF`、`6=FULL`、`7=SUSPENDED`、`8=FAULT`。

`0x0068` 状态位定义：

| 位 | 含义 |
|---:|---|
| bit0 | battery voltage valid |
| bit1 | SOC register read valid；仅表示读取成功，不表示估算已校准 |
| bit2 | battery/input current valid |
| bit3 | qualified VBUS present |
| bit4 | charging active |
| bit5 | low battery |
| bit6 | critical battery |
| bit7 | low-battery lockout |
| bit8 | peripheral power enabled |
| bit9 | BQ25622 communication fault |
| bit10 | MAX17043 communication fault |
| bit11 | two voltage readings differ by more than 100mV |
| bit12 | charging limited or suspended by temperature state |
| bit13 | charger fault |
| bit14 | charge safety timer expired |
| bit15 | hardware charge termination confirmed，充电状态为FULL |

读取数值前应检查对应有效位。MAX17043尚未加载本电芯的定制模型，因此SOC只显示，不参与低电状态和严重低电保护；相关保护仅依据有效电压。`0x0069` 低8位为BQ25622原始故障状态，高位中 `bit8/bit9/bit10` 分别表示BQ通信、MAX17043通信和电压不一致。

`0x006E` 用于定位BQ25622通信或配置失败的位置。诊断阶段定义：`0=none`、`1=init`、`2=watchdog`、`3=input_current`、`4=external_ilim`、`5=charge_voltage`、`6=charge_current`、`7=termination_current`、`8=charge_safety`、`9=adc`、`10=status_read`、`11=interrupt_config`、`12=interrupt_read`。驱动状态定义：`0=OK`、`1=ERROR`、`2=TIMEOUT`、`3=NO_MEMORY`、`4=INVALID_PARAM`、`5=QUEUE_FULL`、`6=QUEUE_EMPTY`、`7=NOT_READY`。诊断阶段为0且状态为0表示BQ配置和最近一次状态读取正常。

`0x006F` 的低字节对应BQ25622 `Charger_Flag_0`：bit0=watchdog、bit1=safety timer、bit2=VINDPM、bit3=IINDPM、bit4=VSYS、bit5=thermal regulation、bit6=ADC done。高字节对应 `Charger_Flag_1`：高字节bit0=VBUS changed，高字节bit3=charge status changed。换算到整个U16后分别是bit8和bit11。

`0x0070` 低字节对应 `Fault_Flag_0`：bit0=TS changed、bit3=thermal shutdown、bit4=OTG fault、bit5=system fault、bit6=battery fault、bit7=VBUS fault。BQ硬件Flag为读清零，固件会先保存最近一次INT原因，再读取当前Status；因此这些寄存器表示最近一次成功读取的中断事件，而不是当前持续故障，当前故障仍以 `0x0069` 为准。

### 3.6 SD状态区

范围：`0x0081..0x00BF`。状态来自`SdLog_GetStatus()`实时快照，不是固定占位值。

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0081` | 1 | U16 | 文件系统状态：`0=未挂载`、`1=已挂载` |
| `0x0082` | 1 | U16 | 记录状态：`0=idle`、`1=recording` |
| `0x0083` | 1 | U16 | SD错误码，`0=无错误` |
| `0x0084` | 2 | U32 | 总容量MB，低字在前 |
| `0x0086` | 2 | U32 | 剩余容量MB，低字在前 |
| `0x0088` | 2 | U32 | 已用容量MB，低字在前 |
| `0x008A` | 1 | U16 | 当前文件ID |
| `0x008C` | 4 | U64 | 当前文件大小，低字在前 |
| `0x0090` | 2 | U32 | 当前写入次数，低字在前 |
| `0x0092` | 1 | U16 | 创建文件控制/状态 |
| `0x009A` | 4 | U64 | 日志长度，低字在前 |
| `0x00A0` | 16 | text | 当前文件名，每个寄存器高字节字符在前 |
| `0x00B0` | 16 | text | 最近文件名，每个寄存器高字节字符在前 |

### 3.7 工作状态

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0500` | 1 | U16 | work_state：`0=idle`、`1=acquiring`、`2=stopping`、`0x8000=error` |

### 3.8 IMU 原始数据区

范围：`0x1000..0x113F`，共 16 个 IMU，每个 IMU 20 个寄存器，即 10 个 float32。

第 `i` 个 IMU 的基地址：

```text
imu_base = 0x1000 + i * 20, i = 0..15
```

| offset | 数量 | 类型 | 含义 | 单位 |
|---:|---:|---|---|---|
| `+0` | 2 | float32 | acc_x | m/s^2 |
| `+2` | 2 | float32 | acc_y | m/s^2 |
| `+4` | 2 | float32 | acc_z | m/s^2 |
| `+6` | 2 | float32 | gyro_x | rad/s |
| `+8` | 2 | float32 | gyro_y | rad/s |
| `+10` | 2 | float32 | gyro_z | rad/s |
| `+12` | 2 | float32 | quat_w | - |
| `+14` | 2 | float32 | quat_x | - |
| `+16` | 2 | float32 | quat_y | - |
| `+18` | 2 | float32 | quat_z | - |

说明：`0x1000..0x113F` 始终返回 IMU 采集后的 raw 数据，校准表不会改写这里的加速度、角速度或四元数。校准只影响后续关节角解算结果。

IMU 状态：

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x1140` | 4 | ROS Time | IMU 数据时间戳 |
| `0x1144` | 1 | U16 | IMU 有效位，bit0 对应 IMU0，bit15 对应 IMU15 |

单个IMU的加速度、陀螺仪或四元数任一项超过100ms未更新时，`0x1144`对应有效位清零，并且该IMU在 `0x1000..0x113F` 中对应的10个float32全部读取为 `0.0`，不会继续返回历史值。恢复收到完整数据后自动恢复有效位和实时数值。

说明：`0x1140`、`0x1340`、`0x2080` 三个数据区时间戳均表示同一帧数据的同步采集时刻 UTC，因此三个值应保持一致。关节区时间戳不是解算完成时刻，而是该帧输入数据的采集时刻。UTC 未同步时读数为 `0.000000000`。

### 3.9 关节解算数据区

范围：`0x1300..0x1335`，共 27 个 float32。

第 `j` 个输出的地址：

```text
joint_addr = 0x1300 + j * 2, j = 0..26
```

| index | 类型 | 含义 |
|---:|---|---|
| `0..15` | float32 | 食指/中指/无名指/小指，每指 4 个输出：MCP flex, MCP swing, PIP flex, DIP flex，单位 deg |
| `16..18` | float32 | 拇指 MCP flex, MCP swing, IP flex，单位 deg |
| `19..22` | float32 | 拇指 CMC quaternion：w, x, y, z |
| `23..26` | float32 | 手掌绝对 quaternion：w, x, y, z |

关节状态：

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x1340` | 4 | ROS Time | 关节数据时间戳 |
| `0x1344` | 1 | U16 | status_flags，bit0=snapshot_valid，bit1=algorithm_valid，bit2=joint_calib_applied |
| `0x1345` | 1 | U16 | joint_valid_bits low，bit0 对应 joint0 |
| `0x1346` | 1 | U16 | joint_valid_bits high |

### 3.10 触觉矩阵数据区

范围：`0x2000..0x207F`，容量 128 个 U16；当前有效点数为 68。

第 `k` 个触觉点的地址：

```text
touch_addr = 0x2000 + k, k = 0..67
```

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x2000..0x2043` | 68 | U16 | touch[0..67] 原始采样值 |
| `0x2044..0x207F` | 60 | U16 | reserved，当前读 `0` |
| `0x2080` | 4 | ROS Time | 触觉数据时间戳 |
| `0x2084` | 1 | U16 | status_flags，bit0=snapshot_valid，bit1=touch_valid |
| `0x2085` | 1 | U16 | 当前有效触觉点数，固定 `68` |
| `0x2086` | 1 | U16 | 触觉区容量，固定 `128` |
| `0x2087` | 1 | U16 | reserved，当前读 `0` |

触觉 index 简要说明：

- `0..19`：5 个手指，每指 4 点，`index = finger * 4 + point`
- `20..67`：掌心 6 列 x 8 行，`index = 20 + palm_col * 8 + palm_row`

## 4. 可写寄存器

当前开放 `0x10 Write Multiple Registers` 和 `0x06 Write Single Register` 写入以下区域。`0x10` 用于批量写表和推荐的 apply/reset；`0x06` 用于兼容通用 Modbus 工具的单寄存器写。

### 4.1 清除历史错误

命令值：`0x004A`；保护魔数：`0xC1EA`。

推荐使用FC10原子写入：

```text
Write Multiple Registers
start = 0x0020
count = 3
data  = 0x004A, 0xC1EA, seq
```

随后读取：

```text
Read Holding Registers
start = 0x0023
count = 3
```

成功条件：`ACK=0x0001`、`ACK_SEQ=seq`、`ERROR=0x0000`。

也支持FC06，但必须按参数、序号、命令的顺序写，最后一笔写命令才会触发执行：

```text
Write Single Register 0x0021 = 0xC1EA
Write Single Register 0x0022 = seq
Write Single Register 0x0020 = 0x004A
```

清除范围：`last error/source/target`、error sequence、error count、last error uptime和最近UART错误细节。不会清除active flags、current error/source/target、恢复阶段、IMU在线掩码或Ready flags；当前故障仍存在时继续对外显示。命令不会写入Flash，设备复位同样会清空历史。

主站每次发起新的命令必须更换`seq`；相同`seq`被视为重发，不会重复执行。

### 4.2 时间同步

| 起始地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x000A` | 4 | ROS Time | 主站写入当前 UTC 时间 |

写入后，可通过 `0x0002..0x0005` 读取当前 UTC 时间，通过 `0x000A..0x000D` 读取最近一次同步值。

### 4.3 IMU 校准四元数表

算法使用形式：

```text
calibrated_quat[imu] = C[imu] * raw_quat[imu] * M[imu]
```

校准区范围：`0x1154..0x1293`。第一版只运行时生效，不做掉电保存。

#### C 表

范围：`0x1154..0x11D3`，16 个 IMU，每个 IMU 一个四元数，占 8 个寄存器。

```text
c_addr = 0x1154 + i * 8, i = 0..15
```

#### M 表

范围：`0x11D4..0x1253`，16 个 IMU，每个 IMU 一个四元数，占 8 个寄存器。

```text
m_addr = 0x11D4 + i * 8, i = 0..15
```

#### 单个四元数寄存器顺序

| offset | 类型 | 含义 |
|---:|---|---|
| `+0` | U16 | w low word |
| `+1` | U16 | w high word |
| `+2` | U16 | x low word |
| `+3` | U16 | x high word |
| `+4` | U16 | y low word |
| `+5` | U16 | y high word |
| `+6` | U16 | z low word |
| `+7` | U16 | z high word |

写表时只进入 staging 缓冲区，不立即影响解算。写完整表后需要执行 apply。

使用 `0x06 Write Single Register` 写校准控制区时，主站应先写 `magic` 和 `seq`，最后写 `command`；写入 `command` 的那一帧会触发 apply/reset。

#### 校准控制区

| 地址 | 类型 | 读写 | 含义 |
|---:|---|---|---|
| `0x1254` | U16 | R/W | magic，执行命令前写 `0xCA1B` |
| `0x1255` | U16 | R/W | command：`1=apply`，`2=reset_identity` |
| `0x1256` | U16 | R/W | seq，主站递增序号 |
| `0x1257` | U16 | R | status |
| `0x1258` | U16 | R | error_index |
| `0x1259` | U16 | R | last_applied_seq |
| `0x125A..0x1293` | U16 | R | reserved，当前读 `0` |

status 定义：

| 值 | 含义 |
|---:|---|
| `0x0000` | idle |
| `0x0001` | applied |
| `0x0002` | reset_done |
| `0x8001` | bad_magic |
| `0x8002` | bad_cmd |
| `0x8003` | bad_quat |

error_index 定义：

- `0..15`：C 表对应 IMU 校验失败
- `16..31`：M 表对应 IMU 校验失败
- `0xFFFF`：无错误

执行 apply：

```text
Write Multiple Registers
start = 0x1254
count = 3
data  = 0xCA1B, 0x0001, seq
```

执行 reset_identity：

```text
Write Multiple Registers
start = 0x1254
count = 3
data  = 0xCA1B, 0x0002, seq
```

`0x06` 执行 apply 示例：

```text
Write Single Register 0x1254 = 0xCA1B
Write Single Register 0x1256 = seq
Write Single Register 0x1255 = 0x0001
```

`0x06` 执行 reset_identity 示例：

```text
Write Single Register 0x1254 = 0xCA1B
Write Single Register 0x1256 = seq
Write Single Register 0x1255 = 0x0002
```

校验规则：每个四元数分量必须为 finite float，且范数平方必须在 `0.25..2.25` 内。apply 成功后固件会归一化四元数，并更新算法当前校准表。命令处理完成后，`command` 和 `magic` 会自动清零；主站通过 `status/error_index/last_applied_seq` 判断结果。

## 5. 推荐读取方式

### 5.1 高频传感快照

上位机需要同时读取IMU、关节和触觉时，使用自定义功能码 `0x41`，避免标准
`0x03`受125个寄存器上限影响而产生5次串行往返。标准 `0x03/0x06/0x10`
接口保持不变，普通调试和寄存器访问仍按原方式使用。

请求帧：

```text
[slave][0x41][CRC_L][CRC_H]
```

响应帧：

```text
[slave][0x41][payload_len_H][payload_len_L][payload][CRC_L][CRC_H]
```

当前`payload_len`为904字节，载荷内每个寄存器仍按Modbus大端字节序排列：

| 载荷顺序 | 寄存器数 | 内容 |
|---|---:|---|
| 1 | 2 | FullFrame `frame_id`，低16位在前 |
| 2 | 4 | FullFrame时间戳，ROS Time的sec/nsec各按低16位在前 |
| 3 | 1 | power state |
| 4 | 1 | IMU有效位 |
| 5 | 1 | 关节状态 |
| 6 | 1 | 触觉状态 |
| 7 | 320 | IMU `0x1000..0x113F` |
| 8 | 54 | 关节 `0x1300..0x1335` |
| 9 | 68 | 触觉 `0x2000..0x2043` |

从站在构造响应前只抓取一次FullFrame快照，因此帧号、时间戳和三段传感数据
属于同一帧。完整响应为910字节，未超过当前1024字节RS485发送缓冲区。

当前3 Mbps、8N1链路发送一帧910字节响应约需3.03 ms，加上200 us响应等待、
请求接收、快照构造和收发方向切换后，为5 ms周期的200 Hz连续轮询保留更多余量。
当前上位机工具配合3 Mbps链路采用5 ms周期的200 Hz轮询；手套内部传感器和
FullFrame同样保持200 Hz，每次RS485响应返回当时最新的一帧。
上位机必须等待完整响应后再发送下一条请求，不允许并发或流水发送请求。

`tools/modbus485_monitor.py`中的“200Hz Sensors”模式采用以下策略：

- 单次`0x41`事务容错时间为8 ms，正常收到完整910字节后立即返回，不固定等待8 ms；
- 串口一次等待完整响应，避免短时间分片造成多次Windows/USB串口驱动往返；
- 高频事务不做周期内重试，超时后清空不完整响应并进入下一笔事务；
- 使用5 ms时基调度；事务超过5 ms时记录overrun，并立即从当前时刻重建节拍；
- 健康状态降为1 Hz读取，避免频繁的标准寄存器访问干扰高频快照；
- 界面分别显示通信响应频率、`frame_id`计算出的真实传感帧率、重复响应数和调度超期数。

上位机统计传感器更新率时，以约1秒窗口内的 `frame_id` 增量除以上位机单调
时钟的实际窗口长度。该统计不依赖UTC是否完成同步；连续响应中的 `frame_id`
相同表示本次通信读到了尚未更新的同一传感器帧。

### 5.2 标准寄存器读取

大块区域需要分包读取：

| 数据 | 地址范围 | 寄存器数 | 建议 |
|---|---:|---:|---|
| 基础状态 | `0x0000..0x000D` | 14 | 一次读 |
| 固件版本 | `0x000E..0x0010` | 3 | 一次读 |
| 命令ACK | `0x0023..0x0025` | 3 | 执行命令后读取 |
| 系统状态 | `0x0040..0x0049` | 10 | 一次读 |
| 统一健康状态 | `0x004A..0x005F` | 22 | 一次读 |
| 电源状态 | `0x0060..0x0071` | 18 | 一次读 |
| SD状态 | `0x0081..0x00BF` | 63 | 一次读 |
| IMU 数据 | `0x1000..0x113F` | 320 | 分包读，例如 100 + 100 + 100 + 20 |
| IMU 状态 | `0x1140..0x1144` | 5 | 一次读 |
| 关节数据 | `0x1300..0x1335` | 54 | 一次读 |
| 关节状态 | `0x1340..0x1346` | 7 | 一次读 |
| 触觉有效数据 | `0x2000..0x2043` | 68 | 一次读 |
| 触觉状态 | `0x2080..0x2087` | 8 | 一次读 |
| C 校准表 | `0x1154..0x11D3` | 128 | 分包读写 |
| M 校准表 | `0x11D4..0x1253` | 128 | 分包读写 |

## 6. 异常说明

- 访问未开放地址：返回 Modbus exception `0x02`，Illegal Data Address
- 读写数量非法：返回 Modbus exception `0x03`，Illegal Data Value
- 写入非开放区域：返回 Modbus exception `0x02`
