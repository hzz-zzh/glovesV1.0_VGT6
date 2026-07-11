# Glove Modbus485 Register Map

## Time Encoding Update

All 4-register time fields now use ROS Time layout instead of a single
`uint64` microsecond counter.

Register order is little-endian by 16-bit Modbus word:

| offset | field |
|---:|---|
| `+0` | `sec` bit15..0 |
| `+1` | `sec` bit31..16 |
| `+2` | `nanosec` bit15..0 |
| `+3` | `nanosec` bit31..16 |

`sec` is the number of seconds since `1970-01-01 00:00:00 UTC`.
`nanosec` is the nanosecond offset within that second and must be
`0..999999999`. When the master writes `0x000A..0x000D`, invalid
`nanosec >= 1000000000` is rejected with Modbus exception `0x03`.

本文档描述当前固件开放给主站的 Modbus RTU 寄存器。寄存器地址均为 16 位 Holding Register 地址。

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

### 3.2 命令状态区

当前命令写入尚未开放，主站可读回状态占位。

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0020` | 1 | U16 | command，占位，当前读 `0` |
| `0x0021` | 1 | U16 | command param，占位，当前读 `0` |
| `0x0022` | 1 | U16 | command seq，占位，当前读 `0` |
| `0x0023` | 1 | U16 | command ack，当前读 `0=idle` |
| `0x0024` | 1 | U16 | command ack seq，占位，当前读 `0` |
| `0x0025` | 1 | U16 | command error，占位，当前读 `0` |
| `0x0026..0x003E` | 25 | U16 | reserved，当前读 `0` |

### 3.3 系统状态区

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0040` | 1 | U16 | system_state，当前 `1=ready` |
| `0x0041` | 1 | U16 | work_mode，当前 `0=normal` |
| `0x0042` | 1 | U16 | log_state，当前 `0=idle` |
| `0x0043` | 1 | U16 | sd_state，当前 `0=not ready` |
| `0x0044` | 1 | U16 | sensor_state，IMU 有效位，bit0 对应 IMU0 |
| `0x0045` | 1 | U16 | comm_state，当前 `1=ok` |
| `0x0046..0x0047` | 2 | U16 | reserved，当前读 `0` |
| `0x0048` | 2 | float32 | board temperature，当前占位 `25.0` |

### 3.4 电源状态区

电源状态区由 `SystemManagerTask` 的实时快照提供。float32 继续采用本协议统一的双寄存器字节序。

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0060` | 2 | float32 | battery voltage，单位 V |
| `0x0062` | 2 | float32 | battery current，单位 A，充电为正、放电为负 |
| `0x0064` | 2 | float32 | battery SOC，单位 % |
| `0x0066` | 1 | U16 | system power state |
| `0x0067` | 1 | U16 | charger state |
| `0x0068` | 1 | U16 | power status flags |
| `0x0069` | 1 | U16 | charger/local fault code |
| `0x006A` | 2 | float32 | VBUS voltage，单位 V |
| `0x006C` | 2 | float32 | input current，单位 A |
| `0x006E` | 1 | U16 | BQ25622 diagnostic：高8位为诊断阶段，低8位为驱动状态 |

system power state：`0=INIT`、`1=ON_NORMAL`、`2=ON_LOW`、`3=USER_OFF`、`4=LOW_BAT_LOCKOUT`。

charger state：`0=UNKNOWN`、`1=NO_INPUT`、`2=IDLE`、`3=CC`、`4=CV`、`5=TOPOFF`、`6=FULL`、`7=SUSPENDED`、`8=FAULT`。

`0x0068` 状态位定义：

| 位 | 含义 |
|---:|---|
| bit0 | battery voltage valid |
| bit1 | SOC valid |
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

读取数值前应检查对应有效位。`0x0069` 低8位为BQ25622原始故障状态，高位中 `bit8/bit9/bit10` 分别表示BQ通信、MAX17043通信和电压不一致。

`0x006E` 用于定位BQ25622通信或配置失败的位置。诊断阶段定义：`0=none`、`1=init`、`2=watchdog`、`3=input_current`、`4=external_ilim`、`5=charge_voltage`、`6=charge_current`、`7=termination_current`、`8=charge_safety`、`9=adc`、`10=status_read`。驱动状态定义：`0=OK`、`1=ERROR`、`2=TIMEOUT`、`3=NO_MEMORY`、`4=INVALID_PARAM`、`5=QUEUE_FULL`、`6=QUEUE_EMPTY`、`7=NOT_READY`。诊断阶段为0且状态为0表示BQ配置和最近一次状态读取正常。

### 3.5 工作状态

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0500` | 1 | U16 | work_state，当前 `0=idle` |

### 3.6 IMU 原始数据区

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

说明：`0x1140`、`0x1340`、`0x2080` 三个数据区时间戳均表示同一帧数据的同步采集时刻 UTC，因此三个值应保持一致。关节区时间戳不是解算完成时刻，而是该帧输入数据的采集时刻。UTC 未同步时读数为 `0.000000000`。

### 3.7 关节解算数据区

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

### 3.8 触觉矩阵数据区

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

### 4.1 时间同步

| 起始地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x000A` | 4 | ROS Time | 主站写入当前 UTC 时间 |

写入后，可通过 `0x0002..0x0005` 读取当前 UTC 时间，通过 `0x000A..0x000D` 读取最近一次同步值。

### 4.2 IMU 校准四元数表

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

大块区域需要分包读取：

| 数据 | 地址范围 | 寄存器数 | 建议 |
|---|---:|---:|---|
| 基础状态 | `0x0000..0x000D` | 14 | 一次读 |
| 系统状态 | `0x0040..0x0049` | 10 | 一次读 |
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
