# Glove Modbus485 协议

- 文档修订：`2.0`
- Modbus协议版本：`0x0200`（主版本2，次版本0）
- Health协议版本：`0x0200`（主版本2，次版本0）
- 传感器快照Schema：`2`
- 固件版本：`V3.0.0`

## 1. 通讯基础

- 物理层：RS485
- 协议：Modbus RTU
- 从站地址：默认 `0x01`
- 串口格式：`8N1`
- 默认波特率：`6000000 bit/s`
- 支持功能码：
  - `0x03`：Read Holding Registers
  - `0x06`：Write Single Register
  - `0x10`：Write Multiple Registers
  - `0x41`：Read Sensor Snapshot，自定义只读功能码
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
当前版本为 `V3.0.0`。

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x000E` | 1 | U16 | 固件主版本，当前为 `3` |
| `0x000F` | 1 | U16 | 固件次版本，当前为 `0` |
| `0x0010` | 1 | U16 | 固件修订版本，当前为 `0` |

#### 3.1.2 协议和设备信息区

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0011` | 1 | U16 | Modbus协议版本，当前`0x0200` |
| `0x0012` | 1 | U16 | `0x41`传感器快照Schema，当前`2` |
| `0x0013` | 1 | U16 | 设备能力位图 |
| `0x0014` | 1 | U16 | 手套侧别：`0=未知`、`1=左手`、`2=右手` |
| `0x0015` | 1 | U16 | 硬件版本；未烧录板级版本时返回`0` |
| `0x0016..0x001B` | 6 | 3×U32 | 设备96位唯一标识，每个U32低16位寄存器在前 |

设备能力位图：bit0=`0x41`传感器快照、bit1=UTC时间同步、bit2=IMU校准、bit3=SD日志。主站应先检查协议版本和能力位，再启用相应功能。

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

error detail定义：`0x0000=none`、`0x0001=invalid param`、`0x0002=duplicate seq`、`0x0003=state denied`、`0x0004=resource not ready`、`0x0005=start failed`、`0x0006=stop failed`、`0x0007=timeout`。相同`seq`的重发不会重复执行命令，而是保留上一笔ACK和error结果。

### 3.3 系统状态区

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0040` | 1 | U16 | system_state，与Health overall state一致 |
| `0x0041` | 1 | U16 | work_mode，当前`0=normal` |
| `0x0042` | 1 | U16 | log_state：`0=idle`、`1=recording` |
| `0x0043` | 1 | U16 | sd_state：`0=not ready`、`1=ready` |
| `0x0044` | 1 | U16 | sensor_state，IMU 有效位，bit0 对应 IMU0 |
| `0x0045` | 1 | U16 | comm_state：`1=ok`、`2=degraded` |
| `0x0046` | 1 | U16 | 设备复位诊断码；供售后分析，主站应记录原始值 |
| `0x0047` | 1 | U16 | 运行监控诊断码；供售后分析，主站应记录原始值 |
| `0x0048` | 2 | float32 | 保留；当前版本不提供有效板载温度，主站应忽略 |

### 3.4 统一健康状态区

范围：`0x004A..0x005F`，共22个寄存器。32位量均为低16位寄存器在前。

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x004A` | 1 | U16 | Health协议版本，当前`0x0200` |
| `0x004B` | 1 | U16 | overall state |
| `0x004C` | 2 | U32 | active flags，低字在前 |
| `0x004E` | 1 | U16 | current error |
| `0x004F` | 1 | U16 | current source |
| `0x0050` | 1 | U16 | current target |
| `0x0051` | 1 | U16 | 恢复状态诊断码 |
| `0x0052` | 1 | U16 | 恢复次数诊断值 |
| `0x0053` | 1 | U16 | last error |
| `0x0054` | 1 | U16 | last source |
| `0x0055` | 1 | U16 | last target |
| `0x0056` | 2 | U32 | error sequence，低字在前 |
| `0x0058` | 2 | U32 | error count，低字在前 |
| `0x005A` | 2 | U32 | last error uptime，单位ms，低字在前 |
| `0x005C` | 1 | U16 | live IMU mask，bit0对应IMU0 |
| `0x005D` | 1 | U16 | sensor ready flags |
| `0x005E` | 1 | U16 | 传感器快照数据龄期，单位ms；`0xFFFF`表示不可用 |
| `0x005F` | 1 | U16 | 最近一次UART诊断码 |

overall state：`0=INIT`、`1=OK`、`2=WARNING`、`3=DEGRADED`、`4=RECOVERING`、`5=FAULT`。

sensor ready flags：bit0=全部IMU、bit1=触觉、bit2=传感器快照、bit3=关节、bit5=时间同步、bit6=RS485。bit4保留。

`current source`、`current target`、`recovery stage`及对应的历史字段用于售后诊断。外部应用应记录原始数值并随故障信息一并上报，不应根据这些字段自动执行恢复操作。

active flags中面向外部应用的状态如下。表中未列出的位为保留或诊断位，主站不得据此改变控制流程。

| 位 | 含义 |
|---:|---|
| 0 | 部分IMU数据不可用 |
| 1 | 全部IMU数据不可用 |
| 2 | 触觉数据不可用 |
| 3 | 传感器快照过期 |
| 4 | 关节数据不可用 |
| 20 | UTC时间未同步或同步已失效 |
| 21 | 校准数据错误 |
| 22 | RS485接收覆盖 |
| 23 | RS485 UART错误 |
| 24 | RS485发送失败 |
| 27 | SD存储错误 |

错误码按功能域划分。外部应用可显示错误码，并结合`overall state`、`active flags`和各数据区有效位判断数据是否可用；未在下表定义的错误码应作为供应方诊断信息原样保留。

| 范围/值 | 含义 |
|---:|---|
| `0x1xxx` | IMU数据或配置异常 |
| `0x2xxx` | 传感器通信异常 |
| `0x3xxx` | 触觉采集异常 |
| `0x4xxx` | 数据同步或处理异常 |
| `0x5xxx` | 采集控制异常 |
| `0x7xxx` | 系统监控告警 |
| `0x8001` | RS485接收帧覆盖 |
| `0x8002` | RS485 UART错误 |
| `0x8003` | RS485发送失败 |
| `0x8004` | 时间同步丢失 |
| `0x9001` | 校准被拒绝，target为条目编号 |
| `0xA001` | SD错误，target为SD错误码 |

`current_*`和active flags表示当前仍存在的问题，经过验证恢复后自动清除；`last_*`、sequence、count和last uptime为历史信息，直到主站执行历史清除命令或设备复位。

### 3.5 SD状态区

范围：`0x0081..0x00BF`。该区域返回设备当前的SD日志状态。

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0081` | 1 | U16 | 文件系统状态：`0=未挂载`、`1=已挂载`；停止录制完成后返回未挂载 |
| `0x0082` | 1 | U16 | 记录状态：`0=idle`、`1=recording`、`2=stopping`、`3=preparing`、`0x8000=error` |
| `0x0083` | 1 | U16 | SD错误码；`0x8001=非exFAT`、`0x8002=写入不完整`、`0x8003=文件号用尽`、`0x8004=UTC未同步`，其他值为文件系统错误 |
| `0x0084` | 2 | U32 | 总容量MB，低字在前 |
| `0x0086` | 2 | U32 | 剩余容量MB，低字在前 |
| `0x0088` | 2 | U32 | 已用容量MB，低字在前 |
| `0x008A` | 1 | U16 | 当前文件ID |
| `0x008C` | 4 | U64 | 当前文件大小，低字在前 |
| `0x0090` | 2 | U32 | 当前写入次数，低字在前 |
| `0x0092` | 1 | U16 | 日志格式版本，当前为 `2` |
| `0x0093` | 1 | U16 | 单帧记录字节数，当前为 `1536` |
| `0x0094..0x0099` | 6 | 3×U32 | 存储性能诊断值；供售后分析，主站应记录原始值 |
| `0x009A` | 4 | U64 | 日志长度，低字在前 |
| `0x00A0` | 16 | text | 当前文件名，每个寄存器高字节字符在前 |
| `0x00B0` | 16 | text | 最近文件名，每个寄存器高字节字符在前 |

日志V2每帧固定1536字节，结构如下：

| offset | 字节数 | 含义 |
|---:|---:|---|
| `0` | 4 | magic=`GLV2` |
| `4` | 12 | 格式版本、头长度、记录长度、IMU/关节/触觉数量，各为U16小端 |
| `16` | 8 | 左右手、时间同步标志、固件主/次/修订版本 |
| `24` | 72 | frame_id、UTC时间戳、数据有效标志、IMU有效掩码、算法与校准状态、数据源序号和时间戳、Health状态 |
| `96` | 640 | 16路IMU，每路10个float32小端 |
| `736` | 108 | 27个关节角float32小端 |
| `844` | 136 | 68点触觉值U16小端 |
| `980` | 136 | 68点触觉baseline U16小端 |
| `1116` | 417 | 保留并补0 |
| `1533` | 2 | 前1533字节的Modbus CRC16，小端 |
| `1535` | 1 | 分隔字节0 |


创建日志文件前必须完成UTC同步，日志文件时间统一使用UTC。

### 3.6 工作状态

| 地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x0500` | 1 | U16 | work_state：`0=idle`、`1=acquiring`、`0x8000=error` |

### 3.7 IMU 原始数据区

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

说明：`0x1140`、`0x1340`、`0x2080` 三个数据区时间戳均表示同一帧数据的同步采集时刻 UTC，因此三个值应保持一致。关节区时间戳表示输入数据的采集时刻。UTC 未同步时读数为 `0.000000000`。

### 3.8 关节解算数据区

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

### 3.9 触觉矩阵数据区

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

### 4.2 SD录制控制

开始录制命令为`0x0094`，停止录制命令为`0x0096`，两者参数均必须为`0`。
推荐使用FC10原子写入：

```text
start = 0x0020
count = 3
data  = command, 0x0000, seq
```

写入后读取`0x0023..0x0025`，`ACK=0x0001`、`ACK_SEQ=seq`、`ERROR=0x0000`
表示命令已被设备接受。主站随后必须轮询`0x0082..0x0083`：开始录制等待
状态进入`recording`，停止录制等待状态进入`idle`；进入`error`表示实际执行失败。
SD卡应使用exFAT文件系统。开始命令会创建下一个`LOGxxxx.BIN`文件；UTC未同步时
命令返回`resource not ready`。停止命令返回成功ACK后仍需等待状态进入`idle`，此时
日志文件才完成保存。在状态进入`idle`前，不得断电或拔出SD卡。

### 4.3 时间同步

| 起始地址 | 数量 | 类型 | 含义 |
|---:|---:|---|---|
| `0x000A` | 4 | ROS Time | 主站写入当前 UTC 时间 |

写入后，可通过 `0x0002..0x0005` 读取当前 UTC 时间，通过 `0x000A..0x000D` 读取最近一次同步值。

### 4.4 IMU 校准四元数表

算法使用形式：

```text
calibrated_quat[imu] = C[imu] * raw_quat[imu] * M[imu]
```

校准区范围：`0x1154..0x1293`。当前版本仅在运行期间生效，设备掉电或复位后需要重新写入并执行apply。

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

写表过程不会立即影响当前解算结果。完整写入C表和M表后，必须执行apply才会生效。

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
仍可用于普通调试和寄存器访问。当前`0x41`响应使用Schema 2。

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
| 1 | 2 | 快照`frame_id`，低16位在前 |
| 2 | 4 | 快照时间戳，ROS Time的sec/nsec各按低16位在前 |
| 3 | 1 | snapshot status：bit0=快照有效且未过期，bit1=UTC时间有效 |
| 4 | 1 | IMU有效位 |
| 5 | 1 | 关节状态 |
| 6 | 1 | 触觉状态 |
| 7 | 320 | IMU `0x1000..0x113F` |
| 8 | 54 | 关节 `0x1300..0x1335` |
| 9 | 68 | 触觉 `0x2000..0x2043` |

每次响应中的帧号、时间戳和三段传感数据来自同一份一致性快照。完整响应长度为910字节。

主站应先检查snapshot status bit0，再检查IMU、关节和触觉状态。UTC未同步时bit1为0，但传感器数据仍可有效；此时可使用`frame_id`和主站本地时间处理数据。

默认6 Mbps链路下，推荐以5 ms周期轮询，轮询频率不应超过200 Hz。主站必须等待
完整响应后再发送下一条请求，不允许并发或流水发送请求。串口超时时间应为完整响应
留出余量，并根据所用RS485适配器和操作系统调度情况调整。

开始高速轮询前，主站应读取并检查协议版本、快照Schema和设备能力位。高速轮询期间
建议只发送`0x41`；其他寄存器访问应安排在轮询停止后或较低负载时执行。发生超时、CRC
错误或不完整响应时，应丢弃本帧并继续下一周期，避免在同一周期内连续重试造成总线拥塞。

上位机统计传感器更新率时，以约1秒窗口内的 `frame_id` 增量除以上位机单调
时钟的实际窗口长度。该统计不依赖UTC是否完成同步；连续响应中的 `frame_id`
相同表示本次通信读到了尚未更新的同一传感器帧。

### 5.2 标准寄存器读取

大块区域需要分包读取：

| 数据 | 地址范围 | 寄存器数 | 建议 |
|---|---:|---:|---|
| 基础状态 | `0x0000..0x000D` | 14 | 一次读 |
| 固件版本 | `0x000E..0x0010` | 3 | 一次读 |
| 设备信息 | `0x0011..0x001B` | 11 | 一次读 |
| 命令ACK | `0x0023..0x0025` | 3 | 执行命令后读取 |
| 系统状态 | `0x0040..0x0049` | 10 | 一次读 |
| 统一健康状态 | `0x004A..0x005F` | 22 | 一次读 |
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
