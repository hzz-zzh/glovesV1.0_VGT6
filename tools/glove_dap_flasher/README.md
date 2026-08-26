# Glove DAP Flasher

这是用于 `STM32H563VGT6` 的 Windows 一键烧录工具，仅支持标准 CMSIS-DAP。固件、Python运行环境、pyOCD和目标芯片 Flash算法都会被打包进可执行文件，目标电脑不需要安装 Keil、Python或 STM32CubeProgrammer。

软件每 1.2 秒自动检测一次 CMSIS-DAP 的 USB 连接状态，支持兼容 pyOCD 的 CMSIS-DAP v1/HID 和 v2/WinUSB 下载器，不限制 ATK 品牌。烧录期间自动暂停检测，避免后台扫描占用下载器。如果同时连接多个 CMSIS-DAP，软件使用扫描到的第一个。

## 接线

```text
CMSIS-DAP SWDIO  -> MCU SWDIO
CMSIS-DAP SWCLK  -> MCU SWCLK
CMSIS-DAP GND    -> 板卡 GND
```

目标板需要单独稳定供电。当前下载器没有连接 `NRST`，程序先以普通 halt 模式连接，再通过 Cortex-M `SYSRESETREQ` 执行软件系统复位并立即停住 CPU，随后启动 Flash算法。如果现有固件导致 SWD完全无法连接，请先给目标板重新上电，再立即点击烧录。

## 运行源码

依赖已安装到项目内的 `.vendor` 后，可以直接执行：

```powershell
python .\app.py
```

## 重新打包

1. 在 Keil中生成最新的 `MDK-ARM\glovesV1.0_VGT6\glovesV1_0_VGT6.hex`。
2. 修改 `resources\firmware.json` 中唯一的 `version` 字段。软件界面、固件信息和 EXE 文件名都会自动使用该版本号。
3. 确认电脑已经安装 `Keil.STM32H5xx_DFP 2.2.0`。
4. 执行：

```powershell
.\build.ps1
```

输出文件：

```text
dist\GloveDAPFlasher_<固件版本>.exe
```

构建脚本会重新复制最新 HEX、计算 SHA-256，并更新内置固件信息。

构建后可执行以下命令进行自检。自检只检查内置资源并枚举 DAP，不连接或改写目标芯片：

```powershell
.\dist\GloveDAPFlasher_<固件版本>.exe --self-test
```

## 烧录保护

- 固件地址必须完全位于 `0x08000000` 至 `0x080FFFFF`。
- 目标 Flash容量必须为 1024 KiB。
- 禁止 pyOCD自动解锁，避免连接异常时意外执行整片擦除。
- 烧录完成后逐段读回，并与 HEX逐字节比较。
- 烧录期间软件禁止退出。
