# STM32H563 批量代码保护与 CLOSED 回退流程

## 1. 适用范围

- 芯片：STM32H563VGT6
- 工程配置：TrustZone Disabled
- 保护方式：Debug Authentication 密码认证 + `PRODUCT_STATE=CLOSED`
- 当前工具：STM32CubeProgrammer 2.19.0、STM32CubeH5 V1.6.0

> `CLOSED` 可阻止通过 SWD/JTAG 读取和调试程序。  
> 本配置不支持无损重新打开调试；解除 `CLOSED` 必须执行 Full Regression，并擦除芯片内全部程序和数据。

## 2. 固定目录和文件

DA 工作目录：

```text
E:\STM32_Security\H563_Provisioning\ROT_Provisioning\DA
```

批量操作使用同一认证凭据时，必须使用配套生成的以下文件：

```text
DA\Binary\DA_ConfigWithPassword.obk
DA\Binary\password.bin
```

`DA\Config\DA_ConfigWithPassword.xml` 中包含明文密码，也必须按密钥文件管理。

要求：

- 禁止使用 ST 默认密码 `0123456789012345`。
- 三个文件不得提交到 Git、网盘公共目录或发送给无关人员。
- `DA_ConfigWithPassword.obk` 和 `password.bin` 必须作为同一组文件备份，禁止单独重新生成或替换。
- 建议保留两份加密备份，并记录该组凭据对应的产品批次。
- 批量生产期间不需要重复生成 OBK；继续使用已确认有效的同一组文件。

## 3. 硬件连接

安全配置、状态查询和 Full Regression 使用 ST-Link：

```text
ST-Link SWDIO  -> MCU SWDIO
ST-Link SWCLK  -> MCU SWCLK
ST-Link GND    -> 板卡 GND
ST-Link VTref  -> 板卡 3.3V
ST-Link NRST   -> MCU NRST
```

注意：

- NRST 必须接到复位按键的 MCU NRST 一侧，不能接到 GND 一侧。
- 板卡必须稳定供电。
- DAP 与 ST-Link 不得同时连接 SWDIO/SWCLK。
- `provisioning.bat`、`discovery.bat`、`regression.bat` 均使用 ST-Link。
- 建议烧录应用时也使用 ST-Link，避免切换探针。

## 4. 剩余批次设置 CLOSED

### 4.1 操作前检查

1. 确认目标板允许擦除。
2. 确认正式固件已经准备好。
3. 确认 OBK 和密码文件是已验证的配套文件。
4. 连接 ST-Link 和 NRST。
5. 关闭 Keil Debug、DAP 和已连接目标板的 CubeProgrammer。

### 4.2 执行 Provisioning

进入：

```text
E:\STM32_Security\H563_Provisioning\ROT_Provisioning\DA
```

运行：

```text
provisioning.bat
```

按照以下顺序操作：

1. 出现 TrustZone 选择时输入：

   ```text
   n
   ```

2. 脚本提示已生成 `DA_ConfigWithPassword.obk` 时，确认文件存在，然后按任意键。
3. Step 2 会初始化 Option Bytes 并 Mass Erase，等待：

   ```text
   Successful option bytes programming
   ```

4. 到达以下位置后不要立即按键：

   ```text
   Step 3 : Images flashing
   Press any key to continue...
   ```

5. 使用 Keil 下载正式程序：

   ```text
   F:\Gloves\code\glovesV1.0_VGT6\MDK-ARM\glovesV1.0_VGT6.uvprojx
   ```

6. 复位或重新上电，确认应用和主要外设功能正常。
7. 退出 Keil Debug，确认 ST-Link 仍已连接。
8. 回到脚本窗口按任意键。
9. 等待：

   ```text
   Successful obk provisioning
   ```

10. 最终产品状态输入：

    ```text
    CLOSED
    ```

11. 看到以下信息后，本次配置完成：

    ```text
    The board is correctly configured.
    ```

严禁选择：

```text
LOCKED
```

`LOCKED` 不支持密码回退，配置后不可恢复。

### 4.3 如果必须使用 DAP 下载应用

DAP 只能用于 Step 3 的应用下载：

1. 脚本停在 `Images flashing` 后断开 ST-Link 的 SWD 信号。
2. 使用 DAP 下载并验证应用。
3. 完全退出 Keil Debug并拔掉 DAP。
4. 重新连接 ST-Link，包括 NRST。
5. 确认 ST-Link 可用后再回到脚本按键。

忘记切回 ST-Link 会在 `Setting the product state PROVISIONING` 处报错。

## 5. CLOSED 状态验证

1. 板卡完全断电后重新上电，确认应用正常运行。
2. 使用 ST-Link运行：

   ```text
   discovery.bat
   ```

3. 应看到类似结果：

   ```text
   PSA lifecycle: ST_LIFECYCLE_CLOSED
   cryptosystems: ST Password
   ```

4. 使用 Keil普通 Debug 或 CubeProgrammer普通 SWD 读取 Flash 应失败。

`discovery.bat` 只查询状态，不擦除程序。

## 6. 解除 CLOSED：Full Regression

> Full Regression 会擦除用户 Flash、SRAM和 OBK，并将产品状态恢复为 `OPEN`。  
> 原固件和数据无法保留。正式成品无需返修时不要执行。

### 6.1 操作前检查

1. 确认确实允许擦除目标板。
2. 确认使用该板设置 CLOSED 时对应的 `password.bin`。
3. 连接 ST-Link 的 SWDIO、SWCLK、GND、VTref 和 NRST。
4. 断开 DAP，关闭 Keil Debug和 CubeProgrammer连接。

### 6.2 查询 CLOSED 状态

运行：

```text
discovery.bat
```

确认显示：

```text
ST_LIFECYCLE_CLOSED
```

### 6.3 执行回退

运行：

```text
regression.bat
```

脚本使用：

```text
DA\Binary\password.bin
```

成功后结果：

- 用户程序和数据被全部擦除。
- OBK认证配置被擦除。
- 产品状态恢复为 `OPEN`。
- 可重新使用 Keil或 CubeProgrammer烧录。

如果需要再次设置保护，必须重新执行本文第4章完整 Provisioning流程。

## 7. 常见错误

### 7.1 `No device found by ST-Link`

常见原因：

- 未连接 NRST或 NRST接错。
- 板卡未上电或 VTref异常。
- DAP仍连接在 SWD总线上。
- Keil/CubeProgrammer占用了 ST-Link。

处理顺序：

1. 确认板卡供电。
2. 确认 ST-Link NRST接到 MCU NRST网络。
3. 拔掉 DAP。
4. 关闭占用探针的软件。
5. 重新插拔 ST-Link。
6. 先运行 `discovery.bat`，成功后再运行 `regression.bat`。

该错误发生在认证前时不会擦除程序。

### 7.2 `Access port is not valid`

如果发生在 Step 3 之后的：

```text
Setting the product state PROVISIONING
```

通常是使用 DAP下载后忘记切回 ST-Link。

若日志确认 `PRODUCT_STATE=0x17` 尚未写入，可以重新运行 `provisioning.bat`；Step 2 会再次擦除程序，因此必须重新下载应用。

### 7.3 在 PROVISIONING 或 OBK 写入阶段失败

如果日志已经显示：

```text
Setting the product state PROVISIONING
```

执行成功，或者已经显示：

```text
Successful obk provisioning
```

此时不要盲目重新运行完整脚本。保留现场并检查：

```text
provisioning.log
ob_programming.log
obk_provisioning.log
```

根据最后成功步骤继续处理。

## 8. 每块板的生产记录

建议至少记录：

```text
板卡序列号：
芯片UID：
固件版本：
DA凭据批次编号：
Provisioning日期：
操作人员：
CLOSED验证结果：
```

记录中只保存凭据编号，不记录明文密码。
