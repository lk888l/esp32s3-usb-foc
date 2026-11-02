# 硬件记录

资料来源：

- `smb://nas.local/home/tmp/ESP32-S3_DRV8316CT_引脚分配.md`
- `smb://nas.local/home/archive/docs/simple-foc/DRV8316CTRGFR.pdf`
- `smb://nas.local/home/archive/docs/simple-foc/v1.0/M创动工坊分享资料2.93.1/说明书/编码器资料/AS5600-ASOM_资料编码器资料.PDF`

用户最初输入的 `DRV316CTRGFR` 按手册文件、接线表与 TI 料号确认为 **DRV8316CTRGFR**。CT 是电阻/引脚配置版本，没有 SPI 配置接口。

| 信号 | ESP32-S3 | 模块接口 |
|---|---|---|
| U / V / W PWM | GPIO15 / 16 / 17 | IN1/H1-9 / IN2/H1-7 / IN3/H1-5 |
| U / V / W 电流 | GPIO4 / 5 / 6 | SOA/H1-8 / SOB/H1-6 / SOC/H1-4 |
| nSLEEP | GPIO18 | NSLP/H1-3，高电平使能 |
| I²C SDA / SCL | GPIO8 / 9 | AS5600 0x36 |
| COM 口 UART0 TX / RX | GPIO43 / 44 | 开发板板载 USB 转 UART 桥，无需外部跳线 |
| USB D− / D+ | GPIO19 / 20 | 原生 USB |
| 地 | GND | H1-1，与编码器和母线共地 |
| 模块 VDD | 未接 | H1-2 |
| 0.1_VCC | 未接 | H1-10；母线电压由手动配置提供 |

电机：2804，7 极对（不是 12 槽）、12 槽、最大电流 1 A。用户实测母线 **11.62 V**。

必须核实模块配置：

- MODE 为普通 3x PWM，SOx 有效；INLx 及 DRVOFF 必须由模块正确拉置。MODE 直接接 AVDD 的 3PWM+限流模式会禁用 SOx，不能用于这里的电流闭环。
- GAIN 的四种设置：AGND=0.15、悬空=0.3、47 kΩ 到 AVDD=0.6、AVDD=1.2 V/A。当前未知，默认最低值作保守换算。
- SOx 的零点是 VREF/2，电流由 OUTx 流向电机时 SOx **升高**（TI Fig.8-30）。换算为 `(SOx - offset) / gain`；已加入极性回归测试。软件会在唤醒后全低侧状态测量三个零点，拒绝异常零点或过大噪声。
- I²C 应有适合 300 kHz 的外部上拉；已启用芯片内部上拉作为辅助。
- nSLEEP 最好有板级下拉，保证上电至 GPIO 初始化之前也保持关闭。
- 本接线没有 nFAULT；只能读取现有信号，不能从 CT 版本读取 SPI 故障寄存器。不能把驱动器自身 8 A/16 A OCP 当作电机 1 A 保护。

## 开发板与 USB 接口记录

用户确认板型为 ESP32-S3-DevKitC-1。切换 COM 口后，已识别到 WCH `1a86:55d3`，序列号 `5B14035954`；esptool 成功读到 ESP32-S3 rev 0.2、MAC `9c:13:9e:aa:f3:a4`、16 MB Flash、8 MB PSRAM。该轮使用 COM 口调参。

先前原生 USB 枚举到 `303a:4001`、序列号 `9c139eaaf3a40000`、`/dev/ttyACM0`。串口权限正常且没有进程占用，但普通复位、不复位握手和手动 BOOT/RESET 后仍没有成功识别芯片，出现 `Write timeout`。仅 USB 描述符不足以确认下载模式或具体芯片，未在识别失败时盲目烧录。

首次烧录、后续调参和复位读回通过板载 COM 口完成。2026-10-30 当前连接已改为原生 USB，枚举 `303a:1001`（USB JTAG/serial debug unit），esptool 成功确认相同 MAC `9c:13:9e:aa:f3:a4`。发现旧固件仍从 UART0 收命令，原生 USB 仅输出日志，因此调参写入超时。本次将默认传输改为原生 USB，关闭应用控制台日志，并已实际烧录、查询参数、运行四种模式及反复重连验证。COM/UART0 仍可在 menuconfig 中选择。

## 当前磁铁和测量条件

AS5600 实测 STATUS=`0x33`，MD=1、ML=1、MH=0。用户确认机械间隙不能缩小，并要求先忽略磁场偏弱，因此默认配置显式启用 `KNOB_ALLOW_WEAK_MAGNET`。状态判断只检查有效位，不把保留位当作故障；缺磁铁和过强磁场仍拒绝驱动。

ADC 两相读取实测约 18–19 μs；控制任务目标周期 500 μs，I²C 与调度偶有延迟，固件会检查采样时间和连续失效次数。GAIN 未经电阻/电压实测确认，遥测中的 A 按 0.15 V/A 计算，不能替代外部电流测量。当前校准确认了引脚对应的相序和编码器方向，仍未完成整个负载与转速范围的仪器验证。

此前恢复调试时枚举的是 WCH COM 桥（`1a86:55d3`），当时的结果仅覆盖 UART0；原生 USB 验证以本次 2026-10-30 记录为准。

转动测试发现的 `0x103` 同时也是 IDF 5.5 I²C 事务失败的返回值，不能单凭该码断言磁铁丢失。现在分别保留传输错误、磁铁状态和重试次数；总线降至 300 kHz，并对预算内的短暂传输失败重试一次。
