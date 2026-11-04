# SmartKnob S3

ESP32-S3 / DRV8316CT / MT6701（SSI + DMA）/ 22008（7 极对）的电流闭环力反馈旋钮。当前版本采用 **PWM 同步 ADC GDMA、5 kHz 电流/手感环、独立编码器任务和集中在跨档区域的力矩曲线**。

## 2026-11-04 母线采样与额定电流验证

本轮新增 **GPIO11 / ADC2_CH0 母线电压采样**，用户已接好 H1-10（0.1_VCC）。原理图的 47 kΩ / 4.7 kΩ 分压倍率是 **11**；空载读回约 19.43–19.47 V。ADC2 在独立任务以 100 Hz 采样，PWM 和 PI 限压使用实测母线，采样无效或超过 100 ms 未更新时禁止驱动或锁存停机。配置中的名义母线与实测值分开报告。三相电流仍专用 ADC1 GDMA。

当前候选固件为 `dma-v5-bus-sense` / `20k70_experimental`，已烧录后停机。20 kHz 采用三个相邻 PWM 周期各一次 START、每四周期完成一次控制；两相触发间隔约 50 μs，仍需要带载和动态验证。逐相低电平钳位在允许的角度减少公共偏置，所有非零 PWM 指令脉宽仍至少 4%（2 μs）；扇区边界保留原偏置。静止零目标时停止 PWM 翻转，三相输入保持 LOW。按 19.15 V 母线、7.2 V 指令，数学测试中的全角度最大占空比约 65.12%；这不是电机端电压的外部实测。

本轮结果与限制：

- **5 kHz / 70%、7.2 V、0.8 A 上限**的 30 秒手动保持测试完成，无故障。135 个低速受力 USB 快照中，同向 Iq 平均约 **0.756 A**、目标约 0.756 A、输出指令幅值约 6.80 V，受力母线约 19.16 V。不同槽仍有约 0.07 A 的差异，限流预算发生收减，不能称为稳定达到 0.8 A。
- 用户明确反馈 5 kHz 的**声调和响度都上升**，因此该方案只保留为电压能力对照，降噪未通过。
- 20 kHz 新候选已通过静态和 0.08 A / 0.8 V 限制下的零目标使能检查，109 份快照均有效且无故障，但期间仍有 97 次控制采样被拒绝。该检查没有形成非零电流目标。此前两轮 20 kHz 采集也没有有效手动偏转，不能作为电流跟踪或带载验证；额定电流及蜂鸣改善仍待人工配合检查。
- 当前已保存参数为 **名义母线 19.4 V、电流上限 0.8 A、电压上限 2.35 V、Kp=0.4、Ki=200、mode=3**。本轮低电流校准为 direction=+1、electrical_zero=6.0791654587 rad。7.2 V 测试参数没有保存。上电保持停机。

新增顺序检查确保上一 DMA 字与元数据处理完成后才启动下一次转换；10/5 kHz 的迟到回调可以在实际窗口内继续转换，完整但越窗的样本被拒绝，未完成组则要求重新同步。20 kHz 还验证两次有用采样之间 PWM 比较值一致，禁止混合跨电压更新的样本。USB 状态报文按 512 字节分块发送；保存 NVS 后自动重建采样，保留既有故障且不自动使能。

14 组桌面测试通过，覆盖母线采样开/关、失效与时效、三种 PWM 电压边界、全角度线电压、最小有效脉宽、逐相强制低电平时序和采样顺序。各候选已通过 ESP-IDF 构建。绝对相电流、实际脉冲、温升和最终噪声仍未完成仪器验证。

本轮候选的配置为 `build/rated-current-20261104/bus.sdkconfig`，构建目录为 `build/rated-current-20261104/idf-bus`；常规 Kconfig 默认仍为 10k50。已添加可复现的实验配置 [sdkconfig.defaults.bus20k](sdkconfig.defaults.bus20k)，在导入 ESP-IDF 环境后、使用新的配置路径构建：

```powershell
idf.py -B build/bus20k -D SDKCONFIG=build/bus20k/sdkconfig -D "SDKCONFIG_DEFAULTS=sdkconfig.defaults;sdkconfig.defaults.bus20k" build
```

这份配置要求 H1-10 已接 GPIO11，并启用仍待带载验证的 20 kHz 方案。候选镜像、失败记录和测量数据均保留在 `build/rated-current-20261104/`。详见[硬件记录](docs/hardware.md)和[协议字段](docs/protocol.md)。

## 先前 10k50 实验参数与手感（历史）

以下为 `dma-localized-v4-10k50-voltage-exp3` 采样窗口实验的源码默认值与实现。已有 NVS 不会自动恢复源码默认值，现场参数和实验结果见 [硬件记录](docs/hardware.md)。

| 项目 | 源码默认值 / 实现 |
|---|---|
| 供电 / 感测增益 | 11.62 V / 0.15 V/A；沿用配置，本轮未重新测量 |
| PWM | 10 kHz 中心对齐，三相同步更新（50% 最大占空比、50 μs 公共低侧窗口实验） |
| 电流与手感环 | 5 kHz，200 μs；每 2 个 PWM 周期采样一次 |
| ADC | ADC1 → GDMA；名义 −15 / 0 / +15 μs 触发，低侧窗口内轮换采两相，重建第三相 |
| 编码器 | 默认 MT6701，14 位 SSI，SPI2 2 MHz + DMA；独立 core 0 任务约 2.5 kHz 更新 |
| 档距 / 强度 / 运行电流上限 | 10° / 20 / 0.8 A；强度可配置至 100，电流可配置至 4.5 A |
| 阻尼 / 逻辑切档点 | 0.003 A·s/rad / 0.55 档距 |
| 电流 PI / 电压矢量上限 | Kp=0.4、Ki=200 / 2.35 V |
| 电阻前馈 | 0，默认关闭；测得新电机有效参数后可在 menuconfig 中设置 |
| 无主机运行 | 默认允许；显式 enable 后无需持续 USB 心跳 |

2026-11-04 已换用 **22008、7 极对**电机，用户提供的额定电流为 0.8 A、最大电流为 4.5 A，驱动器仍为 DRV8316CT。此前更换电机时通过 COM7 读回：idle、fault=none，电流上限 **1 A**、电压上限 **0.8 V**、Kp=0.4、Ki=200、强度 10、端点强度 20、档距 10°、阻尼 0.005、mode=3（弹簧）。此前记录中的 0.12 A / 0.8 V / 强度 1.5 是上轮低电流联调保存值，不能代表这次读回状态。

当前已烧录10 kHz / 50%实验固件，现场RAM配置为 **Kp=0.6、Ki=100、2 A、3.0 V、mode=1**；最后保持停机。本轮没有保存NVS，重启后仍恢复下述此前保存值。

此前的 v4 生产固件曾显式保存：**mode=1（档位）、10°、强度 20、电流上限 0.8 A、电压上限 2.35 V、阻尼 0.003、Kp=0.4、Ki=200**。端点强度保留用户原值 20（源码默认值仍为 8）；snap=0.55、行程 0–11。原 MT6701 电角度校准完整保留，direction=+1、electrical_zero=0.2768845558 rad。保存时设备为 idle、fault=none。

v4 将软件电流配置上限开放至 **4.5 A**，默认使用 **0.8 A**，不把最大值当成持续运行额定值。电流单位为控制器的 q 轴目标幅值及相电流反馈，不能用电源输入电流替代。当前 10 kHz / 50% 实验在 11.62 V 配置母线下允许的电压矢量上限约 **3.086 V**，源码默认仍为 2.35 V；实际电流仍取决于绕组、反电动势和感测增益，尚未实测达到 4.5 A。

档位模式每个档距中间 **60% 区域不请求弹力或阻尼**；靠近跨档点时先阻挡、越过后助推，力矩连续。峰值为 `min(current_limit, strength × width / 2)`，并受实测电流、速度包络与电压能力限制。逻辑档位的迟滞不移动实际力矩曲线。有限行程的端点、弹簧和自由模式保留各自语义。

已删除额外的 1000 A/s 电流目标上升斜坡，目标增加、撤销及反向直接进入电流环；实测超限降额和 d 轴电流余量限制继续生效。当前通过 PI 调节电流。此前 4.5 Ω 前馈估计来自旧 2804 电机，不能直接用于 22008，故默认电阻前馈保持 0；在重新确认感测增益及绕组参数后可启用。

静止且原始目标严格为零时，清空电流 PI 并输出零电压，避免空隙中主动纠正 ADC 零点噪声。运动或出现任何非零力矩请求时恢复连续调节；没有按电流幅值硬切断的小电流死区。用于阻尼的速度滤波为 0.5 ms，共同高速包络使用 10 ms 平滑速度。

已删除旧编码器振动调试时加入的 0.15 ms 双级位置平滑、一码偏差限制和双编码器区间挡位速度分支。手感位置直接使用连续机械角度加快速速度预测，仍按编码器时间戳、传感器延迟与 PWM 生效时刻补偿；电流变换使用采样时刻的预测角度。0.5 ms 快速速度滤波仍用于阻尼和预测。

档位的同向助推在 0–10 rad/s 保持全量，10–25 rad/s 才随速度收减，25 rad/s 归零；阻挡分量保留。这样恢复正常手拨速度下的过档助推，同时保留高速释放时的限制。共同高速包络仍为 25–40 rad/s 渐弱、40 rad/s 暂停，低于 5 rad/s 持续 30 ms 后约 500 ms 恢复。关闭主机租约不改变显式 disable 的行为，也不会使设备上电自动驱动。

`dma-localized-v4` 的运行目标最高 4.5 A，实测超限先降额。普通保护阈值为 `min(5, current_limit + max(0.15, current_limit × 0.25))` A，超限累积 20 ms 停机；5 A 及以上累积 1 ms 停机，6 A 及以上立即停机。低于阈值时暴露计时以四分之一速度恢复，避免频繁尖峰反复清零。采样、模拟输入饱和和编码器失效仍停机。完整规则见[协议说明](docs/protocol.md)。

**已有 NVS 配置不会被新的源码默认值覆盖。** 应先 disable，再 configure 并 save 应用新的手感参数；仅烧录应用不会自动移除板上旧的 0.8 V 限制。更换编码器类型时保留配置、使旧编码器的电角度校准失效，必须重新校准并 save。此前 AS5600 的设备调参和验证见 [DMA 与手感联调记录](docs/dma-haptics-2026-11.md)。[以前的 README 与测试记录](docs/history-2026-10.md) 已单独归档。

弹簧与行程端点的回程阻尼取 `max(damping, 0.04)` A·s/rad，防止将低阻尼档位参数用于持续弹性回中而产生余振；档位空隙不受此值影响。

## 工程入口

| 路径 | 职责 |
|---|---|
| `components/knob_core` | FOC 数学、电流调节、角度跟踪、手感；可独立进行桌面测试 |
| `components/knob_hal` | MCPWM、ADC GDMA、统一编码器接口、驱动使能及采样时序 |
| `components/knob_encoder` | 编码器选择与参数、MT6701 SSI DMA 驱动、保留的 AS5600 I²C 驱动 |
| `components/knob_app` | core 1 电机任务、core 0 编码器/USB 任务、协议和 NVS |
| `components/app`、`main/main.cpp` | C++ 模块生命周期和组合根 |
| `tests`、`tools` | 桌面回归及实机调试脚本 |
| [硬件说明](docs/hardware.md)、[协议](docs/protocol.md) | 引脚、测量边界、参数和遥测 |

电机任务独占控制状态；编码器任务独占所选编码器的通信接口，通过带时间戳的短快照交接。电流变换使用采样时刻的预测角度，输出使用 PWM 生效时刻的预测角度。JSON、编码器传输等待和 Flash 写入不进入电流计算路径。业务层使用统一角度、原始磁场状态和故障类型，芯片协议解析、CRC 与资源管理放在编码器模块中。

默认配置中 ADC1 和 MT6701 的 SPI2 接收都使用 DMA。SSI 每帧接收完整 24 位（14 位角度、4 位状态、6 位 CRC），校验失败或磁场异常不会作为有效角度进入控制。AS5600 后端保留原来的 400 kHz I²C 实现；切换回该后端时，编码器通路不使用 DMA。

## 编码器选择与接线

默认直接执行 `idf.py build` 即构建 MT6701，不需要编码器编译参数或 menuconfig 选项。下次更换时修改 `components/knob_encoder/include/encoder_config.hpp` 中的 `kind`，选择 `EncoderKind::mt6701` 或 `EncoderKind::as5600`，再重新构建；引脚及对应传感器延迟、分辨率也集中在该模块配置中。

| MT6701 信号 | ESP32-S3 / 接线 |
|---|---|
| CLK / B / SCL | GPIO12 |
| DO / A / SDA | GPIO13（SPI MISO） |
| CSN / Z | GPIO10，低有效 |
| VDD / GND | 3.3 V / 共地；VDD 附近接 100 nF 去耦 |
| MODE | 接 3.3 V，选择 SSI/I²C 接口 |

SSI 不使用 MOSI。GPIO10/12/13 与现有 PWM、电流采样、USB 和 UART 引脚没有重叠。模块必须能独立引出 CSN/Z；只有 VDD/GND/OUT/SCL/SDA 的 I²C 模块不能直接按这三根信号线连接。完整时序、磁场状态及原 AS5600 接线见[硬件说明](docs/hardware.md)。

更换后先查询 `status`：`encoder` 应为 `MT6701`，`encoder_interface` 为 `ssi_spi_dma`，`encoder_dma` 为 true，`encoder_resolution_bits` 为 14。旧 AS5600 校准不用于 MT6701；完成 `calibrate` 后再 `enable`，停止后 `save`。MT6701 不继承原 AS5600 的弱磁容忍设置。

上轮 MT6701 / 22008 接入联调已在 COM7、MAC `1c:29:04:38:1e:b8` 烧录并完成静态读取、低电流校准、正反向短时释放测试及 USB 回归。静态 15 秒采集 120 份状态快照，编码器失败/重试均为 0，读取耗时 48–60 μs。最终校准使用 0.08 A，正反向测试以 0.08 A 驱动 100 ms、观察 3 秒，均通过末段速度低于 1 rad/s 的检查；初次 2 秒观察未通过末段收敛条件，已保留原记录。这些结果属于 v3 和上轮低电流参数，不作为 v4 提高力度后的验证。

上轮参数与校准已 save 并经过复位后 150 次状态查询核对，当时设备最终保持 idle、fault=none、duty=0。

上轮未进行 0.8 A 额定或 4.5 A 最大电流负载测试，也未用示波器核验 SSI/模拟采样。静止电流遥测仍有噪声，不能将遥测瞬时峰值视作已确认的物理电流。完整过程、校准和本地数据位置见[硬件说明](docs/hardware.md)。

## 2026-11-04 v4 验证

9 组桌面测试通过；诊断固件完成正反向低速和中速驱动/释放测试：请求 ±10 rad/s、0.08 A 驱动 100 ms，以及请求 ±30 rad/s、0.15 A 驱动 300 ms，各观察 3 秒。四组末 500 ms 的角度均无变化、速度降至数值零附近，通过释放收敛检查。

正向低速测试的完整控制频率峰值计数曾记录 **5.6346 A 异常峰**，约 2 ms 间隔导出的波形未捕获该点，目前不能区分真实瞬态与采样异常。原始记录已保留；这项异常未被解释，释放收敛通过也不等于电流测量已经确认，更不能作为 4.5 A 高电流验证。

随后烧录关闭 `CONFIG_KNOB_DIAGNOSTICS` 的生产镜像，USB 六项回归通过，四模式在线 0.02–0.8 A 限流检查通过；idle 下接受 4.5 A / 强度 100，拒绝 4.51 A / 强度 100.01。限流检查首轮在 mode=2 的 enable 收到 `adc_window_invalid` 明确拒绝、设备未进入 fault；保留首轮记录后，工具增加仅对 idle 状态下此类明确拒绝的有界重试，第二轮四模式全部通过，实际重试次数为 0。生产固件烧录重启后连续 60 次查询确认保存参数与旧校准一致、encoder_failures=0、duty=0、fault=none、保持 idle，诊断字段不存在，位置滤波和目标斜坡遥测均为 false。本次没有进行 4.5 A 负载测试。备份、参数和验证原始数据见 [v4 硬件记录](docs/hardware.md#2026-11-04-v4-挡位增强与验证)。

## Windows 编译与烧录

当前实际工具链为 **ESP-IDF 5.5.6**，安装目录名称仍是 `esp-idf-v5.3.2`。本轮设备是 COM7、MAC `1c:29:04:38:1e:b8`，与历史 Linux 记录中的板卡不同。脚本只使用板上原生 USB，不改动上位机工程。

```powershell
$env:IDF_TOOLS_PATH = 'D:\rj\Espressif\v532'
$env:IDF_PYTHON_ENV_PATH = 'D:\rj\Espressif\v532\python_env\idf5.5_py3.10_env'
. D:\rj\Espressif\v532\frameworks\esp-idf-v5.3.2\export.ps1
idf.py build
idf.py -p COM7 app-flash
```

产物为 `build/smartknob_s3.bin`、`build/smartknob_s3.elf`。`app-flash` 保留已有 NVS；首次空白板需要 `idf.py -p COM7 flash`。修改 Kconfig 后执行 `idf.py reconfigure`。ADC 的专用同步适配器限定 ESP32-S3 / IDF 5.5，升级芯片或 IDF 系列须重新验证采样窗口。

桌面测试：

```powershell
cmake -S tests -B build/host -G Ninja -DCMAKE_CXX_COMPILER=C:/mingw64/bin/g++.exe
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

## 运行与验证

上位机和脚本不能同时占用串口。连接 wl1-studio 后先读取设备参数，避免用旧预设覆盖本轮调好的参数。首次使用或换硬件时执行校准，成功后再启用；停止后使用 save 保存参数和校准。电角度校准只改 NVS，不烧写编码器 OTP/EEPROM。

```powershell
python tools/knobctl.py --port COM7 status
python tools/knobctl.py --port COM7 calibrate
python tools/knobctl.py --port COM7 enable --seconds 10
python tools/knobctl.py --port COM7 save
python tools/current_limit_check.py --port COM7
python tools/usb_smoke.py --port COM7
```

`current_limit_check.py` 在 idle 状态检查 4.5 A 配置边界，默认使能测试最多使用 0.8 A；只有显式指定 `--max-current` 才使用更高测试上限。其结果验证协议和目标限幅，不代表实测达到了该电流或完成了负载验证。

需要 pyserial。调试脚本退出会主动 disable。默认 `CONFIG_KNOB_HOST_LEASE_MS=0`，普通主机关闭但未发送 disable 时设备会继续运行；需要租约的应用可将该值设为毫秒数。上电仍为 idle，故障不会自动清除。

板上加速/释放测试需要 `menuconfig → SmartKnob → Enable bounded FreeRTOS motor release tests`。独立测试任务通过控制队列驱动，硬件停止时限在电机任务中执行；测试结束停机、恢复原配置，不写 NVS：

```powershell
python tools/motor_release_check.py --port COM7 --mode 1 --velocity 55 --current 0.15 --drive-ms 300 --observe-ms 3000 --require-cycle --max-tail-speed 1.0 --output build/release.json
```

正常构建关闭 `CONFIG_KNOB_DIAGNOSTICS`。板上数据验证电气行为和释放稳定性，人手的档位质感仍需实际体验。GAIN 尚未用外部仪器确认，电流遥测按 0.15 V/A 换算；当前输出电压和低侧采样窗口也限制了高速时的力矩能力。
