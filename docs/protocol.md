# SmartKnob USB 协议 v1

## 2026-11-04 v7 强度上限扩展

当前构建 `firmware:"dma-v7-strength-range"` 将 `strength_max` 从 100 扩展至 **1000 A/rad**，`detent_profile` 继续为 `smart_knob`。`current_limit_max` 仍为 4.5 A，默认强度、电流、电压、PI、速度及过流保护均不因范围扩展而自动改变。上位机须按 `strength_max` 决定允许输入的范围；仅升级上位机时，旧固件仍会拒绝超过其上限的强度。

v7 已构建、烧录并在停机状态验证 150 / 500 / 1000 接受、1000.01 拒绝，4.51 A 仍拒绝。最终恢复完整原配置与校准，idle / fault:none；未使能高强度输出或写 NVS。[本次验证](../build/strength-1000-20261104/validation.json)

## 2026-11-04 v6 SmartKnob 挡位更新（算法沿用）

v6 构建报告 `firmware:"dma-v6-smart-knob"`、`detent_profile:"smart_knob"`，协议版本仍为 1。挡位恢复为共用中心的迟滞换挡和 P/D 力矩：`snap` 同时决定逻辑换挡及弹力反向位置，`strength` 表示中心弹力死区外的 A/rad，`damping` 作用于整个挡距。完整算法见下方“SmartKnob 挡位力矩与电流响应”。配置字段、NVS 布局、电气参数和速度保护语义保留；`pwm_profile` 继续独立报告构建的 PWM 配置。

v6 已通过 14/14 桌面回归、ESP-IDF 构建并烧录 COM7，运行 ELF SHA256 为 `f76520b34546c85fbf734ad44c76333110e9db8a885cd555cc8a97cd72666cb7`。临时 0.08 A / 0.8 V 下六项 USB／启停检查通过，启用快照为零目标、无故障，累计拒绝采样增加 95；未完成本版手持跨挡或非零电流跟踪验证。最终为 idle / fault:none、`20k70_experimental`，RAM 配置保留原 0.8 A、7.8 V、strength=20、endstop=20、damping=0.03、snap=0.55、10°、Kp=0.4、Ki=200，仅切换 mode=1；校准零点 6.07916546 rad 保持不变。未保存 NVS，重启仍恢复原 mode=3、2.35 V、damping=0.003 等保存值。[验证记录](../build/smart-knob-restore-20261104/validation.json)

## 2026-11-04 v5 PWM 候选、采样恢复与验证范围（历史）

该轮设备报告 `firmware:"dma-v5-bus-sense"`、`pwm_profile:"20k70_experimental"`，使用 `build/rated-current-20261104/bus.sdkconfig` 构建；源码默认仍为 10 kHz / 50%。该 20 kHz 零相钳位候选已完成构建、烧录、idle 及 0.08 A / 0.8 V 上限下约 5 秒零目标使能检查。该记录目标全为 0，累计拒绝采样增加 97，结束 idle / none，不能作为有载电流或降噪验证。[启动快照](../build/rated-current-20261104/20k-clamped-boot.json)、[零目标检查](../build/rated-current-20261104/20k-clamped-low-current.json)

| 字段 | 20 kHz 候选的含义（v6 沿用） |
|---|---|
| `pwm_duty_floor` | 0，表示允许最低相钳为零；不能理解为允许任意小的非零脉冲 |
| `pwm_min_active_duty` | 0.04，所有非零脉冲至少 4%，20 kHz 下为 2 μs |
| `pwm_zero_clamp` | true，零相在相应 PWM 锁存时刻强制 LOW，扇区边界附近可调整公共偏置以保留最小非零脉宽 |
| `pwm_duty_ceiling` | 0.70；可配置上限按已报告的 floor/ceiling 计算，19.4 V 对应约 7.8404 V，旧 0.04–0.70 偏置方式约 7.3924 V |
| `current_sampling` | `pwm_staggered_adc_gdma_experimental`；在相邻 PWM 周期分次转换 |
| `current_phase_skew_us` | 50，两个有效相电流样本的时间差，不能按同时采样解释 |
| `adc_sample_valid` | 最近发布的 ADC 突发通过有效性判断；idle 时可为 true，不等同于 `current_valid` 或实际带载 |
| `sampling_restart_required` | 丢失突发或 PWM 跟踪失步后置 true；丢失突发时停止后续 START，需停机显式 clear 重建 |
| `adc_completed_mask` | bit 0/1/2 标识完整接收的三槽，完整值为 7 |
| `adc_window_after_mask` | bit 0/1/2 标识已捕获对应槽后边界的 PWM 比较值 |
| `adc_trigger_elapsed_us` | 三次 START 相对突发起点的时间；20 kHz 候选名义为 0/50/100 μs |
| `adc_pwm_compare_before` / `adc_pwm_compare_after` | 3×3 数组，分别记录每槽前后 U/V/W 有效比较值的软件跟踪；不能用它们替代示波器测量采样孔径 |

每槽验证低侧窗口，并拒绝两次有效相采样之间跨越电压指令变化的组合。50 μs 的非同时采样仍会受相电流纹波和动态变化影响，第三相重建与 dq 电流幅值须继续验证。此前两次 20 kHz / 6.8 V 保持尝试，一次目标全为 0，另一次等待 90 秒未形成有效非零保持；新零相钳位版本的有载与噪声结论仍待验证。

5 kHz / 70% 的有效保持分析有 135 点，沿目标方向的 Iq 均值约 0.756435 A、目标约 0.755888 A、Id 约 0.001134 A、电压矢量约 6.7961 V、加载母线约 19.1555 V。三种采样槽分组仍有差异；约 29.969 秒内拒绝采样增加 2574，有效控制 147295 次。用户反馈该版音调和响度更高，因此不能把它当作降噪通过；数据也不证明固定 0.8 A 连续运行合格。详细分组和证据见[硬件记录](hardware.md#2026-11-04-当前验证状态5-khz-电流能力与-20-khz-候选)。

`save` 只允许在失能状态执行。NVS 写入尝试结束后，固件自动使用内部操作重建采样；该内部操作不接受 JSON 调用，不确认已有故障，也不重新使能。返回前在 400 ms 共用恢复期限内等待新完整 ADC 快照；写入失败返回 `save_failed`，重建或等待失败返回 `hardware_restart_failed`。后者不表示 NVS 一定未写入，应查询状态核对参数。手动采样失步恢复仍使用显式 `clear`；恢复后继续要求显式 `enable`。[实机保存验证](../build/rated-current-20261104/save-fault-restart.json)同时保留原 `adc_window_or_control_deadline` 故障并返回有效新 ADC 突发，未隐式清故障。

该轮 NVS 保存的名义母线/电流上限/电压上限为 **19.4 V / 0.8 A / 2.35 V**，Kp=0.4、Ki=200、mode=3；校准零点为 **6.0791655 rad**、direction=+1、7 极对。7.2 V 保持试验仅修改 RAM，未保存。相电流绝对尺度、温升和声学指标尚未完成外部仪器验证。

## 2026-11-04 母线电压与发送更新

新增 `dma-v5-bus-sense` 系列固件，协议版本仍为 1。用户已确认驱动板 H1-10 接到 GPIO11；ADC2_CH0 母线遥测约 19.43–19.47 V，与用户报告的 19.4 V 供电接近。驱动板 47 kΩ / 4.7 kΩ 分压的换算倍率为 **11**；接线、原理图依据及板上读回见[硬件记录](hardware.md#2026-11-04-新增母线电压采样)。该范围为实机快照观测，不是外部仪器标定结果。

`config.bus_voltage` 继续表示名义母线配置，范围 4.5–30 V；启用母线采样后，实际调制使用顶层 `bus_voltage`，配置不会被每次 ADC 读数覆盖，也不会自动写入 NVS。可配置电压上限和实时可用电压分别报告，避免客户端将实测母线波动误解成配置变更。

| 字段 | 含义 |
|---|---|
| `bus_voltage_sensing` | 是否编译启用 GPIO11 母线采样；通用 Kconfig 默认关闭，接线后的现场构建启用 |
| `bus_voltage_source` | 启用时为 `adc2_gpio11`；关闭时为 `configured` |
| `bus_voltage` | 当前调制所使用的母线 V；采样开启且读数无效时为 0，不回退到配置值 |
| `bus_voltage_valid` | 最新采样有效、读取成功、时间戳有效且年龄不超过 100 ms；关闭采样时表示配置母线有效 |
| `bus_voltage_age_us` | ADC 样本年龄，μs；尚无时间戳时为 1000000000；仅在采样启用时解释为测量年龄 |
| `bus_voltage_measured` | 最新 ADC 换算的母线 V；故障时可保留越界读数，只能结合 valid/error 解释 |
| `bus_voltage_raw` / `bus_voltage_adc_mv` | GPIO11 原始 12 位 ADC 码 / 校准后的引脚 mV；不是相电流 ADC 数据 |
| `bus_voltage_error` | 最近读取或初始化的 ESP-IDF 错误码，0 表示成功；无效范围使用 `ESP_ERR_INVALID_RESPONSE` |
| `bus_voltage_divider` | 分压换算倍率，默认约 11；由 `CONFIG_KNOB_BUS_DIVIDER_MILLI / 1000` 决定 |
| `max_voltage_limit` / `configured_max_voltage_limit` | 按 `config.bus_voltage × (pwm_duty_ceiling − pwm_duty_floor) / √3` 计算的可配置上限；两字段相同，与 configure 校验一致 |
| `available_voltage_limit` | 按实际使用母线和同一 duty span 计算的电压能力；无效母线时为 0 |
| `effective_voltage_limit` | `min(config.voltage_limit, available_voltage_limit)`；active 电流环的电压矢量限制，校准仍另限至最多 0.8 V |

新固件取消此前固定 5 V 的软件电压上限，继续按当前 PWM 占空比边界计算能力。电源下降时可用上限和 PI 抗饱和限制随之下降；电源上升不会突破用户的 `config.voltage_limit`。这些电压字段描述控制与调制能力，不是电机端电压的外部测量。PWM 频率和边界以当前设备报告的 `pwm_hz`、`pwm_profile`、`pwm_duty_floor`、`pwm_duty_ceiling` 为准。

母线任务在 core 0 每 10 ms 读取 ADC2，不在电流 ISR 或电机任务中等待转换。ADC1 电流 DMA 不加入母线通道。开启采样后，初始化/读取错误、ADC 饱和、换算值低于 4.5 V 或高于 30 V 均不可用于驱动；运行时锁存 `bus_voltage_invalid`，样本年龄超过 100 ms 时锁存 `bus_voltage_stale`。尚未获得有效母线的 `enable` / `calibrate` 返回 `bus_voltage_not_ready`。停止后显式 `clear` 在母线仍无效时请求工作任务重新初始化，并清除旧快照；应轮询到新的 `bus_voltage_valid:true` 后再显式使能。读数恢复本身不清故障，也不自动重新启动。

USB 发送改为每次最多 **512 字节**，修复扩展诊断回复超过 4096 字节时整段提交受 TX 环形缓冲区单项大小限制的问题。分块保留部分写入处理，整段 JSON 正文共用 100 ms 截止时间；线上格式和协议版本不变，客户端仍按 LF 拼接一条最长 8192 字节的回复，而不是把 512 字节当成报文边界。相电流绝对尺度仍待外部标定，新增母线读数不代表负载电流已完成验证。

## 传输与请求

默认使用 DevKitC 的原生 USB Serial/JTAG（板上 USB 口，GPIO19/20）；主机统一以 115200、8N1、无流控打开串口，原生 USB 不受该波特率限制。编译时可选 COM/UART0，此时 115200 为实际线路速率。两者由 `CONFIG_KNOB_CONSOLE_USB` / `CONFIG_KNOB_CONSOLE_UART` 互斥选择，USB 能枚举或输出日志不能证明其已启用调参。每条请求/回复是一个 UTF-8 JSON 对象，以 LF 结束。请求最大 2047 字节；主机接受最多 8192 字节的一行。超长帧丢弃到 LF，不把尾部当成新命令。日志行不属于协议；默认 `CONFIG_ESP_CONSOLE_NONE=y` 避免应用日志与 JSON 混流。回复前后均带 LF，主机忽略空行与启动日志。一次请求对应一次回复，无主动推送。USB TX 支持部分写入并限制等待时间，后续帧前的 LF 可恢复被中断的帧边界。

```json
{"id":1,"op":"status"}
```

每次回复包含 `device:"smartknob-s3"`、`protocol:1`、相同的 `id`、`ok`，以及当前状态/参数/校准/遥测。拒绝请求时额外含 `error`；解析或校验失败也查询真实快照，但不会续期心跳。控制任务不可用时会返回 `control_unavailable`，附带的默认快照不可作为真实测量。主机必须校验身份、协议和请求 ID，不能因为任意串口返回 JSON 就认作旋钮。

可选字段 `transport` 为 `usb_serial_jtag` 或 `uart0`，用于显示设备实际使用的调参通道，不改变协议版本。编码器另由 `encoder`、`encoder_interface`、`encoder_resolution_bits`、`encoder_dma` 标识，默认值分别为 `MT6701`、`ssi_spi_dma`、14、true；切换源码中的 AS5600 后端时分别为 `AS5600`、`i2c`、12、false。旧上位机可以忽略新增字段，但不能继续把所有 `magnet` 数值当作 AS5600 STATUS。连接初期可在约 3 秒内重试只读 `status` 以等待固件启动；运行期请求维持短超时，不自动重发使能、校准等动作。`control_unavailable` 的默认快照必须作为通信失败处理，不得展示为真实 idle 状态。

| op | 语义 |
|---|---|
| `status` | 查询全部快照，仅在启用 `KNOB_HOST_LEASE_MS` 时续期通讯租约 |
| `configure` | 完整替换 `config`；先整体验证，失败时不修改任何参数 |
| `calibrate` | 从 idle 开始电流零点、固定角对齐、一电周期慢扫、方向及电角度零点校准；对齐及收敛时必须通过平均 d/q 电流跟踪检查与连续 200 ms 的角度静止检测；完成后回到 idle |
| `enable` | 必须已经校准且 idle；重采电流零点后启动手感 |
| `disable` | 立即释放电机；已有 fault 保留，便于诊断 |
| `clear` | 停机状态下清除锁存故障；初始化失败时重试硬件初始化 |
| `zero` | 在当前位置重设逻辑档位中心，不改编码器 OTP 或电角度校准 |
| `pulse` | active 时播放 3 ms 正向 + 3 ms 反向小电流脉冲，受电流限制和高速触觉衰减约束 |
| `save` | 仅停机时，写入带版本/CRC 及编码器身份的 NVS 配置和校准；写入尝试后重建采样并返回新快照，保留已有故障，不保存或恢复 enabled 状态 |

## 完整配置示例

```json
{"id":2,"op":"configure","config":{"bus_voltage":19.4,"sense_gain":0.15,"current_limit":0.8,"voltage_limit":2.35,"current_kp":0.4,"current_ki":200,"width":0.1745329252,"strength":20,"endstop":8,"damping":0.003,"snap":0.55,"min_position":0,"max_position":11,"mode":1}}
```

所有数值必须有限；NaN/Infinity、缺字段、整数越界均拒绝。

| 字段 | 单位和范围 |
|---|---|
| bus_voltage | V，4.5–30；名义配置，决定 configure 可接受的电压上限；ADC2 开启时实际调制使用有效实测值 |
| sense_gain | V/A，只支持 0.15/0.3/0.6/1.2 |
| current_limit | A，0.02–4.5；默认 0.8，运行目标上限，实测越界先收减扭矩 |
| voltage_limit | V，0.05–bus_voltage × (pwm_duty_ceiling − pwm_duty_floor)/√3；边界由设备当前 PWM 配置报告 |
| current_kp / current_ki | V/A，0–20；V/(A·s)，0–1000 |
| width | rad，1°–180°；页面以度编辑 |
| strength / endstop | A/rad，0–1000 / 0–20；默认 20 / 8；v6/v7 档位 strength 为中心弹力死区外的斜率，目标受 current_limit 限制；弹簧和端点语义保留 |
| damping | A·s/rad，0–0.5；v6 普通挡位在整个挡距使用速度阻尼，中心静止门限见下文 |
| snap | 档距倍数，0.5–1；默认 0.55；偏离当前中心严格超过此倍数时换挡，并在同一周期按新中心计算弹力 |
| min_position / max_position | 有序整数，±10000 |
| mode | 0=自由，1=档位，2=有限行程，3=弹簧 |

运行期间允许修改手感参数；电气参数需要先停机。只调整强度、阻尼、切换点和电流限制时保留档位中心与电流环积分；模式、档距或行程边界改变时在当前机械位置重建中心，逻辑位置为 clamp(0, min_position, max_position)。被状态机拒绝的请求（例如运行中重复 enable）也不会续期通讯看门狗。显式修改配置母线/增益会清除电角度有效标志，要求重新校准；正常 ADC2 母线更新不修改配置，也不使校准失效。

运行电流上限与异常停机阈值用途不同。目标始终限制在 `±current_limit`，并按实测相电流和 d/q 电流进一步收减。v6 沿用自 v4 起的分级瞬态保护：

| 条件（任一相的实测或重建电流幅值） | 处理 |
|---|---|
| 达到 `min(5, current_limit + max(0.15, current_limit × 0.25))` A | 先保持闭环降额，累积超限暴露达到 20 ms 后锁存停机 |
| 达到 5 A | 使用独立的快速暴露计时，累积达到 1 ms 后停机 |
| 达到 6 A | 当前有效采样立即停机，不等待上述计时 |

两个暴露计时器在越界时按实际采样时间增加，低于各自阈值时以四分之一速度恢复。因此单个短尖峰可以恢复，频繁尖峰不能靠短暂回落反复清零。无效采样冻结暴露，持续采样失效由独立的 1.5 ms 看门狗停机。ISR 和控制任务使用同一判定规则。在线降低上限立即收紧目标，旧普通保护阈值最多保留 20 ms 以等待电流衰减；快速和立即停机阈值不受此过渡影响。

4.5 A 是软件可配置目标上限，默认仍为 0.8 A；开放上限不代表已经实测达到 4.5 A，也不把最大电流当成持续运行额定值。5 A / 6 A 为异常保护阈值。电流换算仍依赖正确的 `sense_gain`；电源输入电流不等于 PWM 驱动下的绕组相电流。感测信号越出 0.25–3.05 V 仍停机，较高感测增益或偏移零点会缩小可测电流范围。电压上限受低侧有效采样窗口约束；此前 10 kHz / 50% 实验在 11.62 V 配置母线下最大约 3.086 V，其他 PWM 配置应读取当前设备报告的能力，实际电流还受绕组及反电动势限制。

## SmartKnob 挡位力矩与电流响应

v6/v7 构建的 `detent_profile:"smart_knob"` 标识当前算法。普通挡位和有限行程内的普通挡位使用以下规则，电流方向仍为编码器机械坐标：

1. 以 `error = angle − center` 判定换挡。仅在 `abs(error) > width × snap` 时移动中心及逻辑 position，随后重新计算 error；有限行程夹紧在 min/max。跨多挡使用 `ceil((abs(error) − width × snap) / width)` 计算次数，修正旧 `floor(...) + 1` 在精确多挡阈值处多进一挡的问题。
2. 中心弹力死区 `dead_zone = min(width × 0.2, π/180)`。弹力输入 `input = −error + clamp(error, −dead_zone, dead_zone)`，原始目标为 `strength × input − damping × velocity_fast`。这里的 center 与换挡判定相同，不再按半挡距重新取模。因此跨过 snap 后，弹力在该周期转向新的中心；未跨挡时，死区边缘的弹力连续起升。
3. 只有 `input == 0` 且 `abs(velocity_fast) <= 0.05 rad/s` 时将普通挡位目标归零，避免中心静止时速度量化引入小阻尼目标。中心外即使弹力很小也不会被幅值门限切掉；中心内正常转动仍有阻尼。
4. 合成目标先限制在 `±current_limit`，再保留同向助推衰减、共同高速包络及实测电流限幅。静态正向切换前的弹力幅值上界为 `min(current_limit, strength × max(width × snap − dead_zone, 0))`；动态目标还包括阻尼，实际电流受电压能力约束。`strength × width / 2` 是旧周期曲线的峰值公式，不适用于 v6。

历史 `dma-localized-v4`、`dma-localized-v4-10k50-voltage-exp3` 和 `dma-v5-bus-sense` 的 `detent_profile` 为 `localized_barrier`：中心两侧各 0.3 挡距内为零弹力、零阻尼，峰值固定在 0.4 / 0.6 挡距，0.5 挡距的弹力为零，snap 只改变逻辑 position。这解释了逻辑换挡与波形峰值错位；该曲线已由 v6 的共用中心 P/D 替代。协议版本及已有配置字段保持兼容，但相同 strength/damping 数值的手感会变化。

`quiet_output` 表示电气静音：原始力矩意图严格为零且速度低于 0.25 rad/s 连续 20 ms 后，清空 d/q PI 并输出零电压。任何非零力矩意图立即恢复连续电流控制；速度超过 0.7 rad/s 也立即恢复，以便抵消反电动势、避免转动空隙产生短路制动。进入/退出速度迟滞避免静止编码器噪声反复开关。这不是按电流幅值硬切断，微小非零跨档力矩仍保持连续。

v6 继续沿用 v4 已清理的输入路径：不使用 0.15 ms 双级位置平滑、一码偏差限制或两编码器区间挡位速度分支。换挡判定和力矩计算直接使用同一个连续角度加快速速度预测；按时间戳与传感器延迟补偿的预测保留，0.5 ms 速度滤波继续服务于预测和阻尼。电流 Park 变换仍使用电流采样时刻的预测电角度。

电流目标增加、撤力和反向直接生效，已删除 1000 A/s 的额外目标上升斜坡；目标仍受实测超限降额及 d 轴电流余量限制，实际绕组电流由 PI 与电压能力决定。电阻前馈在电压圆约束内与 PI 相加。`current_feedforward_ohms` 报告 Kconfig 中的近似绕组电阻，当前默认 0（关闭）；不是在线测量值，不属于 NVS Config。此前 4.5 Ω 是旧 2804 的估计值，换用 22008 后未复用。

弹簧与行程端点的回程阻尼取 `max(damping, 0.04)` A·s/rad，防止将低阻尼档位参数用于持续弹性回中而产生余振；普通挡位仅使用配置 damping，不使用该最小值。遥测 `elastic_min_damping` 报告该值。自由模式、弹簧和端点的力矩规则未随普通挡位恢复而改变。

## 高速触觉衰减

正常力反馈运行时，超速属于临时触觉限制：状态保持 `active`、`fault:"none"`，不拉低驱动使能，也不要求重新点击使能。绝对速度在 25–40 rad/s 之间时以 smoothstep 曲线逐渐减弱全部效果；达到 40 rad/s 后进入暂停，力度系数在约 10 ms 内收敛到 0。回落到 5 rad/s 或以下并连续稳定 30 ms 才解除暂停，力度从 0 到 1 至少用约 500 ms，避免在阈值附近反复开关。再次超速会重新收减。

力度在所有效果（含阻尼、弹簧、端点和点击）合并并限流后统一缩放，再经过实测电流限幅和 d/q 电流 PI；暂停期间仍闭环调节零目标电流。虚拟档位持续跟踪角度，恢复不会追回积压档位。模式切换、在线调参及 `zero` 不重置高速暂停状态；弹簧原点和有限行程边界保持各自语义。阈值固定在固件中，不更改 `Config` 或 NVS 格式。

普通挡位的同向助推在绝对速度 0–10 rad/s 保持全量，10–25 rad/s 随速度线性减弱，25 rad/s 时归零；跨档前的反向阻挡保留，再经过共同的高速包络。弹簧/行程端点继续使用 10–25 rad/s 的回程助推衰减。低速过档保留完整助推，高速连续跨档和弹性回程仍限制持续补能。v6 的速度阻尼作用于整个挡距，中心静止门限按上述 SmartKnob 规则处理。可选遥测 `detent_assist_fade_start` / `detent_assist_cutoff` 分别报告 10 / 25 rad/s。

这是目标扭矩的渐变，不代表实际绕组电流瞬间归零。异常过流、磁铁丢失、编码器跳变/传输失败/CRC 失败、采样失效及启用租约时的主机超时仍锁存停机。校准及零点采样阶段没有触觉输出包络，过快运动仍中止，原因是 `calibration_motion_too_fast`。

## 采样与任务时序（前轮 10 kHz / 50% 实验）

本节保留前轮实验的触发间隔、窗口与诊断字段说明。下文的固定 5 V 校验和 `dma-localized-v4` 构建条件属于该次记录；母线采样 v5 的电压语义以上方更新为准，新 PWM 配置与验证结果须按各自实验记录解释。

`current_sampling:"pwm_triggered_adc_gdma"` 表示 ADC1 使用 GDMA，两相采样与 PWM 同步。当前实验将 PWM 从 20 kHz 改为 10 kHz，经过 40% / 60 μs 对照验证后，当前占空比限制在 4%–50%，公共低侧窗口为 50 μs，三次名义 START 事件由 −10/0/+10 μs 改为 −15/0/+15 μs，以容纳 DMA 回调及诊断开销；45 μs 整组时限保留。因此该实验同时改变了 PWM 和触发间隔。电压校验为 `min(5, bus_voltage × 0.46/√3)`。上位机从两个 duty 边界计算输入/校验范围，并使用 `max_voltage_limit` 核对当前母线对应的设备能力；旧固件未提供有效的成对边界时退回 4%–40%，不会凭固件名称放开。控制默认 5 kHz，诊断可编译选择 2 kHz。`control_nominal_hz` / `encoder_nominal_hz` / `pwm_hz` 分别报告三个名义频率。硬件固定扫描表避免运行时改通道影响采样；按现有流水线模型丢弃每次突发首字、使用后两字；该模型仍需验证样本新鲜度。后两个结果检查 ADC 单元、通道、范围及 PWM 窗口，不能通过校验的数据不进入电流环。每个 DMA 字仅复制一次后解码，避免字段来自不同次内存读取。编码器独立任务约 2.5 kHz 更新；默认 MT6701 使用 SPI2 2 MHz SSI DMA，保留的 AS5600 后端使用 400 kHz I²C。2 kHz 控制只消费最新快照，不能消费每一帧。快照时间戳用于电角度预测，并采用所选芯片的延迟补偿（MT6701 5 μs、AS5600 150 μs）。电流任务由采样完成事件唤醒，编码器任务由 PWM 分频事件唤醒；等待通知的 5 ms 是超时，PI 的 dt 使用 ADC 时间戳差，不使用相对 RTOS 延时定周期。

v4 保留兼容字段 `detent_quiet_velocity:false`，实验性的两编码器区间挡位速度估计及其 Kconfig 选项已删除。新增 `haptic_position_filter:false` 表示未使用原来的双级手感位置平滑，`current_target_slew:false` 表示未使用额外电流目标上升斜坡；这些字段不关闭速度估计、实测电流限幅或 PI。`velocity_fast` 报告 0.5 ms 快速速度通道，挡位、换相预测、弹簧和端点使用该通道。

`control_hz` 是有效控制次数在约 0.5 s 窗口内的实测值，启动第一窗口前为 0，不是配置值。`control_cycles` 为累计有效计算次数，`missed_samples` 为任务未消费的采样序号数；两者与累计 `rejected_samples`、`reused_samples` 分别表示不同现象。`control_interval_us` 为最近消费采样之间的时间，`control_max_us` 为控制计算耗时峰值（enable 时重置）。`encoder_age_us` / `encoder_age_max_us` 是最近编码器快照的年龄及峰值。`adc_dma_words` 为两相原始 DMA 数据字，供调试通道标签与原始码。

10 kHz 采样/电压实验兼容追加以下状态字段，不改变 `adc_dma_words`、配置结构或 NVS 格式。数组均有 3 项，按本次突发的顺序排列；时间字段是软件观测到的触发/回调时刻，不是直接测得的 ADC 采样保持时刻。

| 字段 | 含义 |
| --- | --- |
| `adc_slot_words` | 三次 DMA 回调各复制一次的原始字，包含首个被控制路径丢弃的流水线结果；后两项对应原有 `adc_dma_words` |
| `adc_slot_mask` | bit 0/1/2 表示对应槽收到预期大小的 DMA 原始字；只解释相应 bit 为 1 的字数据，完整三槽为 7 |
| `adc_trigger_ticks` / `adc_callback_ticks` | 三次触发前 / 三次 DMA 回调入口的 PWM 带方向计数；DOWN 为负、UP 为正 |
| `adc_trigger_down_mask` / `adc_callback_down_mask` | bit 0/1/2 为对应观测时刻的计数方向，1=DOWN、0=UP；用于保留 count=0 时符号无法表达的方向 |
| `adc_trigger_sar` / `adc_callback_sar` | START 前 / 回调清 START 后的 SAR `meas_status` 原始状态；与计数、时间读取非原子，可能被高优先级中断分隔 |
| `adc_callback_elapsed_us` | 各回调距离本次采样起点的微秒数，用于区分 PWM 计数回绕或跨周期回调 |
| `pwm_compare_before` / `pwm_compare_after` | 第一次触发前 / 后续触发时，按 TEZ 锁存链跟踪的三相实际比较值；不是尚未生效的软件 pending 值 |
| `pwm_latch_sequence` | 第一次触发前的 PWM 锁存代次，辅助识别同一突发中 PWM 更新跨越 |
| `pwm_compare_valid` | TEZ 服务及时且比较值影子→生效的软件跟踪链连续；为 false 时不可把 before/after 数值当作已确认生效的比较值。这不是直接读取硬件 active 寄存器；本实验检测到跟踪异常后保持 false，重启后重新建立跟踪 |
| `pwm_apply_delay_us` | 本次电压指令提交到预计 PWM 锁存的延迟，单位 μs；用于输出角度预测，不是电压测量 |
| `pwm_duty_floor` / `pwm_duty_ceiling` | SVPWM 允许的占空比边界；当前为 0.04 / 0.50，旧客户端缺省窗口为 0.04 / 0.40 |
| `max_voltage_limit` | 当前配置母线下允许的配置电压上限；v5 不再固定限制为 5 V，实时能力另见 `available_voltage_limit`；不是实测电机端电压 |
| `pwm_half_period_ticks` / `pwm_resolution_hz` | PWM 半周期计数和计数时钟；本实验分别为 4000 和 80000000，即每微秒 80 tick |

这些诊断值与最新采样一起发布，不能仅凭一份低速 `status` 推断所有控制周期的行为。诊断读取仍每次最多返回 4 条记录，完整 JSON 回复须小于客户端 8192 字节行上限；追加字段会增加串口传输时间。

`reset_reason` 是 ESP-IDF 复位原因枚举，`uptime_ms` 是本次启动时间，`minimum_free_heap` 是启动后最小剩余堆内存字节数；这些字段用于区分端口异常、设备复位和资源问题。

`app_elf_sha256` 为 `esp_app_get_description()` 中完整的 32 字节 ELF SHA-256，以 64 个小写十六进制字符输出。它用于将运行设备与本地应用镜像描述符对应；它不是 `.bin` 文件整体的 SHA-256，也不受启动日志摘要显示长度限制。`firmware` 字符串说明实验系列，唯一构建核对使用此摘要。

## 遥测

- `state`：idle/offsets/align/sweep/settle/active/fault；命令已接受与校准已完成是两个不同事件，应继续轮询状态。
- `angle` / `velocity`：连续机械角度 rad / rad/s；`position` 和 `sub_position` 是逻辑档位及其小数偏移。`velocity` 采用 10 ms 滤波用于高速包络；`velocity_fast` 采用 0.5 ms 滤波用于阻尼和弹性回程限速，减少反转时的控制滞后。
- `id_current` / `iq`：电角度坐标下的 A；只有 `current_valid:true` 时代表有效电流。休眠/校准零点/无效采样期间不得把保留值绘成实时曲线。`target` 是编码器方向的目标 A。作电流曲线时 q 轴目标为 `target * calibration.direction`。
- `currents`：重建后的 U/V/W A；每次取两相，第三相由和为零重建，感测相对按轮换更新。
- `sense_volts` / `offsets`：三个通道最近电压及零点 V。
- `duty`：三相占空比；`sample_us`：PWM 同步 ADC 两相有效采集窗口的耗时（不含尾部 DMA 刷新）；`loop_us`：本次任务计算耗时；`rejected_samples`：启动后的累计无效采样数；`reused_samples`：重复任务通知对应已处理样本的累计次数，此类通知跳过，不重复控制也不计作 ADC 失败。
- `magnet`：所选编码器的原始磁场状态。MT6701 为 Mg[3:0]：低两位 0=正常、1=过强、2=过弱、3=保留且拒绝，bit 3 为失去跟踪，bit 2 为按压检测；正常角度可伴随 bit 2，因此不能仅判断 `magnet == 0`。AS5600 为原始 STATUS，正常有效位为 `(magnet & 0x38) == 0x20`（MD=1，MH=ML=0），不能用原值严格等于 0x20 判断。`weak_magnet_allowed` 仅在 AS5600 后端且启用对应 Kconfig 时为 true；MT6701 始终为 false。
- `hardware_detail` / `hardware_error`：初始化失败操作及 ESP-IDF 错误码；`sample_start_tick` / `sample_end_tick` 是采样相对 TEZ 的有符号 80 MHz 时钟位置，用于检查低侧窗口。
- `encoder_error` / `fault_magnet`：锁存至 clear 的编码器错误码及原始磁场状态。`fault:"encoder_transport"` 表示所选接口传输失败/超时，`encoder_crc` 表示 MT6701 帧 CRC 不符；这两种情况下磁场数据不可信。`encoder_missing_magnet` 表示 AS5600 缺磁铁，`encoder_magnet_strength` 表示磁场强度异常（MT6701 也用于保留的强度状态），`encoder_loss_of_track` 表示 MT6701 失去跟踪。通用故障名取代旧的 `encoder_i2c`，业务层不按芯片原始状态位判断。`encoder_failures` 统计最终失败，`encoder_retries` 统计驱动实际执行的重试。

编码器邮箱单独保留首个未被控制任务读取的失败，后续正常快照不会覆盖它。`clear` 确认此前的错误；持续失败或清除后新到达的失败仍会触发故障。

- `voltage_d` / `voltage_q`：实际发往 SVPWM 的电压分量 V，用于观察电流 PI 是否已到电压上限。
- `encoder_us` / `control_us`：最近一次编码器读取及控制计算耗时 μs。`fault_sample` 在采样/控制时限故障或过流后出现，保留触发样本的 `valid`、`interval_us`、`age_us`、`start_tick`、`end_tick`、`duration_us`，直到 `clear`，便于区分窗口错误和过期采样。可选 `adc_dma_words`、`adc_raw`、`adc_phase` 分别锁存同一故障样本的两字原始 DMA 数据、12 位 ADC 码、相号（0/1/2）；它们不是停机后继续刷新的顶层 `adc_dma_words`。`valid:false` 时原始字段仅用于排查，可能不完整，不代表可用电流。旧固件可以没有这些字段；`clear` 同时清空故障原始数据。
- `current_limit_max`：固件接受的运行上限，当前为 4.5 A；旧协议 v1 固件无此字段时，上位机按 0.8 A 处理。
- `strength_max`：固件接受的强度上限，v7 为 1000，v4–v6 为 100；旧固件没有此字段时，上位机应保留旧版 10 的限制。此字段不改变 `config` 或 NVS 格式。
- `current_trip_limit` / `current_absolute_trip`：当前持续过流阈值 / 单样本立即停机阈值 A；前者会在降低运行上限的 20 ms 过渡期内保留旧值。
- `current_peak_trip` / `current_peak_trip_ms`：快速保护阈值 5 A / 暴露预算 1 ms。`current_overload_trip_ms` 为普通超限暴露预算 20 ms，`current_trip_recovery_ratio` 为恢复时间倍率 4。`current_absolute_trip` 当前为 6 A。上位机应读取固件报告值，不沿用旧版 1.25 A / 1.75 A 常量。
- `current_target_limit` / `current_limited`：收减后可用的 q 轴目标幅度 A / 是否正在限流；停机时幅度为 0。
- `fault_current_valid` / `fault_currents` / `fault_current_trip_limit`：首次过流停机时的有效标志、U/V/W 电流 A 和当时持续过流阈值 A；锁存至 `clear`，不受停机后失效的实时电流影响。`fault_motion` 另锁存过流时最近的 angle / velocity / velocity_fast，避免将停机后速度误当作故障时速度。上述可选字段不改变协议版本。
- `haptic_scale`：实际触觉力度系数 0–1；非 active 状态为 0。`speed_limited` 表示力度尚未完全恢复；`haptic_paused` 表示达到高速阈值后仍等待减速，暂停刚开始的淡出阶段系数可以大于 0。`haptic_fade_start` / `haptic_speed_cutoff` 是渐弱及暂停阈值（rad/s，当前 25 / 40）。`haptic_resume_speed` 为恢复速度（5 rad/s），`haptic_resume_hold_ms` 为低速保持时间（约 30 ms），`haptic_fade_in_ms` 为完整恢复时间（500 ms）。这些是可选遥测字段；旧上位机可忽略，协议仍为 v1。
- `calibration`：valid、direction（±1）、pole_pairs（固定 7）、electrical_zero（rad）。NVS 同时保存编码器身份；读取旧版本时将其视为 AS5600 校准。与当前源码所选类型不匹配时仍恢复 `config`，但 `calibration.valid` 为 false，`enable` 返回 `calibrate_first`。重新校准并在停机后 `save`，才保存新编码器的校准。更换同型号传感器或调整机械安装也应重新校准，类型身份不能自动识别这类机械变化。

设备不自动重连、不自动清除或恢复锁存故障。正常 active 会话中的高速触觉衰减会按上述阈值自动恢复；这不解除任何锁存故障。上位机在每次回复后等待约 40 ms 再轮询；完整 JSON 在 115200 baud 下的传输耗时也计入实际刷新周期。退出/切换产品/关闭窗口会停机并释放串口。默认 `KNOB_HOST_LEASE_MS=0`，USB 丢失或停止查询不会单独触发停机；设备保持已明确启用的手感。退出软件时应显式发送 disable。需要通讯租约的产品可在构建时设置正数毫秒值，此时仅成功接受的命令续期，超时锁存 `host_timeout`。

## 可选板上诊断构建

仅 `CONFIG_KNOB_DIAGNOSTICS=y` 接受以下命令；正常镜像返回 `unknown_op`。

```json
{"id":10,"op":"diagnose","mode":1,"velocity":55,"current":0.15,"drive_ms":300,"observe_ms":3000}
{"id":11,"op":"diagnostic_read","offset":0}
{"id":12,"op":"diagnostic_hold","current":0.15}
```

`diagnose` 仅在 idle 且已校准时启动独立 FreeRTOS 任务。参数：mode=0–3，velocity 的绝对值 10–60 rad/s，current=0.02–0.25 A，drive_ms=50–500，observe_ms=500–3000。`current` 只限制驱动阶段的测试目标；释放阶段恢复当前配置的手感及运行电流上限。任务按当前电气参数和指定模式加速、释放，正常电流调节和所有硬件保护保留；速度超过 75 rad/s 中止。测试时间由电机控制任务限定，测试任务在该固定计划内续期租约，所以无需 USB 持续轮询；结束总是停机并恢复原配置，不保存 NVS。上电不自动测试。

诊断运行期间只接受主机 `status` 和 `disable`；后者取消计划且阻止排队的测试使能重新运行。普通配置/使能命令被拒绝。完成后 `diagnostic_read` 每次取最多 4 个样本，offset 为已有样本数，从 0 逐页读取。

可选 `diagnostic` 对象含 busy、cancelled、stage、count、peak_speed、peak_current、muted_samples、recovered。stage：0=空闲，1=准备，2=测试驱动，3=释放，4=完成，5=中止，6=普通手拨记录中，7=手拨记录已冻结。recovered 表示释放阶段经历完整静音后力度恢复到 1；不等同于机械已完全静止。驱动阶段的测试电流用于模拟外部甩转，因此不受触觉静音系数影响，仍受电流限制约束。

样本为数组：`[elapsed_us, angle, velocity, target, iq, id_current, haptic_scale, voltage_q, phase_peak, flags, velocity_fast, voltage_d, adc_raw_a, adc_raw_b, sample_start_tick, sample_end_tick]`，加速/释放测试间隔约 2 ms，容量 2048。v2 在旧版 12 项数组末尾追加两相原始 ADC 码及采样窗口时刻，便于排查孤立电流尖峰；读取旧固件时应接受缺少末尾四项。flags 的低 8 位为 stage，bit 8 为 haptic_paused，bit 9 为 speed_limited，bits 16–17 / 18–19 为两次实际采样的相号 0/1/2。电流方向与普通遥测相同。取回样本仍使用原协议身份、请求 ID 和 8192 字节主机回复缓冲上限（请求上限仍为 2048 字节）。

`drive_ms=50` 的短驱动阶段按完整控制频率记录（目标 5 kHz），便于观察固定电流目标下的采样纹波；释放阶段仍约 2 ms。最长释放 3000 ms 时，约 250 个驱动样本加 1500 个释放样本不会占满缓冲。

`diagnostic_hold` 是有载斜坡排查用的短时目标保持，仅在 active、有有效电流、未运行其他诊断且速度低于 2 rad/s 时接受。`current` 使用机械方向，幅值 0.04–0.25 A，不得超过运行电流上限，符号须与当时目标一致。它保持请求目标最多 750 ms，仍经过正常速度包络、限流、PI 和故障保护。超过起点四分之一挡宽（上限 0.05 rad）或速度达到 5 rad/s 会提前停止。期间只接受 status/disable，不能续期；结束自动失能并冻结普通环形记录，flags bit 11 标识保持样本。该命令不保存配置，不在上电时启动；主机应使用保持前稳定目标的均值，避免负载变化混淆比较。750 ms 保持后的 5 kHz 环形记录只覆盖保持稳态，不包含切换起点。

诊断构建的普通 enable 会复用记录缓冲，以完整控制频率滚动记录最近 2048 个样本（约 410 ms）；disable 或故障后冻结，可通过 diagnostic_read 取出，记录中读取会返回 disable_before_trace_read。手拨记录 flags 的 bit 10 表示 quiet_output，bits 16–17 / 18–19 为两次实际采样的相号 0/1/2。`tools/exercise.py --capture` 在停止后自动保存 `.trace.json`，用于排查 USB 低速查询看不到的快速电流变化。正常构建没有此缓冲或自动记录。
