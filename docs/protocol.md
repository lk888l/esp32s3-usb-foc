# SmartKnob USB 协议 v1

默认使用 DevKitC 的原生 USB Serial/JTAG（板上 USB 口，GPIO19/20）；主机统一以 115200、8N1、无流控打开串口，原生 USB 不受该波特率限制。编译时可选 COM/UART0，此时 115200 为实际线路速率。两者由 `CONFIG_KNOB_CONSOLE_USB` / `CONFIG_KNOB_CONSOLE_UART` 互斥选择，USB 能枚举或输出日志不能证明其已启用调参。每条请求/回复是一个 UTF-8 JSON 对象，以 LF 结束。请求最大 2047 字节；主机接受最多 4096 字节的一行。超长帧丢弃到 LF，不把尾部当成新命令。日志行不属于协议；默认 `CONFIG_ESP_CONSOLE_NONE=y` 避免应用日志与 JSON 混流。回复前后均带 LF，主机忽略空行与启动日志。一次请求对应一次回复，无主动推送。USB TX 支持部分写入并限制等待时间，后续帧前的 LF 可恢复被中断的帧边界。

```json
{"id":1,"op":"status"}
```

每次回复包含 `device:"smartknob-s3"`、`protocol:1`、相同的 `id`、`ok`，以及当前状态/参数/校准/遥测。拒绝请求时额外含 `error`；解析或校验失败也查询真实快照，但不会续期心跳。控制任务不可用时会返回 `control_unavailable`，附带的默认快照不可作为真实测量。主机必须校验身份、协议和请求 ID，不能因为任意串口返回 JSON 就认作旋钮。

新增可选字段 `transport` 为 `usb_serial_jtag` 或 `uart0`，用于显示设备实际使用的调参通道，不改变协议版本。旧上位机可以忽略此字段。连接初期可在约 3 秒内重试只读 `status` 以等待固件启动；运行期请求维持短超时，不自动重发使能、校准等动作。`control_unavailable` 的默认快照必须作为通信失败处理，不得展示为真实 idle 状态。

| op | 语义 |
|---|---|
| `status` | 查询全部快照，并续期 1 s 通讯看门狗 |
| `configure` | 完整替换 `config`；先整体验证，失败时不修改任何参数 |
| `calibrate` | 从 idle 开始电流零点、固定角对齐、一电周期慢扫、方向及电角度零点校准；对齐及收敛时必须通过平均 d/q 电流跟踪检查与连续 200 ms 的角度静止检测；完成后回到 idle |
| `enable` | 必须已经校准且 idle；重采电流零点后启动手感 |
| `disable` | 立即释放电机；已有 fault 保留，便于诊断 |
| `clear` | 停机状态下清除锁存故障；初始化失败时重试硬件初始化 |
| `zero` | 在当前位置重设逻辑档位中心，不改编码器 OTP 或电角度校准 |
| `pulse` | active 时播放 3 ms 正向 + 3 ms 反向小电流脉冲，受电流限制和高速触觉衰减约束 |
| `save` | 仅停机时，写入带版本/CRC 的 NVS 配置和校准；不保存 enabled 状态 |

## 完整配置示例

```json
{"id":2,"op":"configure","config":{"bus_voltage":11.62,"sense_gain":0.15,"current_limit":0.65,"voltage_limit":2.35,"current_kp":1.5,"current_ki":300,"width":0.5235987756,"strength":5,"endstop":8,"damping":0.015,"snap":0.6,"min_position":0,"max_position":11,"mode":1}}
```

所有数值必须有限；NaN/Infinity、缺字段、整数越界均拒绝。

| 字段 | 单位和范围 |
|---|---|
| bus_voltage | V，4.5–30；手动测量 |
| sense_gain | V/A，只支持 0.15/0.3/0.6/1.2 |
| current_limit | A，0.02–1；运行目标上限，实测越界先收减扭矩 |
| voltage_limit | V，0.05–min(5, bus_voltage × 0.36/√3) |
| current_kp / current_ki | V/A，0–20；V/(A·s)，0–1000 |
| width | rad，1°–180°；页面以度编辑 |
| strength / endstop | A/rad，0–10 / 0–20 |
| damping | A·s/rad，0–0.5 |
| snap | 档距倍数，0.5–1；默认 0.6 保留迟滞 |
| min_position / max_position | 有序整数，±10000 |
| mode | 0=自由，1=档位，2=有限行程，3=弹簧 |

运行期间允许修改手感参数；电气参数需要先停机。只调整强度、阻尼、切换点和电流限制时保留档位中心与电流环积分；模式、档距或行程边界改变时在当前机械位置重建中心，逻辑位置为 clamp(0, min_position, max_position)。被状态机拒绝的请求（例如运行中重复 enable）也不会续期通讯看门狗。母线/增益改变会清除电角度有效标志，要求重新校准。

运行电流上限与异常停机阈值用途不同。目标始终限制在 `±current_limit`，并按实测相电流和 d/q 电流进一步收减。保护阈值为 `min(1.25, current_limit + max(0.15, current_limit × 0.25))` A，连续 4 个有效 ISR 样本越界才锁存停机（2 kHz，约 2 ms）；任一有效样本达到 1.25 A 立即停机。在线降低上限立即收紧目标，旧保护阈值最多保留 20 ms 以等待电流衰减。1 A 是可配置运行上限，不代表物理电流完全没有瞬态超调。

## 高速触觉衰减

正常力反馈运行时，超速属于临时触觉限制：状态保持 `active`、`fault:"none"`，不拉低驱动使能，也不要求重新点击使能。绝对速度在 25–40 rad/s 之间时以 smoothstep 曲线逐渐减弱全部效果；达到 40 rad/s 后进入暂停，力度系数在约 10 ms 内收敛到 0。回落到 5 rad/s 或以下并连续稳定 30 ms 才解除暂停，力度从 0 到 1 至少用约 500 ms，避免在阈值附近反复开关。再次超速会重新收减。

力度在所有效果（含阻尼、弹簧、端点和点击）合并并限流后统一缩放，再进入原有电流目标限速及 d/q 电流 PI；暂停期间仍闭环调节零目标电流。虚拟档位持续跟踪角度，恢复不会追回积压档位。模式切换、在线调参及 `zero` 不重置高速暂停状态；弹簧原点和有限行程边界保持各自语义。阈值固定在固件中，不更改 `Config` 或 NVS 格式。

弹簧及有限行程端点在回程中另有加速力矩衰减：同向加速从 10 rad/s 开始减弱、25 rad/s 时归零；反向制动力矩保留，再经过共同的高速包络。这避免大位移回程反复把旋钮加速至暂停阈值。

这是目标扭矩的渐变，不代表实际绕组电流瞬间归零。异常过流、磁铁丢失、编码器跳变/I²C 失败、采样失效及主机超时仍锁存停机。校准及零点采样阶段没有触觉输出包络，过快运动仍中止，原因是 `calibration_motion_too_fast`。

## 遥测

- `state`：idle/offsets/align/sweep/settle/active/fault；命令已接受与校准已完成是两个不同事件，应继续轮询状态。
- `angle` / `velocity`：连续机械角度 rad / rad/s；`position` 和 `sub_position` 是逻辑档位及其小数偏移。`velocity` 采用 10 ms 滤波用于高速包络；`velocity_fast` 采用 2 ms 滤波用于阻尼和弹性回程限速，减少反转时的控制滞后。
- `id_current` / `iq`：电角度坐标下的 A；只有 `current_valid:true` 时代表有效电流。休眠/校准零点/无效采样期间不得把保留值绘成实时曲线。`target` 是编码器方向的目标 A。作电流曲线时 q 轴目标为 `target * calibration.direction`。
- `currents`：重建后的 U/V/W A；每次取两相，第三相由和为零重建，感测相对按轮换更新。
- `sense_volts` / `offsets`：三个通道最近电压及零点 V。
- `duty`：三相占空比；`sample_us`：ISR 中两次 ADC 读取总耗时；`loop_us`：本次任务计算耗时；`rejected_samples`：启动后的累计无效采样数；`reused_samples`：重复任务通知对应已处理样本的累计次数，此类通知跳过，不重复控制也不计作 ADC 失败。
- `magnet`：AS5600 STATUS 原值；正常有效位为 `(magnet & 0x38) == 0x20`（MD=1，MH=ML=0），不能用原值严格等于 0x20 判断。`weak_magnet_allowed` 表示固件是否按用户要求接受 ML=1，缺磁铁/强磁场仍拒绝。
- `hardware_detail` / `hardware_error`：初始化失败操作及 ESP-IDF 错误码；`sample_start_tick` / `sample_end_tick` 是采样相对 TEZ 的有符号 80 MHz 时钟位置，用于检查低侧窗口。
- `encoder_error` / `fault_magnet`：锁存至 clear 的编码器异常现场。`encoder_i2c` 表示传输失败，此时磁场字节无效；`encoder_missing_magnet` / `encoder_magnet_strength` 才表示有效 I²C 数据中的磁场异常。`encoder_failures` 统计最终失败，`encoder_retries` 统计预算内的短暂重试。
- `voltage_d` / `voltage_q`：实际发往 SVPWM 的电压分量 V，用于观察电流 PI 是否已到电压上限。
- `encoder_us` / `control_us`：最近一次编码器读取及控制计算耗时 μs。`fault_sample` 仅在采样/控制时限故障后出现，保留触发样本的 `valid`、`interval_us`、`age_us`、`start_tick`、`end_tick`、`duration_us`，直到 `clear`，便于区分窗口错误和过期采样。
- `current_limit_max`：固件接受的运行上限，当前为 1 A；旧协议 v1 固件无此字段时，上位机按 0.8 A 处理。
- `current_trip_limit` / `current_absolute_trip`：当前持续过流阈值 / 单样本立即停机阈值 A；前者会在降低运行上限的 20 ms 过渡期内保留旧值。
- `current_target_limit` / `current_limited`：收减后可用的 q 轴目标幅度 A / 是否正在限流或平滑目标变化；停机时幅度为 0。
- `fault_current_valid` / `fault_currents` / `fault_current_trip_limit`：首次过流停机时的有效标志、U/V/W 电流 A 和当时持续过流阈值 A；锁存至 `clear`，不受停机后失效的实时电流影响。这些新增字段均不改变协议版本。
- `haptic_scale`：实际触觉力度系数 0–1；非 active 状态为 0。`speed_limited` 表示力度尚未完全恢复；`haptic_paused` 表示达到高速阈值后仍等待减速，暂停刚开始的淡出阶段系数可以大于 0。`haptic_fade_start` / `haptic_speed_cutoff` 是渐弱及暂停阈值（rad/s，当前 25 / 40）。`haptic_resume_speed` 为恢复速度（5 rad/s），`haptic_resume_hold_ms` 为低速保持时间（约 30 ms），`haptic_fade_in_ms` 为完整恢复时间（500 ms）。这些是可选遥测字段；旧上位机可忽略，协议仍为 v1。
- `calibration`：valid、direction（±1）、pole_pairs（固定 7）、electrical_zero（rad）。

设备不自动重连、不自动清除或恢复锁存故障。正常 active 会话中的高速触觉衰减会按上述阈值自动恢复；这不解除任何锁存故障。上位机在每次回复后等待约 40 ms 再轮询；完整 JSON 在 115200 baud 下的传输耗时也计入实际刷新周期。退出/切换产品/关闭窗口会停机并释放串口。USB 丢失时，不靠上位机后台线程续心跳，固件自行停机。

## 可选板上诊断构建

仅 `CONFIG_KNOB_DIAGNOSTICS=y` 接受以下命令；正常镜像返回 `unknown_op`。

```json
{"id":10,"op":"diagnose","mode":1,"velocity":55,"current":0.15,"drive_ms":300,"observe_ms":3000}
{"id":11,"op":"diagnostic_read","offset":0}
```

`diagnose` 仅在 idle 且已校准时启动独立 FreeRTOS 任务。参数：mode=0–3，velocity 的绝对值 10–60 rad/s，current=0.02–0.25 A，drive_ms=50–500，observe_ms=500–3000。`current` 只限制驱动阶段的测试目标；释放阶段恢复当前配置的手感及运行电流上限。任务按当前电气参数和指定模式加速、释放，正常电流调节和所有硬件保护保留；速度超过 75 rad/s 中止。测试时间由电机控制任务限定，测试任务在该固定计划内续期租约，所以无需 USB 持续轮询；结束总是停机并恢复原配置，不保存 NVS。上电不自动测试。

诊断运行期间只接受主机 `status` 和 `disable`；后者取消计划且阻止排队的测试使能重新运行。普通配置/使能命令被拒绝。完成后 `diagnostic_read` 每次取最多 4 个样本，offset 为已有样本数，从 0 逐页读取。

可选 `diagnostic` 对象含 busy、cancelled、stage、count、peak_speed、peak_current、muted_samples、recovered。stage：0=空闲，1=准备，2=测试驱动，3=释放，4=完成，5=中止。recovered 表示释放阶段经历完整静音后力度恢复到 1；不等同于机械已完全静止。驱动阶段的测试电流用于模拟外部甩转，因此不受触觉静音系数影响，仍受电流限制约束。

样本为数组：`[elapsed_us, angle, velocity, target, iq, id_current, haptic_scale, voltage_q, phase_peak, flags, velocity_fast]`，间隔约 2 ms，容量 2048。flags 的低 8 位为 stage，bit 8 为 haptic_paused，bit 9 为 speed_limited。电流方向与普通遥测相同。取回样本仍使用原协议身份、请求 ID 和 4096 字节帧上限。
