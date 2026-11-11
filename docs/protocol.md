# SmartKnob USB 协议 v1

当前固件为 **`dma-v22-independent-detents`**，协议仍为 v1，沿用 `20k65_sampling`、20 kHz PWM 和 5 kHz 控制。2026-11-05 的 5 kHz / 90% 与 10 kHz / 85% 占空比实验已按用户要求回滚，详见[实验与最终恢复记录](duty90-20261105.md)。采样检查工具按设备 `control_nominal_hz` 的 98% 设置默认有效频率门槛，并验证实际锁存 duty；失败后停止后续电流阶梯。

## 2026-11-11 v22：物理单位与独立点击开关

`reference_config_version:2`、`reference_haptic_model:"esp32_detent_v2"` 标识新版参数语义。沿用模式 4–15 的独立完整条目补丁、停机应用、只读默认值查询和显式 save。省略整个参数库或某个模式时仍保留设备现有值。相同参数重发不重置正在运行的模式。

相对下方 v21 表格，改变如下；其余几何字段和范围保持：

| 字段 | v22 定义 |
| --- | --- |
| `strength` | 档位弹力系数，0–1000 A/rad |
| `damping` | 档位内部速度阻尼，0–0.5 A·s/rad |
| `endstop` | 端点弹力系数，0–20 A/rad，不再乘 4 |
| `endstop_damping` | 新增必填数值，端点速度阻尼，0–0.5 A·s/rad |
| `click_enabled` | 新增必填布尔值，决定是否自动产生切档脉冲；默认 false |
| `click` | 0–0.45 A，仅在点击开启时使用；关闭不清空幅值；模式 14 也可启用，限进入吸附档位时触发 |
| `scale` | 当前模式的弹力/阻尼倍率，0–1，默认 1；摩擦补偿与点击电流不乘此值 |

中心死区仍为 `min(width*0.2, 1°)`，档位电流与原模式共用 `strength*有效角误差 - damping*角速度`，静止死区归零；端点没有中心死区，采用完整误差和独立端点阻尼。点击必须完成两段后才能接受下一次切档，快速跨档不重启第一段。合成点击/摩擦后按运行电流限幅，并沿用原档位的高速正功衰减。`reference_current_limit:4.5` 为硬件能力上限，实际由 `config.current_limit` 进一步限制；不再额外限制为 0.45 A。

保存写独立版本/CRC 记录 `reference_v2`。v2 缺失时才尝试有效的旧 `reference_v1`：逐条与原始预设比较，未修改条目使用新版默认值；自定义条目保留几何，`strength=旧strength*旧scale`，`damping=旧damping*旧scale`，`endstop=4*旧endstop*旧scale`，端点阻尼取转换后的旧阻尼，最后 `scale=1`。阻尼/端点分别限制至 0.5 / 20，点击保留幅值但置 false。旧记录无效时使用默认库，迁移不自动写 Flash。仅 v2 记录损坏时回退新版默认，不复活旧库。电气配置、校准和无摩擦记录布局不变。

旧 v21 上位机不能按新含义写参数，需要配套新版 cf-gui；新版上位机仍按版本显示和验证 v1。请求/回复容量分别保持 2047 / 32768 字节。算法、点击计时、保护全部由 ESP32 执行，未增加主机心跳依赖。

## 2026-11-07 v21：每模式参数库（历史契约）

`reference_config_version:1` 表示模式 4–15 支持独立参数。正常状态在 `config.reference_modes` 返回全部 12 项，以模式编号字符串为键。`reference_current_limit` 报告保留的普通参考模式电流限制 0.45 A；公共 `config.current_limit` 仍可进一步限制输出。

`configure.config` 仍携带完整公共配置；可选的 `reference_modes` **只包含需要修改的模式**，每个列出的模式必须带齐下面所有字段。省略的模式保持原值；空对象无变化。任何字段或模式非法都拒绝整个请求，不部分应用。仅实际发生变化的参考参数要求 `state=idle`，否则返回 `disable_before_reference_config`；同值重发不重置运行状态。仅切换 `config.mode` 沿用现有行为。

| 字段 | 含义与范围 |
| --- | --- |
| `bounded` | 布尔值；是否启用行程边界 |
| `position` | 重新启用/归零时的初始逻辑档位，整数 −10000–10000 |
| `min_position`, `max_position` | 边界整数 −10000–10000；bounded=true 时须 min ≤ position ≤ max；false 时忽略排序，旧无界默认保留 0/−1 |
| `width` | 档距，rad；1°–180° |
| `strength`, `damping` | 参考控制器缩放前的档位 P 增益 0–100、速度 D 增益 0–5 |
| `endstop` | 缩放前端点增益，0–20；控制器使用其 4 倍 |
| `scale` | 输出倍率，0–1；作用于 P/D/端点项 |
| `snap`, `bias` | 以档距为单位的切换阈值与中心偏置；snap 0.5–2，bias 0–1.5 且 bias ≤ snap−0.05 |
| `friction`, `click` | 摩擦补偿与点击电流，A，0–0.45；磁性模式不使用点击项 |
| `magnetic_positions` | 模式 14 最多 8 个互异整数档位；有界时须在边界内。其他模式必须为空数组 |

参考控制器先计算 `scale × clamp(gain × position_error − damping × velocity, −10, 10)`，再加入摩擦和点击项；端点处 gain 为 `4 × endstop`，其余为 strength。最终仍受 0.45 A、用户电流上限及速度/故障保护约束。增益本身不能直接当作输出安培数。默认参数保持 v20 参考手感；Custom 的零力矩默认值现在也可编辑。

默认值使用只读请求 `{"id":1,"op":"status","reference_defaults":true}` 获取，回复追加完整 `reference_defaults` 参数库；不会应用、保存或改变当前模式。该布尔参数仅接受在 status 上使用。上位机“恢复默认”应先装入草稿，再由用户明确应用。

显式 save 将全部参数库写入独立版本/CRC 记录 `reference_v1`，保持原 `settings_v1`、校准与 `frictionless_v1` 布局兼容；旧 NVS 缺少该记录或版本/CRC/参数非法时回退到固件默认库。上电仍为 idle。通信请求正文上限仍为 2047 字节，cf-gui 保留 1900 字节限额以容纳请求 id，通常一次只编辑一个模式。完整回复接收上限扩到 **32768 字节**，默认库仅按需传输。任务栈为 control 9216 / console 24576 字节，控制器仍在独立高优先级任务中运行。

完整状态配置含 12 项参数库，不能直接整对象回传为 configure。仓库中的旧参数测试脚本和 `knobctl --mode` 使用 `common_configuration()` 复制公共/无摩擦参数并省略 `reference_modes`，使设备保留现有参数库；测试备份与最终完整配置比较仍含参数库。`knobctl --config` 中用户显式提供的单模式补丁保持原样，超长请求仍报错。定时脚本原有的结束停机与 RAM 恢复顺序不变。

## 2026-11-06 v20：参考预设与无摩擦模式（历史基线）

`config.mode` 接受 0–16。原 0=free、1=detents、2=bounded、3=spring 不变；4–16 对应 RollerCAN 参考预设 0–12，**16 为无摩擦**。`modes` 字符串数组的下标就是 ESP32 编号，`mode_name` 和 `reference_preset` 提供当前名称和参考编号（旧模式为 -1）。新增普通预设使用参考的固定几何/手感，旧 width/strength/damping 等仅用于原模式，current_limit 继续约束所有模式；普通参考预设合成 pulse 后仍不超过 0.45 A。

完整 `configure.config` 可附加 `frictionless:{friction,viscous,scale,hold_kp,hold_ki}`，提供该对象时五项必须齐全；遗漏则保留当前值，兼容旧客户端。单位分别为 A、A/(rad/s)、无量纲、A/(rad/s)、A/rad。范围依次是 0–0.8、0–0.02、0–10、0–1、0–5。`scale=0` 为持续零 Iq 对照。参数随显式 save 写入独立 CRC 记录 `frictionless_v1`，原 settings_v1 的 56 字节 Config 与校准布局不变。上电仍停机。

顶层 `frictionless` 遥测含 phase（0静止、1诊断观察、2保持、3让步、4起拨获取）、reference（rad/s）、correction 和 feedforward（A），angle_compensation=false。无摩擦静止时继续零 Iq 闭环，因此 quiet_output=false；pulse 在此模式不产生点击。25–40 rad/s 原速度包络、限流和故障保护保留。模式表、具体机制、参考移植差异与本轮实机结果见[运行模式记录](reference-modes-20261106.md)。

## 2026-11-06 v19：双向校准、质量记录与结果保留

每次 enable/calibrate 的零偏采集由历史 128 样本改为每相 4×128=512 个有效样本，检查均值、噪声、标准误和跨窗口漂移。校准再执行三相测量对应关系与极性检查、正反向各 29 点扫描、拟合及 6 个独立位置复核。默认初始电流 0.16 A、最高 0.25 A、电压上限 3 V，机械或拟合质量不足时有界重试；校准序列最多 120 秒，含零偏准备总计最多 121 秒。主机默认等 130 秒，以接收固件终态；固定的 0.2 秒 enable 等待已改为最多 2 秒的 active 状态轮询。

新增 `calibration_report` 区分本次尝试与已接受结果，`calibration_quality` 为当前结果保存质量证据。只有全部通过才替换基础校准和质量；取消/失败保留旧结果、关闭输出，故障不自动清除。成功结果先进入 RAM，显式停机 save 才写入原 `settings_v1` 和新增 `cal_quality` 记录。详细请求、字段及持久化语义见下文，实机结果和门限见[校准记录](calibration-20261106.md)。

完整状态扩展后主机应接受 **16384 字节**回复行。Console 栈同步由 8192 增至 16384 字节；`console_stack_free` / `control_stack_free` 报告对应任务的栈高水位剩余量。control_stack_free 只在电机未激励时重新读取，运行期返回上次缓存值，避免扫描控制任务栈干扰实时调度。新鲜 ADC 帧的重复任务通知不再清空校准窗口；仍跳过重复样本，保留过期/无效反馈和物理窗口保护。

## 2026-11-05 v16：转换完成校验与一致的两相采样（历史基础，v19 沿用）

该轮源码报告 `firmware:"dma-v16-coherent-adc"`，默认 `pwm_profile:"20k65_sampling"`，协议仍为 v1。20 kHz 中心对齐 PWM 的 duty 为 0–65%，零脉宽下限仍为 0；活动电流环的电压能力为 **`0.65 × 有效实测母线 / √3`**。12 V 时约 4.503 V，相比 v10 的 90% 能力降低约 27.8%。这是当前采样时序允许的配置，不是 SVPWM 数学极限。90% / 100% 仍为显式实验选项，客户端必须读取实际 profile 和能力字段。

默认 20 kHz 路径的 `current_sampling` 为 `pwm_staggered_adc_done_experimental`。SAR ADC 时钟为 5 MHz；实际 START 目标位于 TEZ 前 2.25 µs，低侧建立预算仍为 6 µs，上升沿保护仍为 1 µs。完成窗口改用与本次 START 关联的独立 ADC1_DONE ISR 观测。DMA 回调用于数据传输和阶段推进，不再当成本次转换的完成上界。DONE ISR 也包含中断延迟，仍是保守观测，并非模拟采样孔径。

| 字段 | v16 含义 |
|---|---|
| `adc_done_ticks[3]` / `adc_done_down_mask` | 每个 START 对应 DONE ISR 读取的 PWM 计数 / 向下计数位图；80 tick = 1 µs |
| `adc_done_elapsed_us[3]` | 从采样组首个 START 至各槽 DONE ISR 的观测时间，含中断延迟；各槽自身的延迟为该值减去 `adc_trigger_elapsed_us[i]` |
| `adc_done_mask` | 已捕获 DONE 的 START 槽位图；两份有效电流要求低两位齐全，第三槽负责冲刷 DMA 流水线，发布时不要求其 DONE 已到达 |
| `adc_done_raw_mask` | DMA 回调入口时 ADC1_DONE 原始标志的槽位图，仅用于诊断 DMA / DONE 的相对时序 |
| `adc_done_words[3]` | DONE ISR 读到的原始数据寄存器；仅低 12 位作为当前完成转换值，上位通道位可能已指向下一通道 |
| `adc_armed_words[3]` | 每次 START 前读到的寄存器，用其预装通道标签验证该 START 的测量相 |
| `adc_current_words[2]` | 两份经验证的有用 DMA 字：其低 12 位须等于对应 DONE 原始值，通道须等于该 START 前的 armed 通道；用于实际相电流和相窗口检查 |
| `control_interval_us` | 两次被控制环接受的反馈之间的实际时间，不截断遥测值；PI、滤波及手感包络使用 `min(实际间隔, 2 ms)`。拒绝的原始帧不再把下一次有效控制的 dt 重置成 200 µs，原始帧的过流/限流证据仍单独按原始帧时间处理 |

`adc_reject_counts` 沿用下方位定义；数据绑定不一致按传输完整性失败处理，所有物理窗口和跨相指令一致性检查继续生效。非零 PWM 指令若在第一相完成、第二相尚未完成时到达，会延后一个载波周期；电角度预测同时增加 50 µs。延后命令绑定预计生效序号，到期仍被采样占用或已经过期便丢弃，零输出不受此限制。`pwm_deferred` 只统计提交超时后等待下次控制重试的次数，成功入队的主动延后不增加它；`pwm_apply_delay_us` 可观察已接受命令的预计生效延迟。

电流参数、诊断 `target_d` / 固定参考系语义和 NVS 布局保持兼容。根因、预算推导、实机记录与验证边界见[采样时序修复记录](sampling-contract-20261105.md)。下方各版本的默认值与 DMA 完成假设属于历史记录。

## 2026-11-05 v10：90% 电压能力与固定参考系注流（历史）

v10 源码报告 `firmware:"dma-v10-windowed-svpwm"`，默认 `pwm_profile:"20k90_svpwm"`，协议仍为 v1。`pwm_duty_floor=0`、`pwm_duty_ceiling=0.9`，最低相钳零。独立的零脉宽下限策略避免降占空比时恢复历史配置的 4% 下限。

`max_voltage_limit`、`available_voltage_limit`、`effective_voltage_limit` 的当前默认能力均为 `0.9 × bus_voltage / √3`；有效母线 12 V 时约为 6.235383 V。`configured_max_voltage_limit` 采用同一比例和名义 `config.bus_voltage`，仅供参考。无效母线时可用能力为 0。`voltage_limit_automatic` 仍为 true，`config.voltage_limit` 仍被拒绝；比例是编译配置，不是新增的可写协议字段。

显式构建 `20k65_sampling` 时使用 `0.65 × bus_voltage / √3`、0–65% duty；`20k100_svpwm` 则为 `bus_voltage / √3`、0–100%。客户端应读取实际 profile、duty 和能力字段，不能只靠 firmware 字符串推断比例。占空比下调不替代实际相窗口检查，90% 也不保证高电流时采样持续有效。

顶层新增 `target_d`，表示经过限流后的 d 轴目标 A。普通手感的 `target` 继续使用原有机械方向 q 轴目标语义。诊断对象新增 `axis`（`"q"` / `"d"`）及 `reference`；d 轴测试为 `"fixed_start_angle"`，其 `id_current`、`iq`、`voltage_d`、`voltage_q` 对应该固定电角度，不能解释成转子发生偏移后的实时转子 dq 分量。固定参考系注流和仅恢复 RAM 校准的接口见文末“可选板上诊断构建”。

## 2026-11-05 v9 采样拒绝原因与跳过来源

v9 `firmware:"dma-v9-sampling-fix"` 保留了 v8 的调制、电压能力和协议版本，修复第一槽跨 TEZ 正常锁存新 PWM 指令时的误拒绝；v10 沿用该修复和以下计数字段。首槽旧下降沿与新上升沿仍分别检查；指令一致性检查为 `after[0] == before[1] == after[1]`，不再额外要求首槽 `before[0] == after[0]`。

`enable` / `calibrate` 进入零输出 offsets 时不再要求最新一帧物理有效；采样必须晚于重建边界、时间戳不在未来、年龄不超过 2 ms 且传输未失步。随后仍只消费有效样本，等待桥唤醒及足量合格零偏样本后才能进入 active/align；v9 当时为每相 128 份，v19 已扩展至每相 512 份和多窗口质量检查。这样避免一帧偶发迟到造成整个准备流程取消；持续无效反馈不会被接纳为零偏或电流。

`rejected_samples` 是 HAL 拒绝的采样组总数，既包含真实窗口不足，也包含软件时序/完整性判断；不能标成“ADC 硬件转换失败”。新增字段如下：

| 字段 | 含义 |
|---|---|
| `adc_reject_reasons` | 最近一组 HAL 样本的原因位掩码；通过 HAL 时为 0，位定义见下表 |
| `adc_reject_counts` | 按原因累计的对象；一组可命中多个原因，不能将各项求和作为总拒绝数 |
| `adc_timing_skips` | ADC 样本无效、过旧或长期未更新造成的控制跳过次数 |
| `encoder_timing_skips` | 编码器超过 2 ms 预测时限、尚未到 10 ms 丢失故障时限的控制等待次数 |
| `timing_skips` | 保留兼容，等于上述两类控制跳过次数之和；和 HAL 拒绝次数不是同一口径 |

| 位值 / `adc_reject_counts` 键 | HAL 拒绝原因 |
|---|---|
| 1 / `transport` | 无法完成传输、帧长度/通道不合法、阶段所有权不一致、重复相或完成掩码不完整；不直接等同于 ADC 硬件损坏 |
| 2 / `trigger` | START 方向/时刻不合要求或启动时 SAR 忙 |
| 4 / `pwm_tracking` | 无法可靠关联实际锁存的 PWM 比较值或载波序号 |
| 8 / `burst_timing` | 回调时 SAR 忙、回调间隔过长，或采样组顺序/总时长不合要求 |
| 16 / `phase_window` | 实际测量相未通过固件的低侧建立/完成窗口预算，或完成位置无法证明窗口成立；不是 ADC 转换错误标志 |
| 32 / `pair_change` | 两次有效相采样之间 PWM 指令发生变化 |

HAL 原因计数和 `rejected_samples` 一样跨采样硬件重建保留、重启清零。`adc_sample_valid` 还考虑应用层重建后的样本时间边界，因此不能仅用 `adc_reject_reasons == 0` 推断应用层已经接受样本。比对计数时应使用同一固件运行区间的增量。验证工具 `tools/sampling_health_check.py` 同时检查拒绝比例、有效控制频率、故障及清理结果；`passed:true` 不代表高占空比或外部电流精度验证。

窗口规则目前使用 6 µs 软件建立预算和 DMA 回调时刻作为完成上界。前者不是本轮实测的模拟建立时间，后者包含软件中断延迟，不是 ADC 采样孔径；因此这些检查有保守性。`phase_window` 表示无法按当前规则接受这组测量，不能单独证明 ADC 数据物理错误。未经边界波形和电流可信度验证，不能仅将该位清零后接受反馈。

## 2026-11-05 v8 自动最大电压与时序容错（历史默认）

`firmware:"dma-v8-full-svpwm"` 默认报告 `pwm_profile:"20k100_svpwm"`，线性 SVPWM 使用 0–100% 软件占空比范围。`config` 不再包含 `voltage_limit`；完整配置也不需要该字段。旧客户端提交该字段时返回 `ok:false,error:"voltage_limit_automatic"`，配置和校准保持原值。客户端应删除电压上限输入控件，按只读 `effective_voltage_limit` 显示当前能力。

| 字段 | v8/v9 全范围配置含义（当前默认 20k65 能力见上方 v16/v19 说明） |
|---|---|
| `voltage_limit_automatic` | true；电压能力由使用中的母线与调制范围计算，没有用户设定值 |
| `pwm_duty_floor` / `pwm_duty_ceiling` | 默认完整范围为 0 / 1；实际指令仍由电流闭环计算 |
| `max_voltage_limit` / `available_voltage_limit` / `effective_voltage_limit` | 默认完整 SVPWM 时均为 `bus_voltage / √3`，有效母线 12 V 对应约 6.928 V；无效母线时为 0 |
| `configured_max_voltage_limit` | 兼容保留的名义 `config.bus_voltage` 对应能力，仅供参考，不限制实测母线下的输出 |
| `timing_skips` | 因样本窗口、时效或调度条件不适用而跳过控制更新的累计次数，包括编码器短时过期的暂停周期 |
| `sampling_recoveries` | 连续无效 ADC 反馈或编码器短时过期触发零电压矢量恢复的累计次数；桥保持使能，有效反馈恢复后自动继续闭环 |
| `pwm_deferred` | 未赶上 PWM 安全提交时刻而延后本次输出更新的累计次数 |
| `pwm_peak_duty` | 从启动或采样硬件重建以来，HAL 按实际 TEZ 锁存的比较值统计到的最高占空比；不是尚未应用的 PI 指令 |
| `pwm_full_duty_cycles` | 同一统计期间内至少一相实际锁存为 100% 比较值的 PWM 周期数；不能替代驱动器输出波形测量 |

本工程默认启用已接线的 GPIO11 母线采样，每次控制使用当前有效实测值，最大电压能力随供电升降。关闭采样的构建只能使用 `config.bus_voltage`，无法检测真实供电波动。v8–v16 的历史校准使用最多 0.8 V 的独立对齐限制；v19 使用 calibrate 请求的 voltage，默认上限 3 V，并受实时能力限制。电压字段都是软件 dq 指令幅值，不是电机端电压外部测量；100% PWM 不等于 dq 电压幅值等于母线电压。

采样按两个实际测量相各自的低侧窗口校验，第三相由电流和为零重建。窗口不足的反馈不进入 PI，不清除既有过流暴露证据。偶发窗口或控制周期波动不再锁存 `adc_window_or_control_deadline`；连续无效超过约 2 ms 时请求零矢量恢复采样，保持桥使能，恢复后继续调节。这种恢复可能降低持续难采样角度下的平均输出，不能把 0–100% 指令范围解释为已保证连续 100% 实际输出。默认 20 kHz ADC 路径保留迟到回调所属 START，等待并完成原突发，超过 10 ms 仍未完成才标记失步并报告 `adc_transport_failure`；真实过流、无效/过期母线、编码器异常和控制任务真正停滞仍保留保护。

旧 NVS 中原电压字段的字节保留用于布局与 CRC 兼容，但不参与校验和调制；其值不会重新成为限幅，也不会导致旧校准被丢弃。以下版本变更与实机记录保留历史语境，旧版本电压参数不适用于 v8。

编码器样本年龄超过 2 ms 时同样暂停控制并请求零矢量，收到新鲜帧后继续；超过 10 ms 仍报告 `encoder_stale`。传输、CRC、磁场或角度跳变错误保留各自故障，不能作为普通调度波动忽略。

## 2026-11-04 v7 强度上限扩展（历史）

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

`config.bus_voltage` 继续表示名义母线配置，范围 4.5–30 V；启用母线采样后，实际调制使用顶层 `bus_voltage`，配置不会被每次 ADC 读数覆盖，也不会自动写入 NVS。以下母线字段延续到 v8，电压能力已改为自动最大值。

| 字段 | 含义 |
|---|---|
| `bus_voltage_sensing` | 是否编译启用 GPIO11 母线采样；v8 本工程针对已确认接线默认开启 |
| `bus_voltage_source` | 启用时为 `adc2_gpio11`；关闭时为 `configured` |
| `bus_voltage` | 当前调制所使用的母线 V；采样开启且读数无效时为 0，不回退到配置值 |
| `bus_voltage_valid` | 最新采样有效、读取成功、时间戳有效且年龄不超过 100 ms；关闭采样时表示配置母线有效 |
| `bus_voltage_age_us` | ADC 样本年龄，μs；尚无时间戳时为 1000000000；仅在采样启用时解释为测量年龄 |
| `bus_voltage_measured` | 最新 ADC 换算的母线 V；故障时可保留越界读数，只能结合 valid/error 解释 |
| `bus_voltage_raw` / `bus_voltage_adc_mv` | GPIO11 原始 12 位 ADC 码 / 校准后的引脚 mV；不是相电流 ADC 数据 |
| `bus_voltage_error` | 最近读取或初始化的 ESP-IDF 错误码，0 表示成功；无效范围使用 `ESP_ERR_INVALID_RESPONSE` |
| `bus_voltage_divider` | 分压换算倍率，默认约 11；由 `CONFIG_KNOB_BUS_DIVIDER_MILLI / 1000` 决定 |
| `max_voltage_limit` / `available_voltage_limit` | 按实际使用母线计算的只读电压能力；当前默认 20k65 为 `0.65 × bus_voltage / √3`，完整 SVPWM 配置为 `bus_voltage / √3`，无效母线时为 0 |
| `configured_max_voltage_limit` | 按名义配置母线计算的参考能力，不是可设置或实际使用的限幅 |
| `effective_voltage_limit` | 等于 `available_voltage_limit`；active 电流环的电压矢量限制；v19 校准再受请求 voltage 限制（默认 3 V） |

v8 取消手动电压上限，电源下降或上升时，可用上限与 PI 抗饱和限制均跟随变化。电压字段描述控制与调制能力，不是电机端电压的外部测量。PWM 频率和边界以当前设备报告的 `pwm_hz`、`pwm_profile`、`pwm_duty_floor`、`pwm_duty_ceiling` 为准。

母线任务在 core 0 每 10 ms 读取 ADC2，不在电流 ISR 或电机任务中等待转换。ADC1 电流 DMA 不加入母线通道。开启采样后，初始化/读取错误、ADC 饱和、换算值低于 4.5 V 或高于 30 V 均不可用于驱动；运行时锁存 `bus_voltage_invalid`，样本年龄超过 100 ms 时锁存 `bus_voltage_stale`。尚未获得有效母线的 `enable` / `calibrate` 返回 `bus_voltage_not_ready`。停止后显式 `clear` 在母线仍无效时请求工作任务重新初始化，并清除旧快照；应轮询到新的 `bus_voltage_valid:true` 后再显式使能。读数恢复本身不清故障，也不自动重新启动。

USB 发送改为每次最多 **512 字节**，修复扩展诊断回复超过 4096 字节时整段提交受 TX 环形缓冲区单项大小限制的问题。分块保留部分写入处理，整段 JSON 正文共用 100 ms 截止时间；线上格式和协议版本不变，客户端按 LF 拼接一条最长 16384 字节的回复，而不是把 512 字节当成报文边界。相电流绝对尺度仍待外部标定，新增母线读数不代表负载电流已完成验证。

## 传输与请求

默认使用 DevKitC 的原生 USB Serial/JTAG（板上 USB 口，GPIO19/20）；主机统一以 115200、8N1、无流控打开串口，原生 USB 不受该波特率限制。编译时可选 COM/UART0，此时 115200 为实际线路速率。两者由 `CONFIG_KNOB_CONSOLE_USB` / `CONFIG_KNOB_CONSOLE_UART` 互斥选择，USB 能枚举或输出日志不能证明其已启用调参。每条请求/回复是一个 UTF-8 JSON 对象，以 LF 结束。请求最大 2047 字节；v21 主机接受最多 32768 字节的一行。超长帧丢弃到 LF，不把尾部当成新命令。日志行不属于协议；默认 `CONFIG_ESP_CONSOLE_NONE=y` 避免应用日志与 JSON 混流。回复前后均带 LF，主机忽略空行与启动日志。一次请求对应一次回复，无主动推送。USB TX 支持部分写入并限制等待时间，后续帧前的 LF 可恢复被中断的帧边界。

```json
{"id":1,"op":"status"}
```

每次回复包含 `device:"smartknob-s3"`、`protocol:1`、相同的 `id`、`ok`，以及当前状态/参数/校准/遥测。拒绝请求时额外含 `error`；解析或校验失败也查询真实快照，但不会续期心跳。控制任务不可用时会返回 `control_unavailable`，附带的默认快照不可作为真实测量。主机必须校验身份、协议和请求 ID，不能因为任意串口返回 JSON 就认作旋钮。

可选字段 `transport` 为 `usb_serial_jtag` 或 `uart0`，用于显示设备实际使用的调参通道，不改变协议版本。编码器另由 `encoder`、`encoder_interface`、`encoder_resolution_bits`、`encoder_dma` 标识，默认值分别为 `MT6701`、`ssi_spi_dma`、14、true；切换源码中的 AS5600 后端时分别为 `AS5600`、`i2c`、12、false。旧上位机可以忽略新增字段，但不能继续把所有 `magnet` 数值当作 AS5600 STATUS。连接初期可在约 3 秒内重试只读 `status` 以等待固件启动；运行期请求维持短超时，不自动重发使能、校准等动作。`control_unavailable` 的默认快照必须作为通信失败处理，不得展示为真实 idle 状态。

| op | 语义 |
|---|---|
| `status` | 查询全部快照，仅在启用 `KNOB_HOST_LEASE_MS` 时续期通讯租约 |
| `configure` | 完整替换 `config`；先整体验证，失败时不修改任何参数 |
| `calibrate` | 仅 idle 可开始：每相 512 样本零偏、三相对应关系/极性检查、正反向各 29 点、拟合和 6 点复核；允许有界升流重试；全部通过才发布结果并返回 idle，取消/失败保留旧结果 |
| `enable` | 必须已经校准且 idle；每相重采 512 个有效零偏样本并通过质量检查后启动手感 |
| `disable` | 立即释放电机；取消正在进行的校准并保留原已接受结果；已有 fault 保留 |
| `clear` | 停机状态下清除锁存故障；初始化失败时重试硬件初始化 |
| `zero` | 在当前位置重设逻辑档位中心，不改编码器 OTP 或电角度校准 |
| `pulse` | active 时播放 3 ms 正向 + 3 ms 反向小电流脉冲，受电流限制和高速触觉衰减约束 |
| `save` | 仅停机时，写入带版本/CRC 及编码器身份的 NVS 配置、校准和独立质量记录；写入尝试后重建采样并返回新快照，保留已有故障，不保存或恢复 enabled 状态 |

## 校准请求与质量结果（v19）

```json
{"id":3,"op":"calibrate"}
{"id":4,"op":"calibrate","current":0.16,"max_current":0.25,"voltage":3.0}
```

三个参数都是可选的顶层字段，省略时分别使用 0.16 A / 0.25 A / 3.0 V。请求必须满足 `0.02 ≤ current ≤ max_current ≤ 0.25` A 和 `0.05 ≤ voltage ≤ 3.0` V，max_current 不得超过当前 `config.current_limit`。只修改本次校准计划，不改 config。非数值/非有限参数返回 `calibration_parameters`；电流顺序或范围错误返回 `calibration_current_range`，电压越界返回 `calibration_voltage_range`，超运行电流预算返回 `calibration_exceeds_current_limit`，非 idle 返回 `idle_required`。

正反向分别以 `j × π/2`、j=0…28 扫描 7 圈电角度，对本电机约一机械圈；保留连续角度，使用所有点拟合方向与极对数，以圆周平均求零点。极对数相对 7 的误差须 ≤3%，双向零点差 ≤0.12 rad、最大回差 ≤0.30 rad、残差 RMS ≤0.12 rad、最大残差 ≤0.35 rad；6 个非拟合位置的最大复核误差 ≤0.20 rad，均为电角度。电流跟踪、停稳和反馈有效性须同时满足。拟合数值不是替换固定整数 7 的运行极对数，而是质量证据。

机械停稳或回差/方向偏差/残差失败时，电流可乘以 1.5、封顶 max_current 后重试，最多 3 次且共用序列 120 秒上限。电流跟踪、相检查、电压饱和及反馈异常不触发升流重试。含 offsets 的固件总上限为 121 秒；主机工具等待默认 130 秒，仅增加接收终态的余量。`calibration_check.py --firmware-defaults` 明确发送裸请求；不带该标志时，它显式发送主机默认参数，不能替代固件默认值测试。

`calibration_report.stage` 正常路径为 `offsets → phase_check → align → forward → reverse → evaluate → align → verify → complete`，启动后尚未运行时为 `not_run`，退出还可能为 `failed` / `cancelled`。顶层 state 保持旧枚举：正反向扫描使用 sweep，拟合/复核使用 settle，其余激励阶段使用 align。主机应同时检查 idle、`calibration_report.passed:true`、`calibration_quality.verified:true` 和零输出；旧 `calibration.valid:true` 在新尝试期间仍可保留，不能单独作为成功判据。

| 对象/字段 | 含义 |
|---|---|
| `calibration_quality.method` / `verified` | 当前已接受结果的方法和证据有效标志：`bidirectional_v1` / true，或 `legacy_or_unverified` / false |
| `calibration_report.method` / `passed` / `cancelled` / `error` | 最近一次尝试的方法、通过/取消状态及原因；不随保存持久化，重启为 not_run |
| `calibration_report.attempts` / `point` / `elapsed_s` | 当前尝试次数、阶段内点序号、校准已用秒数 |
| `current` / `voltage_limit` | quality/report 共有；该次激励实际选用的电流 A 与计划电压幅值上限 V，不是外部实测电压 |
| `forward_points` / `reverse_points` / `verification_points` | quality/report 共有；通过时分别为 29 / 29 / 6 |
| `pole_pairs` / `forward_pole_pairs` / `reverse_pole_pairs` | quality/report 共有；拟合极对数及两个方向的独立拟合值 |
| `direction_difference` / `hysteresis_max` / `residual_rms` / `residual_max` / `verification_max` | quality/report 共有；双向零点差、最大回差、残差 RMS/最大值和复核最大误差，电角度 rad |
| `phase_check_passed` | quality/report 共有；仅使用独立采到的相电流完成对应关系/极性检查；不代表绝对感测增益已标定 |
| `offset_noise_a[3]` / `offset_sem_a[3]` / `offset_drift_a[3]` | quality/report 共有；该次零偏噪声、名义均值标准误、扣除随机均值不确定度后的漂移，A |
| `calibration_report.zero_forward` / `zero_reverse` | 两方向独立求得的电角度零点 rad |
| `calibration_report.observed_d` / `observed_q` / `tracking_error` / `saturation_fraction` | 最近统计窗口的指令定子角坐标电流 A、跟踪误差 A、饱和样本比例；可解释等待/失败原因 |
| `calibration_report.phase_a_response[3]` / `phase_b_response[3]` / `phase_c_response[3]` | 各相激励下的三通道独立采样均值 A；不采用重建相充当测量证据 |

取消或失败时，旧 calibration 和 calibration_quality 保持不变；report 则记录新尝试的取消或失败。失败锁存 fault，disable 不清除，显式 clear 才尝试清除；不能自动清除后重新使能。新结果只在 RAM 中生效，直到停机 save。

NVS 的 `smartknob/settings_v1` 基础配置/校准布局保留，另以 `cal_quality` 存放版本/CRC、完整基础校准、编码器身份、感测增益及质量指标。加载时须完整匹配且数值合格才恢复 verified；质量记录缺失、损坏或绑定不匹配时，基础校准仍按原校验恢复，但质量为 legacy_or_unverified。两记录保存不是突然断电下的原子事务。该质量证据不替代外部电流增益、机械转矩或绝对角度标定。

## 完整配置示例

```json
{"id":2,"op":"configure","config":{"bus_voltage":19.4,"sense_gain":0.15,"current_limit":0.8,"current_kp":0.4,"current_ki":200,"width":0.1745329252,"strength":20,"endstop":8,"damping":0.003,"snap":0.55,"min_position":0,"max_position":11,"mode":1}}
```

所有数值必须有限；NaN/Infinity、缺字段、整数越界均拒绝。

| 字段 | 单位和范围 |
|---|---|
| bus_voltage | V，4.5–30；名义配置，ADC2 开启时实际调制使用有效实测值 |
| sense_gain | V/A，只支持 0.15/0.3/0.6/1.2 |
| current_limit | A，0.02–4.5；默认 0.8，运行目标上限，实测越界先收减扭矩 |
| current_kp / current_ki | V/A，0–20；V/(A·s)，0–1000 |
| width | rad，1°–180°；页面以度编辑 |
| strength / endstop | A/rad，0–1000 / 0–20；默认 20 / 8；v6/v7 档位 strength 为中心弹力死区外的斜率，目标受 current_limit 限制；弹簧和端点语义保留 |
| damping | A·s/rad，0–0.5；v6 普通挡位在整个挡距使用速度阻尼，中心静止门限见下文 |
| snap | 档距倍数，0.5–1；默认 0.55；偏离当前中心严格超过此倍数时换挡，并在同一周期按新中心计算弹力 |
| min_position / max_position | 有序整数，±10000 |
| mode | 0=自由，1=档位，2=有限行程，3=弹簧；4–16 为参考预设，16=无摩擦，映射见上方 v20 |

原 0–3 模式的手感参数允许在运行期间修改；参考模式 4–15 的独立参数按 v21 节要求先停机。电气参数也需要先停机。原模式只调整强度、阻尼、切换点和电流限制时保留档位中心与电流环积分；模式、档距或行程边界改变时在当前机械位置重建中心，逻辑位置为 clamp(0, min_position, max_position)。参考模式使用其独立配置的初始 position。被状态机拒绝的请求（例如运行中重复 enable）也不会续期通讯看门狗。显式修改配置母线/增益会清除电角度有效标志，要求重新校准；正常 ADC2 母线更新不修改配置，也不使校准失效。

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

这些诊断值与最新采样一起发布，不能仅凭一份低速 `status` 推断所有控制周期的行为。诊断读取仍每次最多返回 4 条记录，完整 JSON 回复须小于当前客户端 32768 字节行上限（该历史实验最初为 16384 字节）；追加字段会增加串口传输时间。

`reset_reason` 是 ESP-IDF 复位原因枚举，`uptime_ms` 是本次启动时间，`minimum_free_heap` 是启动后最小剩余堆内存字节数；这些字段用于区分端口异常、设备复位和资源问题。

`app_elf_sha256` 为 `esp_app_get_description()` 中完整的 32 字节 ELF SHA-256，以 64 个小写十六进制字符输出。它用于将运行设备与本地应用镜像描述符对应；它不是 `.bin` 文件整体的 SHA-256，也不受启动日志摘要显示长度限制。`firmware` 字符串说明实验系列，唯一构建核对使用此摘要。

## 遥测

- `state`：idle/offsets/align/sweep/settle/active/fault；命令已接受与校准已完成是两个不同事件，应继续轮询状态。
- `angle` / `velocity`：连续机械角度 rad / rad/s；`position` 和 `sub_position` 是逻辑档位及其小数偏移。`velocity` 采用 10 ms 滤波用于高速包络；`velocity_fast` 采用 0.5 ms 滤波用于阻尼和弹性回程限速，减少反转时的控制滞后。
- `id_current` / `iq`：电角度坐标下的 A；只有 `current_valid:true` 时代表有效电流。普通运行使用转子电角度，校准使用指令定子角，固定参考系诊断另见 diagnostic.reference。休眠/校准零点/无效采样期间不得把保留值绘成实时曲线。`target` 是编码器方向的目标 A。作普通运行电流曲线时 q 轴目标为 `target * calibration.direction`。
- `currents`：重建后的 U/V/W A；每次取两相，第三相由和为零重建，感测相对按轮换更新。
- `sense_volts` / `offsets`：三个通道最近电压及零点 V。
- 顶层 `offset_noise_a` / `offset_sem_a` / `offset_drift_a`：最近一次 enable/calibrate 零偏采集的质量，A；与 calibration_quality 内绑定已接受校准的那次零偏数据分开。
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
- `calibration_quality` / `calibration_report`：分别为已接受结果的质量证据和最近一次尝试，详见“校准请求与质量结果”。取消或失败只更新后者，不能据后者覆盖仍被保留的已接受结果。

设备不自动重连、不自动清除或恢复锁存故障。正常 active 会话中的高速触觉衰减会按上述阈值自动恢复；这不解除任何锁存故障。上位机串行查询快照，收到完整回复后再等待下一次查询；轮询频率只影响显示刷新，不是触觉控制周期。

默认 `KNOB_HOST_LEASE_MS=0`，USB 丢失、停止查询、上位机切换页面或关闭窗口都不会单独触发停机；设备保持已明确启用的手感。cf-gui 的智能旋钮工作台将“断开连接”与“停止电机”分开：断开只释放串口，只有显式停止操作发送 `disable`。重新连接只读取设备当前状态，不回放本地配置、不自动使能、清故障或保存。固件的上电状态仍为 idle；独立运行不等于上电自动驱动。

触觉状态机、无摩擦保持、限流和故障保护均由 ESP32 电机任务执行，USB 控制台通过队列提交离散命令。定时测试脚本可在其明确的测试生命周期结束时主动 `disable`，这不是固件对普通上位机的存活要求。需要通讯租约的产品可在构建时设置正数毫秒值，此时仅成功接受的命令续期，超时锁存 `host_timeout`；主机应按实际 `host_lease_ms` 显示该设备的行为，不能假定任何构建均可离线持续运行。

## 可选板上诊断构建

仅 `CONFIG_KNOB_DIAGNOSTICS=y` 接受以下命令；正常镜像返回 `unknown_op`。

```json
{"id":10,"op":"diagnose","mode":1,"velocity":55,"current":0.15,"drive_ms":300,"observe_ms":3000}
{"id":11,"op":"diagnostic_read","offset":0}
{"id":12,"op":"diagnostic_hold","current":0.15}
{"id":13,"op":"diagnose","axis":"d","mode":0,"velocity":0,"current":0.3,"drive_ms":500,"observe_ms":500}
```

`diagnose` 仅在 idle 且已校准时启动独立 FreeRTOS 任务。省略 `axis` 或指定 `"q"` 时执行加速/释放：mode=0–16，velocity 的绝对值 1–60 rad/s，current=0.02–0.25 A，drive_ms=50–3000，observe_ms=500–10000。驱动采用独立速度 PI（Kp=0.015、Ki=0.04），前 50 ms 爬升目标；无摩擦保持控制器在这段时间只观察，释放时获取实测速度。`current` 只限制驱动阶段的测试目标；释放阶段恢复当前配置的手感及运行电流上限。正常电流调节和所有硬件保护保留；速度超过 75 rad/s 中止。总期限由电机控制任务在 ADC 有效性判断前检查，采样失败不会延长驱动；测试任务在固定计划内续期租约，无需 USB 持续轮询。结束停机并恢复原 RAM 配置，不保存 NVS。上电不自动测试。

`axis:"d"` 选择 v10 固定参考系注流。`mode` 和 `velocity` 可以省略，默认 0；显式给出 velocity 时必须为 0。`current` 范围为 **0.05–3 A**，且不得超过当前 `config.current_limit`；`drive_ms=50–30000`，`observe_ms=100–1000`。请求电流先经过约 200 ms 的平滑爬升，再由正常限流器和 PI 调节；drive_ms 小于 200 时会在爬升完成前结束。`target_d` 报告限流后的实际请求，配置或请求为 3 A 不表示绕组已达到 3 A。

d 轴测试在正式驱动开始时记录转子电角度，并固定该参考系，不随转子继续旋转；同时将该参考系的 q 轴目标设为零。这样产生静止定子磁场，允许空载转子在运动界限内趋于稳定。它不重新校准，也不等同于已经完成真实机械堵转或扭矩标定。机械位移超过起点 0.15 rad、速度绝对值超过 5 rad/s 或运动值非有限时，中止并报告 `diagnostic_motion_guard`。

d 轴驱动期满后立即请求零电压，观察阶段继续保持零输出；总期限到达后失能停机。期限与运动界限由电机任务在 ADC 有效性判断之外检查，采样失败不会延长注流；普通过流、母线和编码器检查仍保留。`diagnostic.reference:"fixed_start_angle"` 表明此时 Id/Iq 和 d/q 电压采用测试开始的固定参考系。正常 q 轴诊断的 reference 为 `"rotor"`。

诊断运行期间只接受主机 `status` 和 `disable`；后者取消计划且阻止排队的测试使能重新运行。普通配置/使能命令被拒绝。完成后 `diagnostic_read` 每次取最多 4 个样本，offset 为已有样本数，从 0 逐页读取。

可选 `diagnostic` 对象含 busy、cancelled、axis、reference、stage、count、peak_speed、peak_current、muted_samples、recovered。stage：0=空闲，1=准备，2=测试驱动，3=释放，4=完成，5=中止，6=普通手拨记录中，7=手拨记录已冻结。recovered 表示 q 轴释放阶段经历完整静音后力度恢复到 1；不等同于机械已完全静止，也不是 d 轴诊断的电流跟踪验收值。q 轴驱动阶段的测试电流用于模拟外部甩转，因此不受触觉静音系数影响，仍受电流限制约束。

样本的前 16 列保持：`[elapsed_us, angle, velocity, target, iq, id_current, haptic_scale, voltage_q, phase_peak, flags, velocity_fast, voltage_d, adc_raw_a, adc_raw_b, sample_start_tick, sample_end_tick]`，容量 2048。v2 在旧版 12 项数组末尾追加两相原始 ADC 码及采样窗口时刻；读取旧固件时应接受缺少末尾四项。flags 的低 8 位为 stage，bit 8 为 haptic_paused，bit 9 为 speed_limited，bits 16–17 / 18–19 为两次实际采样的相号 0/1/2。电流方向与普通遥测相同。取回样本使用原协议身份、请求 ID 和当前 32768 字节主机回复缓冲上限（请求正文最大 2047 字节）。

v10 d 轴诊断在该数组末尾追加第 17 项 `target_d`，v20 保留该布局。v20 q 轴诊断追加四项 `hold_phase, hold_reference, hold_correction, hold_feedforward`，共 20 列。两种轴的采集间隔均为 `max(200, ceil((drive_us + observe_us) / 2047))` µs，实际按有效控制 tick 向上取整，保证缓冲覆盖长释放尾段；有效采样不足时记录更稀疏。USB 快照均值和抽取记录不是完整 5 kHz 电流纹波。

历史短驱动的全频率记录已改用上述统一周期；普通 enable 的环形记录仍按完整控制频率采集。

`diagnostic_hold` 是有载斜坡排查用的短时目标保持，仅在 active、有有效电流、未运行其他诊断且速度低于 2 rad/s 时接受。`current` 使用机械方向，幅值 0.04–0.25 A，不得超过运行电流上限，符号须与当时目标一致。它保持请求目标最多 750 ms，仍经过正常速度包络、限流、PI 和故障保护。超过起点四分之一挡宽（上限 0.05 rad）或速度达到 5 rad/s 会提前停止。期间只接受 status/disable，不能续期；结束自动失能并冻结普通环形记录，flags bit 11 标识保持样本。该命令不保存配置，不在上电时启动；主机应使用保持前稳定目标的均值，避免负载变化混淆比较。750 ms 保持后的 5 kHz 环形记录只覆盖保持稳态，不包含切换起点。

诊断构建的普通 enable 会复用记录缓冲，以完整控制频率滚动记录最近 2048 个样本（约 410 ms）；disable 或故障后冻结，可通过 diagnostic_read 取出，记录中读取会返回 disable_before_trace_read。手拨记录 flags 的 bit 10 表示 quiet_output，bits 16–17 / 18–19 为两次实际采样的相号 0/1/2。`tools/exercise.py --capture` 在停止后自动保存 `.trace.json`，用于排查 USB 低速查询看不到的快速电流变化。正常构建没有此缓冲或自动记录。

`diagnostic_restore_calibration` 仅在诊断构建、idle 且无正在执行的诊断时接受，用于将同一设备烧录前备份的校准恢复到 RAM。请求包含 `encoder` 字符串和完整 `calibration` 对象；编码器名称必须等于当前构建名称，valid 必须为 true，direction 为 ±1，pole_pairs 为 7，electrical_zero 为 `[0, 2π)` 内有限数。身份不符返回 `calibration_identity`，字段不合法返回 `invalid_calibration`，非 idle 返回 `idle_required`。调用只恢复 RAM 基础校准，并清除 calibration_quality 的验证标记；不使能、不改配置、不写 NVS。

例如，使用已经读回并保存的完整状态 `backup`：

```python
device.request("diagnostic_restore_calibration",
               encoder=backup["encoder"], calibration=backup["calibration"])
```

该接口不验证备份是否属于当前物理转子或磁铁安装位置；只有在安装与接线未变化时，原备份才仍有相同含义。普通运行配置仍通过停机后的完整 `configure` 恢复；`tools/stall_window_check.py` 本身不生成新校准，退出时核对配置和校准与进入时一致。
