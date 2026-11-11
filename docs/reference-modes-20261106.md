# 2026-11-06：参考运行模式与无摩擦控制

本轮将本地 `m5-rollercan_smart-knob` 的默认运行预设接入 ESP32-S3 工程。固件标识为 `dma-v20-reference-modes`，模式总数为 17。原有模式 0–3 保留，参考预设以独立的 4–16 编号追加；ESP32 的电流闭环、20 kHz PWM、5 kHz 控制、采样有效性检查及既有电角度校准继续使用本工程实现。

本记录前半部分描述源码行为和复现方法。末尾的实机验证部分待填入本轮板上原始记录；源码和离线测试不代表硬件通过。

## 模式编号与预设行为

| ESP32 `config.mode` | 参考预设编号 | 名称 | 几何与主要行为 |
| --- | --- | --- | --- |
| 0 | — | Free | 原自由模式，按 `-damping × velocity` 请求电流；`damping=0` 时运动中跟踪零 Iq |
| 1 | — | Detents | 原普通挡位，沿用现有 `width`、`strength`、`damping`、`snap` |
| 2 | — | Bounded | 原有限行程挡位，沿用现有 `min_position`、`max_position` 与端点配置 |
| 3 | — | Spring | 原弹簧回中，沿用现有弹力和回程阻尼 |
| 4 | 0 | Custom | 参考 Custom 的编译默认值：无界、10°、无连续弹力；本轮未增加参考 Custom 编辑接口 |
| 5 | 1 | Unbounded smooth | 无界、10°计数，包含参考固件的平滑摩擦电流项；与主动速度保持模式分开 |
| 6 | 2 | Bounded 0-10 | 位置 0–10、10°；区间内平滑，端点回拉 |
| 7 | 3 | Multi-rev smooth | 位置 0–72、10°，约两圈行程；增强端点 |
| 8 | 4 | On/off strong | 位置 0–1、60°，双稳态强挡位 |
| 9 | 5 | Return to center | 单中心、60°基准，回中及参考摩擦项 |
| 10 | 6 | Fine smooth | 位置 0–255、1°，从位置 127 开始，细分平滑计数 |
| 11 | 7 | Fine detents | 位置 0–255、1°，从位置 127 开始；参考默认 P=0，主要通过双相点击提供细挡反馈 |
| 12 | 8 | Coarse strong | 位置 0–31、10°，强挡位 |
| 13 | 9 | Coarse weak | 位置 0–31、10°，较弱连续弹力及双相点击 |
| 14 | 10 | Magnetic detents | 位置 0–31、7°，只在位置 2、10、21、22 启用磁性挡位弹力；端点仍有效 |
| 15 | 11 | Center detents | 位置 −6–6、60°，保留朝中心方向的换挡偏置 0.4 |
| 16 | 12 | Frictionless | 无弹簧、无端点，运动确认后施加平均损耗前馈和柔性速度保持 |

模式 4–15 的几何、P/D、换挡阈值、参考电流倍率、稀疏挡位及点击取自参考固件默认表。切入模式、显式 `zero` 或重新使能时，以当前位置建立参考中心，并使用表中起始逻辑位置。参考模式沿用其原计数方向：正机械角度增量使逻辑位置减小；原 0–3 的计数方向不改动。

参考预设普通输出先受 `min(config.current_limit, 0.45 A)` 限制，再进入本工程的共同速度包络、电流降额及故障保护。参考的点击为相反极性的两个 2 ms 电流段，合计 4 ms；它与手动 `pulse` 命令不同。

模式 4–15 不读取旧的 `config.width`、`strength`、`endstop`、`damping`、`snap`、`min_position`、`max_position` 来修改预设手感。这些字段继续保留、校验并用于原 0–3 模式；公共 `current_limit` 和电气参数仍生效。若需要调整普通预设的独立参数，应另行扩展预设配置接口，不能把旧配置字段的读回值误认为当前参考预设的有效几何。模式 16 使用下面的独立 `config.frictionless` 参数。

## 配置、状态与保存兼容性

`status` 新增以下字段：

| 字段 | 含义 |
| --- | --- |
| `mode_count` | 当前为 17 |
| `modes` | 以实际 ESP32 模式编号为数组下标的名称列表 |
| `mode_name` | 当前模式名称 |
| `reference_preset` | 参考预设编号 0–12；原模式返回 −1 |
| `config.frictionless` | 独立的无摩擦参数对象 |
| `frictionless.phase` | 0 静止、1 诊断运动观察、2 保持、3 让步、4 起拨获取 |
| `frictionless.reference` | 当前参考速度，rad/s |
| `frictionless.correction` | 保持器对平均前馈的纠偏电流，A |
| `frictionless.feedforward` | 限幅及限变化率后的平均损耗前馈，A |
| `frictionless.angle_compensation` | 当前固定为 `false` |

顶层 `frictionless` 是运行遥测，`config.frictionless` 是参数；两者用途不同。普通状态中的 `target` 经过速度包络和实测电流预算处理，可能小于前馈与纠偏之和。

`configure` 仍要求原完整配置对象。对象内可选提供 `frictionless`；若提供，必须完整包含 `friction`、`viscous`、`scale`、`hold_kp`、`hold_ki` 五个有限数值。遗漏整个对象时保留当前无摩擦参数，避免旧客户端的配置写入将新参数复位。参数变更只在 RAM 中生效；停机后显式 `save` 才写 NVS。

原 C++ `Config` 的 56 字节布局、`settings_v1` 配置/校准记录及校准质量记录继续保留。新参数单独保存到 `smartknob` 命名空间的 `frictionless_v1`，包含版本、参数和独立 CRC。旧设备没有此键、其长度/版本/CRC/参数无效时，使用无摩擦编译默认值，不以这些参数代替现有电角度校准。显式保存时一起写入配置、校准质量和无摩擦记录。

兼容范围是新固件读取既有记录，并保留 0–3 的原含义。旧固件不认识新保存的模式 4–16，不承诺旧版本能够回读新模式选择。

切换模式时应先读设备配置，在读回对象上修改模式，再完整下发。例如以下脚本只改变 RAM 模式选择并保持停机，不写 NVS：

```python
from tools.knobctl import Knob

device = Knob("COM7")
try:
    current = device.request("status")
    device.request("disable")
    config = current["config"]
    config["mode"] = 16
    print(device.request("configure", config)["mode_name"])
finally:
    device.close()
```

## 无摩擦参数和控制机制

以下默认值为当前源码起点，尚不代表本台 ESP32 电机完成损耗辨识。运行电流仍受公共 `config.current_limit`、本模式 0.8 A 上限及实测电流降额共同约束。

| 参数 | 默认值 | 允许范围 | 单位 |
| --- | --- | --- | --- |
| `friction` | 0.022 | 0–0.8 | A，平均运行摩擦等效电流 |
| `viscous` | 0.000304 | 0–0.02 | A/(rad/s)，黏性损耗系数 |
| `scale` | 1.0 | 0–10 | 无量纲前馈倍率；0 清除并关闭整个无摩擦保持输出 |
| `hold_kp` | 0.016 | 0–1 | A/(rad/s)，保持 PI 比例增益 |
| `hold_ki` | 0.024 | 0–5 | A/rad，保持 PI 积分增益 |

无摩擦控制器在本工程快速编码器速度上增加 8 ms 低通。速度不超过 0.2 rad/s 时，目标严格为零并清除旧速度目标；速度至少 0.6 rad/s 且持续确认 5 ms 后才能开始助力。停止或换向后等待 40 ms 才能重新使能；0.2–0.6 rad/s 之间使用平滑低速系数。

平均损耗模型使用速度绝对值 `v`：

```text
mean_loss = scale × taper(v) × [friction × (1 + 0.30 × exp(-v² / 9)) + viscous × v]
```

符号取当前运动方向。平均前馈独立限制到 ±60 mA，变化率不超过 1 A/s，并受当前可用电流预算约束。静止时不会凭该模型产生启动电流。

起拨进入有期限的获取阶段：至少观察 200 ms，速度趋势安静 80 ms 后获取目标，最迟 1.2 s 结束。保持阶段的参考固定，用有限 PI 修正模型残差；本台适配后，平均前馈之外的额外助力及反向电流最多 **60 mA**（参考原值 24 mA），负反馈可以撤销全部平均前馈。总请求仍受当前预算和独立 0.8 A 上限约束，这些是请求值，不代表实测持续电流。积分按剩余电流预算防饱和。明显运动变化或持续偏差/额外纠偏会触发让步，解除旧参考，随后获取新速度。此过程使用运动与电流估计，没有触摸传感器，不能把让步状态解释为真实触摸检测。

诊断驱动期间仅观察实际滤波速度；切换到释放阶段时获取测得速度，而不是把诊断请求速度直接设为正式保持目标。无效输入、超过 2 ms 的控制时间步、停机或故障均清除相关状态。实测限流器可能短暂将 q 轴可用预算降至零：连续不足 20 ms 时目标严格为零、参考和积分冻结，速度和停止/换向检测继续；连续达到 20 ms 才清目标。负数或非有限预算仍立即清状态，实际限流始终执行。

实机发现参考固件的手势门槛会在本电机相同机械角度反复误触发，让步后的参考逐次降低。本版将快速手势与慢拨误差判断改用已有 80 ms 速度趋势，并要求慢拨趋势同向远离参考至少 150 ms 后才接受原电流/误差确认；保持 PI 仍使用 8 ms 速度。回归覆盖正反方向、3/5/10/20/30 Hz、幅度 3 rad/s 的周期波动，不改参考；持续真实加减速仍能进入让步。这是本台硬件适配，不是参考控制器原封不动的行为。

模式 16 在零目标、极低速和静止时仍运行零 Iq 闭环，绕过原 `QuietOutput` 的零电压静止路径。这样避免把零目标直接等同于三相零电压短接。禁用输出和故障停机仍执行原停机路径。

ESP32 仍沿用共同速度包络：25–40 rad/s 渐弱，达到 40 rad/s 暂停，低于 5 rad/s 持续 30 ms 后，以约 500 ms 恢复。该规则同样作用于新预设和无摩擦模式；因此本版本没有复制参考固件后期“无摩擦模式不设保持速度上限”的行为。原过流、母线、编码器、采样及任务失效保护继续生效。

本轮不复制参考电机测得的每圈 7、12、14 次角度谐波，也不复制其虚拟惯量的转子参数。不同电机、磁铁零位和电流测量链不能共用这些标定。当前 `angle_compensation=false`，极低速局部齿槽及真实手感需结合本台硬件测量和人工体验评估。

## 自动验证与证据

`tools/frictionless_check.py` 通过正常 USB 协议做一次试验。驱动/释放需要启用 `CONFIG_KNOB_DIAGNOSTICS`；纯静置检查也可用于生产构建。工具支持模式 16 和模式 0 对照，`--current` 指诊断驱动预算；`--runtime-current` 指正式运行/释放预算，省略时等于 `--current`。测试期间将 `current_limit` 临时设为运行预算，不超过连接时的运行电流设置。

```powershell
$python = 'D:\rj\Espressif\v532\python_env\idf5.5_py3.10_env\Scripts\python.exe'

# 无外部刺激的静置检查，结束关闭输出。
& $python tools/frictionless_check.py --port COM7 --rest-seconds 5 --current .1 --runtime-current .8 --output build/modes-20261106/rest.json

# 正反向驱动到初速，再由无摩擦控制器接管。
& $python tools/frictionless_check.py --port COM7 --velocity 10 --current .1 --runtime-current .8 --drive-ms 2000 --observe-ms 5000 --require-hold --output build/modes-20261106/hold-pos.json
& $python tools/frictionless_check.py --port COM7 --velocity -10 --current .1 --runtime-current .8 --drive-ms 2000 --observe-ms 5000 --require-hold --output build/modes-20261106/hold-neg.json

# 相同刺激，关闭无摩擦补偿，始终零 Iq；不要求它保持速度。
& $python tools/frictionless_check.py --port COM7 --mode 16 --scale 0 --velocity 10 --current .1 --runtime-current .8 --drive-ms 2000 --observe-ms 5000 --output build/modes-20261106/zero-iq-pos.json

# 仅本次覆盖平均损耗参数，退出恢复原完整配置。
& $python tools/frictionless_check.py --port COM7 --velocity 6 --current .1 --friction .02 --viscous .0003 --scale 1 --output build/modes-20261106/tuning-pos.json
```

工具范围为请求速度幅度 1–30 rad/s、诊断驱动预算 0.02–0.15 A、运行预算 0.02–0.8 A、驱动 50–3000 ms、观察 500–10000 ms。驱动预算不能超过运行预算。`--rest-seconds` 范围 0–30 秒；非零时不发送驱动刺激。固件 q 轴诊断协议自身允许速度幅度 1–60 rad/s、刺激预算最高 0.25 A；仍由电机任务执行截止时间和测试速度保护，主机超时不替代板端停止期限。

每次试验先备份完整配置、校准和质量记录，退出时先 `disable`，等待诊断监督任务清理，再恢复原 RAM 配置并核对 `idle / fault:none / duty=[0,0,0]`。工具不 `save`、不重校准、不清故障。保存路径必须尚不存在，避免新试验覆盖旧失败记录。

JSON 报告分别给出：

- `passed`：执行完整、初速有效、轨迹完整、采样条件满足且恢复核对通过。默认不包含速度保持评分；加 `--require-hold` 才同时要求 `hold_passed`。
- `hold_passed`：初始释放速度达到指定同向下限，尾段保持同向，释放期不停止/反向，首尾速度差不超过 `max(0.5 rad/s, 初速 × 20%)`，观察至少 1 秒。模式 16 还要求末尾 500 ms 保持阶段为 2。静置检查不宣称保持通过。
- `trace`：首/尾释放平均速度、漂移、位移、驱动末段速度、命令电流与 Iq 均值、跟踪偏差，以及保持参考/纠偏/前馈。
- `sampling`：只统计 USB 观察到的实际 active 窗口，驱动/释放测试从首次 active stage 2 到末次 active stage 3，静置检查从首个 active 快照到最后一个 active 快照。要求窗口覆盖至少 85% 的请求运行时间且中间状态连续 active，拒帧率低于 1%，传输、跨相指令变化、反馈恢复、重复通知及漏处理增量均为零。报告首末快照、索引、覆盖比例、未观察时长和窗外计数差；该结论只覆盖观察窗口，不代表首尾未观察的通电边沿。
- `sampling_full`：仍保留准备前到诊断监督清理后的全部计数与原严格判据。全程的传输、跨相指令变化、反馈恢复增量和低于 1% 拒帧率继续参与总验收，不能通过裁掉边沿隐藏真实恢复。`reused_samples` 实际是已被跳过的重复通知计数，`missed_samples` 是跨序号计数；二者在 idle 也可能增长，不能仅凭闲置端点归因于 active。全部轮询保存在 `status_samples`；旧记录未保留中间快照的，不能事后改判通过。
- `rest`：角度跨度不超过 0.05 rad、峰值速度不超过 0.3 rad/s、严格零目标、全程 active；模式 16 还要求 `quiet_output=false`。

诊断 q 轴波形沿用前 16 列，随后追加 `hold_phase`、`hold_reference`、`hold_correction`、`hold_feedforward`；d 轴原有第 17 列 `target_d` 保留。波形取自主控任务捕获，均值按捕获时间加权，USB 状态轮询不作为均匀高速采样。

## 本轮实机结果

设备为 COM7、MAC `1c:29:04:38:1e:b8`、MT6701 / 22008，实测母线约 11.5 V。所有驱动/释放试验均为台架空载；运行预算 0.8 A、诊断驱动上限 0.1 A（早期低预算对照除外），均沿用 Kp=3、Ki=60、7 极对和原电角度校准 0.2927631735801697 rad。下表速度由板端抽取轨迹按时间加权，初段为释放后 250 ms、尾段为最后 500 ms；请求初速不等于释放瞬间速度。

| 试验与原始记录 | 初段→尾段 rad/s | 结果 |
|---|---:|---|
| [正向 10 秒，60 mA 额外预算](../build/modes-20261106/final-pos10-long.json) | 9.9525→9.6057 | 保持和采样通过；参考固定 9.7013，位移 96.558 rad；活动频率约 4973.85 Hz，拒帧 0.51309%，传输/配对/反馈恢复/重复通知/漏处理增量 0 |
| [反向 10 秒，60 mA 额外预算](../build/modes-20261106/final-neg10-long.json) | −10.4704→−10.0026 | 保持和采样通过；参考固定 −10.1088，位移 −100.095 rad；活动频率约 4998.57 Hz，拒帧 0.03188%，上述异常增量 0 |
| [正向 5 rad/s](../build/modes-20261106/final-pos5.json) | 4.7949→0.0009 | 保持失败，采样通过；参考固定 4.9987，未误触发让步，约 13.63 rad 后停下 |
| [反向 5 rad/s](../build/modes-20261106/final-neg5.json) | −4.0578→0.0002 | 保持失败，采样通过；参考固定 −3.8770，约 3.05 rad 后停下 |
| [额外 100 mA 对照](../build/modes-20261106/extra100-pos5.json) | 1.2341→−0.0002 | 未解决正向低速停转，也未达到规定初速；已恢复 60 mA，不采用该试验值 |
| [临时平均摩擦 40 mA](../build/modes-20261106/hold-pos5-fc40.json) | 6.1281→5.9564 | 单次保持评分通过，但参考从 5.6554 改为 6.4415，且拒帧率接近 1%；不作为固定参考或模型正确的证据，未采用该参数 |

正反零 Iq 对照 [zeroiq-pos](../build/modes-20261106/zeroiq-pos.json) / [zeroiq-neg](../build/modes-20261106/zeroiq-neg.json) 在驱动末段平均约 ±10 rad/s，释放后约 1.16 / 1.20 rad 行程内停下。旧版工具把准备/停机时的重复通知和跨序号计数算入严格全程判断，因此两个原报告仍为失败；缺少当时完整 active 快照，未事后重判。新工具明确区分 active 观察窗口与全区间数据。

低速停转并非通过放宽停止条件解决：正向末次阻力位置约为机械角 π，最后 30 ms 捕获均值中，目标约 82 mA、Iq 约 56 mA，纠偏达到 +60 mA；电压远未饱和。更大的请求上限仍未解决，说明局部阻力与电流跟踪动态需要进一步针对本电机处理。此处电流均值来自约 3.6 ms 抽取轨迹，感测绝对精度未另用仪器标定，不能当作完整 5 kHz 电流纹波或真实机械转矩标定。

早期失败全部保留：原移植的瞬时零预算会反复清目标；修复后又发现让步锁定在相同机械角度，参考逐次下移，才引入趋势判断。首轮静置的 0.3 rad/s 峰速门槛失败，以及一次 `offset_mean_drift` 启动拒绝，也保留在 `rest-first.json`、`hold-pos-first.json`。仅对明确零偏故障执行一次 clear 后重测，没有删除采样或校准门限。

## 正式镜像、持久化与最终状态

最终关闭 `CONFIG_KNOB_DIAGNOSTICS` 的镜像位于 `build/modes-20261106/idf-prod/`，设备报告的 `app_elf_sha256` 为 **`3df03821168db271f12213e38c23bf5f36e72b75b7c4348c94def00d37f612b4`**。状态不含 diagnostic，`diagnose` 返回 unknown_op。早期文件名带 production 的构建仍继承了本地诊断选项，不能按文件名认定正式版本；正式检查使用 `production-final-*`，由报告中的实际字段和 SHA 区分。

正式镜像的 [USB 7 项检查](../build/modes-20261106/production-final-usb.json)、[17 模式及 15 项非法参数/兼容性检查](../build/modes-20261106/production-final-modes.json) 均通过。模式检查是在低电流下验证切换、遥测、限流与静态运行，实际手拨跨挡质感需人工体验；离线回归验证了双相点击、稀疏磁吸、端点、偏置和换挡几何。

[持久化检查](../build/modes-20261106/persistence.json) 临时将 mode=16、scale=0.9 保存并硬复位，验证新编号和独立参数恢复；随后恢复初始 mode=1 与 scale=1，再次 save、硬复位并核对完整配置、校准和 verified 质量记录。最终 RAM 单独选择 mode=16 供体验，未再次保存模式选择，重启仍恢复原 mode=1；上电不会自动使能。

[正式静置 10 秒](../build/modes-20261106/production-final-rest.json) 全程目标为零、持续零 Iq 闭环、无故障，但角度跨度 0.006903 rad（约 0.396°）、速度峰值 0.3457 rad/s 超过严格 0.3 门槛，因此静置检查不通过，不用诊断版本静置通过覆盖该结果。[原档位模式 60 秒采样检查](../build/modes-20261106/production-final-sampling.json) 完成 145 次已接受短脉冲，有效控制约 4999.33 Hz、32 组拒帧（0.01066%），其中 4 次 pair_change 导致严格不通过；无故障、传输错误、重复/漏处理或反馈恢复。既有立即零输出保护可能打断配对的路径仍保留，本轮没有放宽该验收门槛。

完整逐次汇总见 [validation-summary.json](../build/modes-20261106/validation-summary.json)，最后状态见 [final-status.json](../build/modes-20261106/final-status.json)。最终无摩擦模式 60 秒采样记录单独保存在 [production-frictionless-sampling.json](../build/modes-20261106/production-frictionless-sampling.json)，其普通 pulse 请求会被接受但不叠加点击，因此这是零目标运行采样检查，不能当作持续外力扰动验证。

这轮无摩擦检查在 **60.032 秒通过**：有效控制 4999.58 Hz，18 组拒帧（0.005997%）；传输、pair_change、重复通知、漏处理和反馈恢复增量均为零，pwm_deferred=7 表示错过提交时刻后等待下次有效控制。1320 份快照的目标全部为零、保持阶段全部为静止，角度跨度 0.003835 rad、峰速 0.1460 rad/s。此次较长观察符合静置边界，但不抹去前一轮严格失败。最终状态为 **mode=16 / idle / fault:none / duty=[0,0,0]**，原配置除 RAM 模式外逐字段相同，校准和 verified 质量记录完整相同；镜像文件散列和逐字段核对见 [completion.json](../build/modes-20261106/completion.json)。

原设备应用分区和 NVS 分别备份为 `previous-device-app.bin`、`previous-nvs.bin`；均通过 ROM 无 stub 读取，应用镜像校验有效。旧构建目录的 `previous-v19-app.bin/.elf` 也保留，但回退设备镜像应优先使用实际读出的应用备份。整个过程仅烧录应用分区，未重校准或改写编码器内部配置。

离线工具验证已完成：`python tests/frictionless_check_tests.py` 的 20 项测试通过，覆盖正反向评分、停转不算保持、截断/无效波形拒绝、配置回复丢失后的恢复、故障保留、静置路径、运行/驱动预算分离，以及只发生在停机边沿的通知计数保留与真正 active 漏帧继续拒绝。它们只验证主机检查工具，不替代电机实测。
