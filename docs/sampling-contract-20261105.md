# 2026-11-05：高电流采样失败的根因与采样时序约束

本文对应 `dma-v16-coherent-adc`、默认 `20k65_sampling`。最终硬件短测及 8 轮各 30 秒的长测均通过采样健康门槛，反馈恢复为 0；高请求时实际电流仍受可用电压限制，未达到 3 A。旧的 [90% 对照记录](stall-window-20261105.md)保留为历史证据，其中“默认 90%”“仅使用 DMA 回调边界”等描述不再代表本版实现。

## 结论与已复现现象

大电流请求下的失采确实与可用低侧窗口有关，但把最大电压从 100% 改成 `0.9 × Vbus / √3` 不足以解决。现有板级建立预算与 20 kHz PWM 下的 90% 占空比不相容；与此同时，代码还把 DMA 交付事件当成当前转换的完成时刻，没有正确计入 ADC 的两级分频，并允许一次较慢的控制计算在两次测相之间更新 PWM。这些问题需要分别修正。

本轮在原 `dma-v10-windowed-svpwm / 20k90_svpwm` 上取得了新的 [baseline90.json](../build/sampling-contract-20261105/baseline90.json)，实际复现了频繁的反馈恢复：

| 固定参考系 d 轴请求 | 驱动时间 | 驱动快照区间拒绝率 | 有效控制频率 | 后半程有效快照 Id 均值 | 整轮反馈恢复增量 |
|---|---:|---:|---:|---:|---:|
| 0.3 A | 3 s | 0.963% | 4951.5 Hz | 0.2944 A | 0 |
| 0.8 A | 3 s | 60.491% | 1975.6 Hz | 0.6755 A | 0 |
| 1.5 A | 3 s | 64.300% | 1784.9 Hz | 0.6963 A | 0 |
| 3 A | 10 s | 90.588% | 470.6 Hz | 0.6147 A | 3980 |

3 A 档驱动快照首末之间为 9.924 s，有效控制 4670 次、拒绝 44945 组、恢复 3964 次；3980 是包含准备和结束阶段的整轮恢复增量。该档后半程只有 9 份有效电流快照，均值不代表所有控制周期的连续电流。请求 3 A 不等于实际输出了 3 A。主要拒绝原因是 `phase_window`，驱动区间计数为 44941；同一组可以同时命中多个原因，原因计数不能直接相加。

持续约 2 ms 没有可信反馈时，固件请求零输出并复位 PI，随后等待有效采样恢复。这条路径会造成短暂失力；本轮恢复计数证明它确实反复发生。它不等于已经测到了电源掉电，也不等于 MCU 重启或 DRV8316C 自主保护，后者仍有下文所述观测限制。

## 1. 由采样约束推导输出上限

DRV8316C 的电流输出依赖对应低侧导通。TI 给出的 CSA 建立时间条件不能替代板上 PWM 传播、模拟链路和 ADC 的完整预算；本版继续保留 6 µs 板级建立预算及 1 µs 上升沿保护，不以典型参数直接缩短它们。[TI DRV8316C 数据手册](https://www.ti.com/lit/ds/symlink/drv8316c.pdf)

20 kHz 的载波周期为 50 µs，计数时钟为 80 MHz，中心对齐半周期为 2000 ticks。对于单相 duty，TEZ 附近低侧窗口半宽为：

```text
compare = 2000 × (1 − duty)
H = compare / 80 µs = 25 × (1 − duty) µs
START ≥ −H + 6 µs
DONE 观察时刻 < H − 1 µs
```

| 单相 duty | 低侧总窗口 | 最早合法 START | 最晚合法完成观察 |
|---|---:|---:|---:|
| 90% | 5 µs | +3.5 µs | < +1.5 µs |
| 70% | 15 µs | −1.5 µs | < +6.5 µs |
| 65% | 17.5 µs | −2.75 µs | < +7.75 µs |

90% 处连开始与结束约束都无法同时满足。只要轮换到这种高占空比相，该组就可能不可用；SVPWM 的理论电压圆并不保证低侧采样窗口。动态选相也不能单独保证所有电角度下都有两相满足当前窗口，仍需联合安排调制、实际测相和触发时刻。

本版将约束放入共享的 [sampling_timing.hpp](../components/knob_core/include/sampling_timing.hpp)：START 目标为 −180 ticks，即 −2.25 µs；完成观察设计预算为 +560 ticks，即 +7 µs；另保留 40 ticks，即 0.5 µs 的余量。由两侧要求取较大值：

```text
下降沿要求 = 480 + 180 = 660 ticks
上升沿要求 = 560 + 80 + 1 = 641 ticks
最小 compare = max(660, 641) + 40 = 700 ticks
默认 duty 上限 = (2000 − 700) / 2000 = 0.65
```

其中 `+1` 对应运行时严格的小于条件。+7 µs 是设计预算，并不声称 ISR 每次必定在此时刻前执行；运行时仍使用每槽真实 START、DONE 观察和前后锁存比较值重新检查。上限、触发目标、窗口默认参数共享常量，并有编译期断言，避免以后单独修改某项而破坏约束。

因此默认 PI 电压圆变为 `sqrt(Vd² + Vq²) ≤ 0.65 × 实测 Vbus / √3`，PWM 保持最低相钳零、0% 下限。相对此前 0.9 的电压系数，最大可用电压减少约 27.8%；这是实际输出能力取舍，不能描述成仅损失很小余量。90% 和 100% 配置保留为显式实验选项，不提供每个测相组合均可采的保证。电流请求上限与这个电压约束是不同参数；没有把电机重新固定到 0.8 A。

## 2. 分开记录实际转换完成与 DMA 交付

仅降低到 65% 的早期候选仍有约 5% 拒绝，见 [candidate-short.json](../build/sampling-contract-20261105/candidate-short.json)。814 个前两槽回调观察中有 50 个明显迟到，其中 47 个集中在 20 个载波周期中的同一相位；这与 1 ms 周期性调度干扰相符，但单凭此统计不能证明具体是哪一个 ISR。[回调相位统计](../build/sampling-contract-20261105/candidate-short-callback-phase.json)

进一步探测发现，同槽 DMA 回调甚至可能早于当前 ADC1 转换完成，不能仅把它解释为完成后的软件延迟。原始 [idle-done-raw.json](../build/sampling-contract-20261105/idle-done-raw.json) 的 40 份快照中，DMA 入口处当前 DONE 原始状态均未置位；当时回调约在 TEZ 后 7.5 µs，而 DONE 观察约在 +13 µs。DMA 会交付流水线中的结果，不能凭回调序号就将数据及完成时间归给当前 START。官方连续 ADC 文档定义了 DMA 转换帧和带单元、通道、数据的结果结构；它不承诺本工程手动 START 适配器的当前槽归属。[ESP-IDF ADC continuous 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/peripherals/adc_continuous.html)

本地 ESP32-S3 TRM 第 39.3.7.1 节说明每次转换需要 25 个 SAR 时钟，SAR 时钟不超过 5 MHz；第 39.5 节定义 ADC1_DONE 为一次转换完成事件。[ESP32-S3 TRM](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)、[本地核验摘录](../build/sampling-contract-20261105/adc-controller-trm.txt)

默认 `CLKM=15`、`SAR=1` 都采用加一编码，所以真正的 SAR 时钟是：

```text
80 MHz / (15 + 1) / (1 + 1) = 2.5 MHz
25 / 2.5 MHz = 10 µs，仅转换本体已超过原来的完成假设
```

分频公式及默认值分别可核对 [Espressif bootloader 源码](https://github.com/espressif/esp-idf/blob/v5.5.6/components/bootloader_support/src/bootloader_random_esp32s3.c)和 [S3 ADC LL 源码](https://github.com/espressif/esp-idf/blob/v5.5.6/components/hal/esp32s3/include/hal/adc_ll.h)。本版在关闭自动触发后、预热扫描前设置 `CLKM=7`、`SAR=1`，得到 10 MHz 数字控制器和 5 MHz SAR，25 周期转换本体为 5 µs；启动时验证 APB 80 MHz 及实际时钟选择，编译期限制 SAR 不超过 5 MHz。母线测量采用 ADC2 独立 RTC oneshot 路径，此处不修改它的 RTC 时钟。

转换完成单独通过公开 `esp_intr_alloc()` 分配 `LEVEL3 | IRAM` 的 ADC1_DONE ISR，和 PWM 同级。ISR 只观察新鲜 DONE、记录一致的 PWM 计数与时间、保存结果寄存器及所属槽，不通知 FreeRTOS。每次 START 前仅清除 DONE bit31；重复 DONE、无所属槽、缺少所需 DONE 都不能作为有效反馈。预热完成后再启用该中断，停止 ADC 前先禁用、清除并释放。GDMA 仍由 IDF 驱动持有。[ESP-IDF 中断分配接口](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/system/intr_alloc.html)

`adc_done_ticks` 是 ISR 观察到完成事件的保守时间上界，不是硬件捕获时间，也不是实际模拟采样孔径。DMA 的触发、回调、帧数及耗时诊断继续保留。完成时刻还必须满足同槽、START 在合法 TEZ 前区间、观察不早于 START、经过时间小于 40 µs，以及两种非原子时钟读数之间最多 ±5 µs 的一致性检查。±5 µs 只用于元数据读数交叉检查；真实 PWM 窗口仍使用计数器时间，6 µs 建立及 1 µs 上升沿保护没有放宽。

`adc_done_elapsed_us[i]` 与 `adc_trigger_elapsed_us[i]` 都以整组第一槽 START 为原点；该槽 START 到 DONE 的时间差应将两者相减，不能直接把第二、三槽的 elapsed 值当作转换耗时。

## 3. 将完成时刻、ADC 数值与相位明确关联

增加 DONE 后不能直接把状态寄存器的高位当作当前转换通道。v14 曾采用这个假设，随后带电测试证伪：某组 DONE 寄存器为 `[26406, 34711]`，低 12 位为 `[1830, 1943]`，高位看似 A、B；对应有效 DMA 字为 `[42790, 26519]`，数值相同，但 DMA 标签是 C、A。DONE 时状态寄存器的高位已经反映下一个预装载通道。该候选触发 `diagnostic_motion_guard`，记录仍保留为失败：[final-short.json](../build/sampling-contract-20261105/final-short.json)。它不属于最终通过结果。

最终固定扫描表仍为 `3,4,5,4,5,3,5,3,4`，三次 START 位于连续载波，前两次提供测相结果，第三次用于流水线排空。在每次 START 前读取 `adc_armed_words`；在 DONE ISR 读取 `adc_done_words`。发布电流前逐槽同时要求：

1. 前两槽有属于各自 START 的新鲜 DONE，完整 DMA 帧及阶段所有权检查通过。
2. DMA 字为 ADC1、合法通道 3–5，没有非法保留位；两份有效结果来自不同相。
3. DMA 通道标签等于该槽 START 前状态寄存器的预装载通道标签。
4. DMA 低 12 位等于该槽 DONE 时状态寄存器低 12 位。
5. 用这一已关联相位和 DONE 观察时刻检查对应的真实低侧窗口。

通过后，`adc_current_words` 保存实际用于电流重建的 DMA 标签及数值；`adc_dma_words`、`adc_slot_words`、`adc_armed_words` 和 `adc_done_words` 各自保留诊断用途。任一关联条件失败都拒绝该组，不进入 PI。仅数值相等不能证明身份，尤其零输出和饱和码可能连续相同，所以还必须结合预 START 标签、固定扫描及独立阶段所有权。

预 START 标签关联是本芯片、本 SDK 和本手动扫描适配器上得到的实验依据，不应推广成所有 ADC 使用模式的公开保证。S3 LL 的 ADC1 读数接口只取低 12 位也与这种谨慎解释一致。[官方 ADC LL 实现](https://github.com/espressif/esp-idf/blob/v5.5.6/components/hal/esp32s3/include/hal/adc_ll.h)

v15 的 [idle-v15.json](../build/sampling-contract-20261105/idle-v15.json) 中 40/40 快照通过关联检查；桥关闭时数值处于饱和，不能据此验证模拟电流精度。[0.1 A 带电记录](../build/sampling-contract-20261105/final-short-v15.json)的后半程有效快照均值为 0.09980 A，说明修正后的相位关联可以跟踪该小电流请求；该轮仍因运动保护结束，并存在下一节的 PWM 对间变化，所以整份报告仍为失败，不能视作最终回归通过。

## 4. 两次测相之间的 PWM 必须属于同一指令

此前已经保留的合法条件是 `after[0] == before[1] == after[1]`：允许第一槽 TEZ 正常锁存新指令，但要求从第一槽之后到第二槽之后使用同一组 PWM 比较值。两个测量窗口还分别用真实下降沿和上升沿验证。[先前一致性修复](sampling-fix-20261105.md)

v15 小电流测试的约 3 s 区间中出现了数百个 `pair_change`，同时没有相窗口、传输或恢复异常；控制计算偶尔超过 100 µs，上一组反馈产生的输出可能在下一组第一槽之后写入影子寄存器，并于第二槽 TEZ 锁存。这是输出调度破坏测相一致性，不能通过放宽 `pair_change` 判定来接受。

v16 将 `adc_pending && adc_stage == 1` 定义为非零输出的受保护阶段，并在两种写入接口中统一处理：

| 相对第一槽 TEZ | 事件及允许的行为 |
|---:|---|
| −2.25 µs 附近 | 第一槽 START，进入 stage 1 |
| 0 µs | 第一槽 TEZ；其后的非零新命令只排队 |
| +25 µs | TEP；stage 1 期间不写入非零队列 |
| +47.75 µs 附近 | 第二槽 START，进入 stage 2；5 µs 提交保护仍禁止此时直接写入 |
| +50 µs | 第二槽 TEZ，原比较值保持；此后恢复正常直接提交 |
| +75 µs | TEP 可写入属于目标锁存代数的排队命令 |
| +100 µs | 排队命令锁存，不改变刚完成的有效采样对 |

相较通常的下一 TEZ 锁存，排队路径增加一整个 50 µs 载波。`write_voltage` 将这段实际延迟计入电角度预测；三角函数计算前捕获 TEZ 序列、待服务状态、计数器相位和是否需要延期，提交时全部复核，变化则重新计算。普通直接路径保持原有行为。[MCPWM 比较值更新机制](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/peripherals/mcpwm.html)

每个非零队列命令绑定 `当前 TEZ 序列 + 2` 的目标锁存代数。目标未到就等待；到达目标且保护已解除才写入；目标时仍受保护或已过期就丢弃，保持此前输出并由现有反馈恢复逻辑处理。这样 DMA 偶发延迟不能让带旧角度预测的电压在更晚载波应用。序列判断使用可处理 `uint32_t` 回绕的模运算。零命令不受采样保护和代数过期限制，仍通过安全锁存路径生效。

既有遥测 `pwm_deferred` 只统计 `write_voltage` 返回 `ESP_ERR_TIMEOUT` 后跳过提交的次数；成功进入上述计划队列不会增加它。因此该计数为 0 不能解释为从未使用计划延期。

## 5. PI 时间间隔只由实际接受的反馈推进

另一个独立问题是被拒绝原始帧也推进了原来的时间基准，下一次 PI 可能只看到一个名义采样间隔，遗漏它实际暂停的时间。现在原始帧时钟与 [ControlSampleClock](../components/knob_app/include/control_timing.hpp)分开：只有新鲜 ADC、编码器及控制准入检查全部通过后才推进 PI 时钟；第一次采用名义周期，后续按相邻两次接受样本的时间戳差计算。

遥测 `control_interval_us` 保留未截断的真实接受间隔；PI、反馈滤波及触觉包络使用的控制时间上限为 2 ms。控制会话或反馈恢复重置 PI 时也重置该时钟；quiet 输出分支仅清空积分，不需要重置连续接受样本的时钟。原始帧过流暴露及电流折返计时仍单独使用其原始帧时间基准。无效样本不更新 PI，也不会通过补算电流或插值伪装成有效采样。约 2 ms 连续无反馈的零输出恢复、过流保护和独立编码器时效检查保留。

## 验证状态与统计口径

最终源码桌面回归为 21/21 通过，见 [host-tests-v16-final.log](../build/sampling-contract-20261105/host-tests-v16-final.log)。测试包括采样窗口严格边界、所有有效相对组合、DONE 缺失与错周期、晚 DMA/正常 DONE、DMA 原始值与预 START 标签不匹配、采样对中途指令变化、延期角度预测、提交竞争、队列目标过期及序列回绕、PI 接受间隔。桌面测试不替代硬件验证。

最终短测的 0.3 / 0.8 / 1.5 A 各驱动 3 s，3 A 请求驱动 10 s；四档均正常完成且通过采样检查。[final-short-v16.json](../build/sampling-contract-20261105/final-short-v16.json)

| 短测请求 | 驱动区间拒绝率 | 有效控制频率 | 后半程 Id 均值 | 整轮恢复 |
|---|---:|---:|---:|---:|
| 0.3 A | 0% | 4999.7 Hz | 0.2968 A | 0 |
| 0.8 A | 0% | 5000.0 Hz | 0.5024 A | 0 |
| 1.5 A | 0% | 4999.7 Hz | 0.4920 A | 0 |
| 3 A | 0.02820% | 4998.2 Hz | 0.4856 A | 0 |

长测为 0.3 / 0.8 / 1.5 A 各 30 s，再做 5 轮 3 A 请求各 30 s，累计驱动 240 s，其中 3 A 请求累计 150 s；各轮之间有短暂停止观察，不能表述成连续 240 s 恒流输出。[final-long-v16.json](../build/sampling-contract-20261105/final-long-v16.json)

| 长测请求 / 轮次 | 驱动区间拒绝组数 | 驱动区间拒绝率 | 有效控制频率 | 后半程 Id 均值 |
|---|---:|---:|---:|---:|
| 0.3 A | 0 | 0% | 4999.93 Hz | 0.2997 A |
| 0.8 A | 3 | 0.002006% | 4999.93 Hz | 0.4718 A |
| 1.5 A | 1 | 0.000668% | 4999.93 Hz | 0.4736 A |
| 3 A，第 1 轮 | 1 | 0.000668% | 5000.00 Hz | 0.4652 A |
| 3 A，第 2 轮 | 1 | 0.000668% | 4999.90 Hz | 0.4609 A |
| 3 A，第 3 轮 | 0 | 0% | 5000.03 Hz | 0.4619 A |
| 3 A，第 4 轮 | 3 | 0.002001% | 4999.90 Hz | 0.4507 A |
| 3 A，第 5 轮 | 3 | 0.002003% | 5000.03 Hz | 0.4511 A |

长测整轮合计有效控制 1,225,585 次、拒绝 14 组，反馈恢复、传输异常、采样对指令变化及漏消费均为 0。14 组均包含 `phase_window`，其中 5 组兼有 `trigger`、1 组兼有 `pwm_tracking`；驱动首末快照区间合计拒绝 12 组，两种口径不能混用。约 5000 Hz 的微小上下浮动来自毫秒级统计时间边界。

短测与长测全部高于 0.3 A 的档位，后半程有效快照均达到软件电压饱和判据；没有达到 0.8、1.5 或 3 A 请求值。长测最大指令 duty 约 0.567，并未在所有电角度实测到单相 0.65 上限，报告的 `sampling_window_stressed` 为 false；固定参考方向下电压圆饱和可以先于某相 duty 达到上限。测试证明此次条件下的饱和运行不再伴随原来的大量失采和恢复，不能扩大成“所有角度满上限验证”或“3 A 持续电流验证”。

两份报告都为 `passed:true`、`cleanup_errors:[]`，各轮诊断正常结束；报告末尾均为 idle、fault=none、duty=[0,0,0]，RAM 配置和校准与各自连接时一致。

额外 [USB 回归](../build/sampling-contract-20261105/usb-final-v16.json)通过 7/7 项，包括分帧、流水请求、100 次状态读取、20 次重连及显式停止。[60 s 常规运行检查](../build/sampling-contract-20261105/health-final-v16.json)实测 60.030 s、1351 份快照、144 次内置短脉冲全部接受，有效控制 300152 次，拒绝 1 组，约 5000.03 Hz；传输异常、漏消费、反馈恢复和 `pair_change` 均为 0。该 1 组同时包含 trigger、pwm_tracking、phase_window，未将迟到观察伪装成成功。

[最终状态记录](../build/sampling-contract-20261105/final-state-v16.json)核对设备 ELF 摘要与本地最终镜像一致，并确认 idle、fault=none、duty=[0,0,0]；配置及校准逐字段恢复到本轮开始的 [before.json](../build/sampling-contract-20261105/before.json)，未保存 NVS。常规运行检查是低电流/运动回归，不能与前面的饱和注流测量混为同一种工况。

两份最终硬件报告的固件均为 `dma-v16-coherent-adc / 20k65_sampling`，读回 ELF SHA256 为 `7ede88f20e82f607478bc6444c225138e50af6ee2153c016314e39136a003467`。对应 [final-v16.bin](../build/sampling-contract-20261105/final-v16.bin)、[final-v16.elf](../build/sampling-contract-20261105/final-v16.elf)、[构建日志](../build/sampling-contract-20261105/build-v16-final.log)、[烧录日志](../build/sampling-contract-20261105/flash-v16.log)已归档。原始报告散列为：

```text
final-short-v16.json  0789f7586ed19db80fb6b6ba7a930850f63355b9146c5237eba3f9710a320865
final-long-v16.json   48905a99178d742006b9cf4617034aa9d4015d01e9f671db7b6cbbed16b90d51
```

衍生 [validation-summary.json](../build/sampling-contract-20261105/validation-summary.json)及图表需按文件和固件版本分组；前述旧版或失败候选的结果不计入 v16 通过结果。

统计定义如下：

- 驱动区间仅取 `diagnostic.stage == 2` 的首末 USB 快照，以设备 `uptime_ms` 和累计计数差计算。
- 拒绝率为 `Δrejected_samples / (Δcontrol_cycles + Δrejected_samples)`；漏消费另列，不能隐藏在该指标中。
- 有效控制频率为 `Δcontrol_cycles / Δ时间`，与名义 5000 Hz 区分。
- 电流均值只取后半程且 `current_valid` 的快照；无效快照是缺测，不按零或插值处理。有效快照可能有选择偏差。
- 整轮计数包含准备、驱动和观察；必须与驱动首末区间计数分列。诊断失败、运动保护、清理失败均保留原记录。

## 可复现方法

以下命令使用本工程 ESP-IDF/Python 环境及已连接的 COM7；`stall_window_check.py` 使用启动时固定的定子参考系 d 轴注流，包含固件斜坡、运动保护、硬截止时间及结束观察。它不等同于外部机械夹紧的标准堵转试验，也不是电流或转矩标定。工具临时改变 RAM 参数，结束时停止输出并恢复连接时配置及校准，不保存 NVS。每次使用新的输出路径，避免覆盖原始证据。

```powershell
cmake --build build/host
ctest --test-dir build/host --output-on-failure

python tools/stall_window_check.py --port COM7 --currents 0.3,0.8,1.5,3 --seconds 3 --final-seconds 10 --output build/commissioning/sampling-short-new.json
python tools/stall_window_check.py --port COM7 --currents 0.3,0.8,1.5,3,3,3,3,3 --seconds 30 --final-seconds 30 --output build/commissioning/sampling-long-new.json
```

工具默认门槛为驱动拒绝率低于 1%、有效控制高于 4900 Hz、无传输异常和反馈恢复，并要求诊断正常结束、清理完成。`passed:true` 只表示这些采样与运行条件满足；仍要单独检查 `requested_current_reached`、电压饱和、实际电流和完成时长，不能把 3 A 请求或采样检查通过当成持续输出 3 A 的证明。

## 仍有的观测限制

`nFAULT` 没有接入 MCU，DRV8316CT 为硬件引脚配置，现有串口 `fault:none` 不能排除驱动自主过流、欠压或温度保护。母线约 10 ms 一次的采样也不能排除短暂供电跌落。硬件接线与测量路径见 [hardware.md](hardware.md)。本轮已确认反馈恢复导致的失力路径，但不能仅凭软件修复宣称所有物理掉电原因都已排除。

电流由板上 ADC、芯片校准及当前传感增益估算，没有外部电流仪表的绝对校准；桥关闭的饱和读数不具有零电流精度含义。没有外部同步示波器直接观察 SO/PWM/母线/nFAULT，因此 6 µs 建立预算仍为设计约束，DONE 观察仍是软件上界。要进一步提高电压利用率，应重新验证模拟建立时间、转换速度、测相选择及调制时序的联合设计，不能只提高软件 duty 上限或删除窗口校验。
