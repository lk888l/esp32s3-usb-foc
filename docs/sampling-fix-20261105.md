# 2026-11-05 电流采样误拒绝修复

## 结论与原因

正常空载运行的大量 `rejected_samples` 主要来自软件一致性条件误拒绝，并非 ADC 转换器频繁失效，也不以达到 100% 占空比为前提。原实现要求第一槽 START 前的 PWM 比较值等于跨 TEZ 锁存的新比较值；正常的控制更新因此也会被拒绝。

第一相采样的旧下降沿、新上升沿分别通过低侧窗口检查时，测量期间该相一直 LOW。此时 TEZ 锁存新比较值不等于产生实际开关沿；两次有效采样之间的载波应按新的比较值检查。修复将 `before[0] == after[0] == before[1] == after[1]` 改成 `after[0] == before[1] == after[1]`，保留每相真实窗口、完整性、时效和两次有效采样之间的指令检查。

另修复使能/校准入口的单帧有效性门：进入零输出 offsets 只要求传输未失步、样本晚于采样重建边界、时间戳不在未来且年龄不超过 2 ms。偶发无效帧不会阻止开始零偏采集。控制任务仍只消费有效样本；桥唤醒、每相至少 128 份零偏样本及均值/方差检查都继续依赖有效反馈，之后才可进入 active/align。旧的母线、故障、编码器和校准准入条件保留。

## 修改前后对比

设备为 COM7，MT6701，20 kHz PWM / 5 kHz 名义控制，空载。基线 ELF 为 `fef96a6576ee0048d2550a1aadb7bb763dc330dc8ca386c80ffcec33f7e9e69f`。原参数为电流上限 0.8 A、Kp=3、Ki=60、mode=1、strength=20、damping=0.003，电角度零点 6.0791654587 rad。两轮均以原参数发送周期性固件内置短脉冲，剔除最初 10 份快照，以设备 uptime 和累计计数差计算。

| 项目 | 修改前 | 仅一致性修复及诊断计数 |
|---|---:|---:|
| 统计时间 | 11.651 s | 11.689 s |
| 有效控制次数 | 40,119 | 58,374 |
| 拒绝采样组 | 18,138 | 69 |
| 拒绝率 | 31.1345% | 0.1181% |
| 有效控制频率 | 3443.40 Hz | 4993.93 Hz |
| 漏消费 / 反馈恢复 | 0 / 0 | 0 / 0 |
| 新增 100% PWM 周期 | 0 | 0 |

拒绝率定义为 `Δrejected_samples / (Δcontrol_cycles + Δrejected_samples)`；两轮没有漏消费或编码器暂停，因此该分母适用于此比较。此处为相同参数和刺激方法的先后实测，不是完全相同的机械轨迹回放。首次修复镜像 ELF 为 `3e6fdba5ae5e048b15124f4ccf24c91cb88b62dc2037d34221ccf39182ff9ee3`。

原始记录：[修改前](../build/sampling-fix-20261105/baseline-detent.json)、[首次修复后](../build/sampling-fix-20261105/fixed-detent.json)、[差值汇总](../build/sampling-fix-20261105/comparison.json)。此前零输出自由模式约 11.47 s 仅拒绝 35 组；这也支持误拒绝主要与正常 PWM 更新相遇有关。[自由模式记录](../build/sampling-fix-20261105/baseline-active.json)

首次修复镜像还通过 60.003 s、143 次短脉冲、1350 份状态记录的运行检查：有效控制 299,710 次，拒绝 299 组，拒绝率 0.09966%，有效控制 4994.92 Hz。传输完整性异常、编码器时效跳过、漏消费、反馈恢复、PWM 延后、新增 100% 周期均为 0。原因计数为 trigger=298、pwm_tracking=29、phase_window=4，其余为 0；一组可命中多项，不能将原因计数相加。此结果没有把迟到的无效帧伪装成成功。[首次 60 秒检查](../build/sampling-fix-20261105/soak60.json)

## 诊断与验证范围

`rejected_samples` 是 HAL 拒绝组数；`timing_skips` 是控制任务跳过更新次数，两者都不能解释为 ADC 硬件转换错误次数。新增 `adc_reject_reasons`、六类 `adc_reject_counts`、`adc_timing_skips`、`encoder_timing_skips`，具体口径见[协议](protocol.md)。校验工具 [sampling_health_check.py](../tools/sampling_health_check.py) 按明确门槛判断采样健康，而非仅检查无故障。

首次修复镜像的扩展运动序列两次在准备阶段取消，单独反向测试也有一次准备取消；均为 stage=5、count=0、fault=none，原始失败记录保留：[序列一](../build/sampling-fix-20261105/normal-motion.json)、[序列二](../build/sampling-fix-20261105/normal-motion-retry.json)、[反向首次](../build/sampling-fix-20261105/motion-negative.json)。失败附近恰好新增触发迟到拒绝，且使能成功时才会重置的统计未重置，与单帧有效性准入门相符；旧诊断任务没有保留内部拒绝码，不能将这些记录说成直接捕获了具体错误字符串。

该镜像单独正反向 0.25 A 诊断目标、500 ms 驱动 / 500 ms 观察均有完成记录，峰值速度约 +44.77 / −44.93 rad/s；板上峰值相电流约 0.3358 / 0.2007 A。这些是瞬态板上估计，不是恒定电流精度标定。记录：[正向](../build/sampling-fix-20261105/motion-positive.json)、[反向复测](../build/sampling-fix-20261105/motion-negative-retry.json)。入口修正后的最终镜像验证另列下文，避免混用二进制证据。

本轮保留 `20k100_svpwm` 的 0–100% 软件指令范围。正常低占空比复现中，删去不必要条件已恢复控制频率，因此没有为解决这个问题额外降低输出能力。高占空比是另一种限制：现有固定 START、6 μs 建立余量和轮换测相意味着被测相在约 70% 附近就可能缺少满足当前判定的窗口，具体还取决于完成时刻；简单从 100% 改成 95% 并不能保证可采。真实低侧窗口不足仍拒绝，持续无反馈仍零矢量恢复。本次不声称持续满占空比反馈稳定、有载性能或外部相电流精度已经验证。

驱动低侧电流感测与建立时间参见 [TI DRV8316C 数据手册](https://www.ti.com/lit/ds/symlink/drv8316c.pdf)。软件窗口还包含传播、时序和回调完成余量；此次没有仅根据典型器件参数放宽整个板级窗口。

## 最终镜像复核

最终应用 BIN SHA256 为 `83cf11a3828f5a37d711e44007a9b06f50aaf2ec9f18f8e8b83e9ba5823328bf`，ELF SHA256 为 `2bc097dbd94b295eb2d46c3721805cd1e05d5e01a24c0c2d567a0b43c6a1bb5b`。只写应用分区并校验通过，NVS 未写入。备份和最终文件位于 `build/sampling-fix-20261105/previous-v8.bin/.elf`、`final-v9.bin/.elf`。

最终源码通过 16/16 桌面回归，包括首 TEZ 更新接受、采样对中途更新拒绝、真实窗口不足拒绝、零偏入口的新鲜度/失步边界，以及无效样本仍不能进入 PI。[桌面日志](../build/sampling-fix-20261105/host-tests-final.log)、[构建日志](../build/sampling-fix-20261105/build-final.log)、[烧录日志](../build/sampling-fix-20261105/flash-final.log)

入口修正后，0.08 / 0.25 / 0.8 A 三档运行预算的完整短脉冲及运动序列一次通过，所有诊断正常完成，准备取消为 0。运动诊断刺激仍另限于 0.25 A，不能将 0.8 A 预算称为已持续输出 0.8 A。实际锁存峰值 duty 约 45.85%，无新增满占空比周期。[最终运动序列](../build/sampling-fix-20261105/final-motion.json)

最终镜像 60.029 s 运行检查通过，147 次内置短脉冲全部接受、1326 份状态记录：

| 项目 | 最终实测 |
|---|---:|
| 有效控制次数 / 平均频率 | 299,977 / 4997.20 Hz |
| 拒绝采样 / 时序跳过 | 160 / 160 |
| 拒绝率 | 0.05331% |
| ADC 传输完整性异常 | 0 |
| 相窗口不足 / 采样对指令变化 | 0 / 0 |
| 编码器时效跳过 / 漏消费 | 0 / 0 |
| 反馈恢复 / PWM 延后 | 0 / 0 |
| 新增 100% PWM 周期 | 0 |

160 个拒绝组均含 trigger 原因，其中 22 组同时包含 pwm_tracking；保留这些计数如实反映偶发调度迟到。报告确认 fault=none、退出后 idle 且 duty=[0,0,0]、原参数与校准保持一致。[最终长时报告](../build/sampling-fix-20261105/final-soak60.json)

最终镜像另通过 7/7 USB 回归，包含分帧、流水、100 次查询、20 次重连及显式停止。最终设备读回 ELF 摘要与本地 `final-v9.elf` 一致，idle / fault=none / duty=[0,0,0]；配置和校准逐字段匹配本轮连接时的记录，没有保存 NVS。[最终 USB](../build/sampling-fix-20261105/usb-final.json)、[最终状态](../build/sampling-fix-20261105/final-status.json)、[验证汇总](../build/sampling-fix-20261105/validation-summary.json)

## 复现

导入工程的 ESP-IDF Python 环境（需 pyserial）后运行：

```powershell
python tools/sampling_health_check.py --port COM7 --seconds 60 --output build/commissioning/sampling-health.json
```

默认沿用现场模式和配置，每约 0.4 s 发送固件内置有界短脉冲。检查拒绝率低于 1%、有效控制高于 4900 Hz、无故障/传输异常、脉冲全部被接受，并确认退出停机与配置/校准恢复。结果写入新文件；不覆盖已有证据、不写 NVS。`--pulse-interval 0` 可关闭脉冲，`--mode` 只临时修改 RAM 模式。
