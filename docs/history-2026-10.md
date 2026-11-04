# 2026-10 版本历史记录

以下保留本次 DMA/手感修改前的 README。端口、MAC、构建路径、参数和测试数字属于当时的设备与固件，不是 2026-11-03 本轮结果。当前使用方法以根目录 README 和协议文档为准。

# SmartKnob S3

ESP32-S3 / DRV8316CT / AS5600 / 2804（7 极对、12 槽、最大 1 A）的电流闭环触觉旋钮。使用 ESP-IDF 5.5，沿用 `../esp32_idf/esp_idf_template` 的 C++ 模块工厂、生命周期与失败回滚结构。

功能：d/q 电流 PI、Clarke/Park、低侧采样窗口约束的 SVPWM、MCPWM、AS5600 磁场诊断、相电流零点和电角度校准、自由旋转/虚拟档位/有限行程/弹簧回中、短触觉点击、USB 串口调参、NVS 保存。CAN 和实体屏幕驱动不包含在这一版；用户引脚表未指定屏幕，旋钮界面放在 wl1-studio。

## 工程入口

| 路径 | 职责 |
|---|---|
| `main/main.cpp` | 组合根，仅注册并启动模块 |
| `components/app` | 从模板复用的 AppManager / AppModule / AppTask |
| `components/knob_core` | 无 ESP-IDF 依赖的 FOC 数学、PI、角度展开、手感算法 |
| `components/knob_hal` | DRV8316CT 的 3PWM/nSLEEP、MCPWM 同步 ADC、AS5600 |
| `components/knob_app` | 电机状态机、命令队列、JSON 协议、校验和 NVS |
| `tests` | 桌面可运行的数学及手感边界测试 |
| `tools/knobctl.py` / `tools/commission.py` | 串口调试、限时运行及可重复的真机验收 |
| `docs/hardware.md` | 实际接线与尚待核实的电路参数 |
| `docs/protocol.md` | 上位机协议、参数范围及故障行为 |

电机任务独占控制状态和所有电机外设；通信任务通过有界 Command 队列请求变更，通过带请求 ID 的 Reply 获取快照。实时路径不解析 JSON、不写 Flash、不分配内存。模式、档距或行程改变时在当前位置重建档位中心；只调整强度、阻尼或电流限制时保留档位与中心，不重置正在运行的电流 PI。算法层可在 Linux 上独立测试；硬件细节限定在 Board 适配器中。通信传输在编译时选择，避免两个上位机同时续期扭矩租约。

## 默认参数

- 母线：**11.62 V**，由用户实测；未接母线 ADC，不会自动更新。
- 电流增益：**0.15 V/A**，因为模块 GAIN 未知，先取数据手册允许的最低增益。实际增益较高时电流会被高估，从而提前限流；显示值和扭矩强度需要在确认 GAIN 后重新标定。
- 运行电流：默认 0.65 A，上位机与固件均允许 0.02–1 A；运行时限制目标并按实测电流收减扭矩，异常过流保护另行保留。
- 电压矢量限制：默认 2.35 V；校准上限 0.8 V、d 轴目标 0.12 A。
- 默认手感：每 30° 一档，强度 5 A/rad，端点强度 8 A/rad；电流 PI 为 Kp=1.5、Ki=300。另提供轻盈 24 档、细腻 48 档及弹簧预设。
- PWM：20 kHz，三个操作器共用同一中心对齐计数器。
- 电流与手感控制：目标 2 kHz，MCPWM 每十个周期触发采样及任务通知。
- 高速手感：25–40 rad/s 渐弱，40 rad/s 暂停触觉，降到 5 rad/s 并稳定 30 ms 后用约 500 ms 恢复；运行会话保持 active，不报超速故障。
- AS5600：300 kHz I²C，12 位 RAW ANGLE；只修改易失 CONF，绝不烧写 OTP。

## 编译

已实测开发板为 ESP32-S3-DevKitC-1（兼容板，WCH USB 转串口），ESP32-S3 rev 0.2，16 MB Flash、8 MB PSRAM。默认通过板上 **USB** 口的原生 USB Serial/JTAG（GPIO19/20）调参，当前枚举为 `303a:1001`、`/dev/ttyACM0`。`menuconfig → SmartKnob → Tuning transport` 可改为 COM/UART0（GPIO43/44，115200）。同一固件只接收所选通道的命令；应用日志默认关闭，避免插入 JSON 回复。

本机已有容器 `esp-dev`，工具链实际安装在 `/opt/esp-idf`，与模板 README 的路径不同：

```bash
docker exec -w /workspace/smartknob_s3 esp-dev bash -c \
  'source /opt/esp-idf/export.sh && idf.py -B build/idf build'
```

产物：`build/idf/smartknob_s3.bin`、`build/idf/bootloader/bootloader.bin`、`build/idf/partition_table/partition-table.bin`，以及完整烧录参数 `build/idf/flash_args`。

实际工具链为 **ESP-IDF 5.5.4**。ADC 适配器先通过官方 oneshot 驱动独占 ADC1、设置衰减并生成 eFuse 校准查表，然后在同步 ISR 中用 `adc_ll` 切换通道并触发转换。驱动的通用 ISR 读取入口在本板测得两相约 82 μs，无法放入采样窗口；当前专用路径约 18–19 μs。此路径受 ESP32-S3 / IDF 5.5 编译期检查约束，升级必须重新验证。不要引入另一个 SAR ADC、ULP 或 MCPWM0 使用者。`sdkconfig.defaults` 开启 ADC/MCPWM IRAM 选项。

```bash
cmake -S tests -B build/host -G Ninja
cmake --build build/host
ctest --test-dir build/host --output-on-failure
```

容器以 root 构建后，可将本项目生成物归还用户，避免后续主机测试的目录权限问题：

```bash
docker exec esp-dev chown -R 1000:1000 /workspace/smartknob_s3/build /workspace/smartknob_s3/sdkconfig
```

## 下载与首次运行

1. 通过 BOOT + RESET/EN 进入下载模式，先确认芯片身份：

   ```bash
   docker exec esp-dev /root/.espressif/python_env/idf5.5_py3.12_env/bin/python \
     -m esptool --port /dev/ttyACM0 chip_id
   ```

2. 确认是目标 ESP32-S3 后烧录：

   ```bash
   docker exec -w /workspace/smartknob_s3 esp-dev bash -c \
     'source /opt/esp-idf/export.sh && idf.py -B build/idf -p /dev/ttyACM0 flash'
   ```

3. 在 wl1-studio 的产品库打开 **智能旋钮**，选择板上 **USB** 口对应的 **Espressif USB JTAG/serial debug unit** 串口连接。连接先识别协议，再请求停机；不会自动使能。
4. 查看磁场、采样耗时与丢弃计数；尚未校准时点击 **自动校准**。初期零点采样保持静止，随后电机会缓慢转动约一个电周期（机械约 51.4°）。校准同时检查 d/q 电流跟踪和机械位移，失败会停机并显示原因。当前板已完成并保存校准，可直接启动力反馈。
5. 校准成功后 **启动力反馈**，调节参数并 **应用参数**。修改母线/增益/电流 PI/电压限制须先停止。更改母线或增益后需重新校准。
6. 停止后 **保存到设备**；保存校准结果及参数，重启仍保持未使能。每次使能重新采集电流零点。

命令行可用宿主机现有 pyserial；不要同时打开上位机和调试工具：

```bash
python3 tools/knobctl.py --port /dev/ttyACM0 status
python3 tools/knobctl.py --port /dev/ttyACM0 calibrate
python3 tools/knobctl.py --port /dev/ttyACM0 save
python3 tools/knobctl.py --port /dev/ttyACM0 enable --seconds 5
```

`enable` 会持续发送状态心跳，并在结束/异常时发送停机。任何使能过程若 1 s 未收到有效命令，固件锁存 `host_timeout`。控制任务无法及时推进时，采样中断还会在 10 ms 截止时间拉低 nSLEEP。

当前 AS5600 的 STATUS 为 `0x33`：检测到磁铁，磁场偏弱。依用户的机械间隙限制，`CONFIG_KNOB_ALLOW_WEAK_MAGNET=y` 允许此状态运行，界面显示提示；磁铁丢失、磁场过强、I²C 错误和角度跳变仍会停机。不调整磁铁也可继续调参，实际角度误差和手感需要在这套机械结构上评估。

## 上位机

路径：`/home/kk/kk_data/rust_project/wl1-studio`。新增文件为 `src/components/knob/`、`src/lib/knob.ts`、`src-tauri/src/knob.rs`；接入产品路由、窗口关闭、切换产品和固件刷写互斥。

`src-tauri/Cargo.toml` 已对 dev/test profile 设置 `incremental = false`。运行构建时再显式指定：

```bash
cd /home/kk/kk_data/rust_project/wl1-studio
CARGO_INCREMENTAL=0 npm run tauri -- build --debug --no-bundle
./src-tauri/target/debug/wl1-studio --show
```

浏览器只提供界面预览；USB 使用桌面端的 serialport 后端。预览数据不会当作实测数据展示。界面包含实时角度、档位、电流曲线、采样耗时、磁场状态、参数和校准结果。

2026-10-30 的 USB 增强已直接应用到上位机源工程：只读握手等待、原生 USB 端口优先显示、实际调参通道、一键切换模式（保留未应用的表单修改），以及无效遥测时关闭会话。

上位机电流上限已提高到 1 A，并根据固件报告的 `current_limit_max` 校验；连接没有此字段的旧固件时仍限制为 0.8 A。界面分别显示设备已生效的运行上限、限流状态、异常保护阈值及过流停机现场。编辑后点击 **应用参数** 即更新正在运行的控制器；保存到设备仍须先停机。

后续 USB 鼠标应使用 USB-OTG/TinyUSB 的 CDC + HID 复合设备，复用此处的 JSON 协议与旋钮状态逻辑。当前 USB Serial/JTAG 是固定功能外设，此版本尚不发送鼠标报告。

## 2026-10-30 高速触觉渐变

参考 `/home/kk/kk_data/cpp_project/m5-rollercan_smart-knob/MyFile/src/smart_knob.c` 的高速迟滞和持续档位跟踪，将普通运行中的 `overspeed` 锁存改为触觉力度包络。保留本板 40 rad/s 阈值；25–40 rad/s 平滑减弱，达到阈值后约 10 ms 淡出到零，回到 5 rad/s 或以下并稳定 30 ms，再用约 500 ms 恢复。档位、端点、弹簧、阻尼及点击统一处理；高速期间仍更新逻辑位置并运行电流闭环，模式切换和归零不能绕过暂停。

固件新增 `haptic_scale`、`speed_limited`、`haptic_paused` 及阈值遥测。上位机现有协议解析接受这些可选字段，并继续依据 `state/fault` 显示运行状态；正常高速衰减不会触发错误横幅。真正的电流、编码器、采样和通讯故障仍停机，校准中快速运动仍中止。详见 `docs/protocol.md`。

联调发现控制任务可能消费同一采样对应的重复通知；现在按时间戳跳过，避免重复更新编码器和把同一采样重复计错。新增 `reused_samples` 计数、`encoder_us` / `control_us` 耗时及锁存的 `fault_sample` 诊断。使用 32 KB 指令缓存给控制、I²C 和 USB 路径更多缓存空间：同样的 USB 轮询下，30 秒静置记录的平均控制周期约 320 μs、峰值 367 μs；此前 16 KB 缓存的诊断记录平均约 700 μs。该变化消除了此前持续超过 500 μs 目标周期的现象，仍保留原来的采样窗口及故障阈值。AS5600 启动配置读取增加最多三次有限重试（仅驱动休眠时），处理 USB 复位中断总线事务的情况；运行期间的读取时限不变。

新增 C++ 测试覆盖双向渐变和迟滞、10 ms/500 ms 时间边界、30 ms 恢复保持时间和短暂反转、重复超速、四模式、1° 档位高速跟踪、点击及电流目标限速、无效输入。人工快转记录脚本：

```bash
python3 tools/speed_limit_check.py --port /dev/ttyACM0 --seconds 90 --require-cycle
```

运行前断开上位机。脚本使能后提示 READY；在记录期间双向快转并等待恢复，结束会停机并恢复原配置。只有实际捕获到暂停且随后完整恢复，`--require-cycle` 才通过；低速/静止记录不能算作高速验证。JSONL 原始快照旁保存 `.summary.json`。

### 板上 FreeRTOS 自动释放测试

`CONFIG_KNOB_DIAGNOSTICS` 默认关闭。调试构建增加独立的 `knob_test` FreeRTOS 任务，由显式 `diagnose` 命令启动；上电不自动驱动电机。测试任务通过专用队列请求配置、使能和有限时长的加速/释放，原电机任务独占外设、电流环和停止时限，USB 查询不会驱动测试时序。测试结束自动停机、恢复原运行参数，不写 NVS；主机 `disable` 可以取消测试。

本次调试镜像使用 `build/diagnostic.sdkconfig` 和 `build/idf-diagnostic/`，普通镜像仍用 `sdkconfig` 和 `build/idf/`。首次使用先将 `sdkconfig` 复制到 `build/diagnostic.sdkconfig`，用同一组 `-B` / `-D SDKCONFIG` 参数运行 `idf.py menuconfig`，启用 `SmartKnob → Enable bounded FreeRTOS motor release tests`。随后编译、烧录调试镜像：

```bash
docker exec -w /workspace/smartknob_s3 esp-dev bash -c \
  'source /opt/esp-idf/export.sh && idf.py -B build/idf-diagnostic -D SDKCONFIG=/workspace/smartknob_s3/build/diagnostic.sdkconfig -p /dev/ttyACM0 build flash'
python3 tools/motor_release_check.py --mode 1 --velocity 55 --current 0.15 \
  --drive-ms 300 --observe-ms 3000 --require-cycle \
  --output build/commissioning/speed-limit/release.json
```

板上每 2 ms 保存角度、速度、快速速度、目标/实测电流、触觉力度、电压及阶段，停止后分页取回。每次加速最多 500 ms、观察最多 3000 ms；驱动阶段的测试目标最多 0.25 A，释放阶段使用原配置的运行电流上限，另设 75 rad/s 停止界限。原有电流、编码器、采样和看门狗保护同时生效。恢复调参使用普通构建，它不包含测试命令。

弹簧/端点回程从 10–25 rad/s 渐弱同向加速力矩，同时保留制动；阻尼使用独立的 2 ms 速度滤波。这样既不在大位移回中时反复冲入高速暂停，也能及时跟踪反转，避免低速速度估计滞后带来的余振。

最终自动验证覆盖四种模式、正反两个方向共 8 组，每组以 ±55 rad/s 为驱动目标、0.15 A 为驱动阶段目标上限，加速 300 ms 后释放并观察 3000 ms。累计取回 12,907 个板上样本，实测最高 59.20 rad/s；8 组均捕获零触觉目标及完整恢复，未触发故障，结束自动停机并恢复参数。释放阶段使用原先 0.65 A 上限，全部测试的相电流采样峰值为 0.550 A。各组最后约 0.4 秒的速度峰值为 0.35–0.69 rad/s；这是真机无负载恢复记录，最终的人手触感尚未复核。

独立任务另通过非法计划拒绝、准备/驱动阶段取消且不重新使能、主机停止轮询 4 秒后仍自行完成并恢复参数的检查。正式镜像已重新烧录，确认不含测试入口；3 组桌面 C++ 测试、96 个在线限流/点击快照和 6 项 USB 回归均通过。设备最终为 idle / fault:none，原有 0.65 A 档位配置及电角度校准保持不变。

本次证据集中在 `build/commissioning/speed-limit/`：`validation-summary.json` 为总表，`release-matrix-summary.json` 为 8 组结果，原始样本文件由总表关联；另有 `diagnostic-session-checks.json`、`current-limits-final.json`、`usb-smoke-final.json` 和 `final.json`。失败的早期记录也保留，不能与最终验证混用。源码变更归档为 `changes.patch`，烧录前 Flash 与源码分别备份在 `flash-before.bin` 及 `*-before.tar.gz`。

## 2026-10-30 电流限制联调

旧版本虽然限制了扭矩目标，仍有两处固定 0.9 A 的过流锁存；上位机只能设置到 0.8 A。新版本统一支持 1 A，增加闭环收减、目标变化限速和分级过流保护。调试中还捕获到低上限 0.2 A 时的约 0.41 A 短时电流，原来的单样本判定会直接停机；现保留首次故障电流，并对普通越界作连续样本确认。

- 两组 C++ 控制测试通过，覆盖限幅、反转、实测电流收减/恢复、d 轴余量、在线降低上限、延迟 RL 模型，以及短脉冲/持续过流/立即保护判定。ESP-IDF 构建并烧录成功。
- 四种模式各执行 `0.8 → 1 → 0.2 → 1 → 0.02 → 0.8 A` 在线调整与触觉点击；96 个快照均 active、无故障，目标和可用电流始终在当前配置内，保护阈值按配置更新。1.01 A 被完整拒绝，原配置不变。
- 0.8 A 人手转动记录覆盖约 4.26 rad，目标峰值约 0.800 A，相电流采样峰值约 0.694 A，没有过流停机。约 10.6 秒时触发原有 40 rad/s 超速保护，用户确认此前快速甩转；当时仍使用超速锁存，现已由上文的高速触觉渐变替代。随后 60 秒记录无故障，但角度跨度仅 0.029 rad，只作为静置稳定性验证，不算完整的人手负载测试。
- 新固件通过 USB 分帧、100 次查询、20 次重连与拒绝命令不续心跳检查。上位机真实 Rust 后端通过 USB 握手、读取、拒绝 1.01 A、接受 1 A、恢复参数及断开停机验证。
- 上位机前端 251 项测试及 lint/typecheck/build 通过；Rust 工作区 197 项测试通过，14 项硬件测试默认跳过，其中旋钮串口测试另行实机通过；fmt/clippy 和 Tauri 桌面构建通过。实际桌面界面已连接 USB，并完成四种模式按钮切换与回执核对。没有在界面中自动使能电机。

完整记录：`build/commissioning/current-limit-validation.json`、`current-limits-qualified.json`、`current-limit-usb-smoke.json` 和 `current-limit-desktop-ui.json`。最终运行配置恢复为用户原先的 0.8 A / 虚拟档位，电机停机、无故障；没有改写原有 NVS 设置。上位机可执行文件为 `/home/kk/kk_data/rust_project/wl1-studio/src-tauri/target/debug/wl1-studio`。

```bash
python3 tools/current_limit_check.py --port /dev/ttyACM0
python3 tools/exercise.py --port /dev/ttyACM0 --seconds 60 --current-limit 0.8
```

运行脚本前先断开上位机，避免两个进程同时占用串口。脚本结束会停机并恢复运行配置。

## 采样与限流的实际边界

DRV8316CT 的 SOx 是**低侧电流感测**。本实现将 SVPWM 零矢量偏置到 000，占空比限制为 0.04–0.40，保留至少 30 μs 的公共低侧窗口。MCPWM 比较事件在下数阶段、TEZ 前 10 μs 触发。ISR 明确检查实际 ADC 开始时刻距最后 PWM 下降沿至少 6 μs，覆盖驱动传播和 CSA 建立时间，并要求结束距下一上升沿至少 1 μs。实际有效区间为相对 TEZ 的 −9 至 +14 μs，两相读取约 18–19 μs。ISR 轮换采两相，第三相以 Kirchhoff 关系重建；三个 ADC 引脚都会被使用。窗口失败不会送入 PI；连续三个无效控制样本停机。

d 轴 PI 先调节磁通，q 轴 PI 使用电压圆内剩余幅度；两轴各自按实际可用电压抗积分饱和，扭矩轴不再固定损失约 29% 的电压余量。回归测试覆盖电压圆边界及带一周期延迟的 RL 电流阶跃模型。

I²C 单次读取有时效检查：仅在第一次传输快速失败（500 μs 内）时重试一次；总读取超过 1 ms 即拒绝使用。磁场状态只有在成功取得 I²C 数据后才判断，不把 IDF 的事务失败错误码当作磁铁丢失。超过预算或重复失败会锁存停机，不自动恢复扭矩。

`current_limit` 同时用于手感目标限幅和实测电流反馈：q 轴目标按 100 A/s 平滑变化，预留实测 d 轴电流余量；相电流峰值或 d/q 矢量超过运行上限时快速收减目标，电流回落后缓慢恢复扭矩。降低上限立即限制目标，保护阈值延后 20 ms 收紧，给绕组电流留出衰减时间。

异常过流阈值为 `min(1.25, current_limit + max(0.15, current_limit × 0.25))` A，随配置同步：0.8 A 对应 1 A，1 A 对应 1.25 A。普通越界须连续 4 个有效 ISR 样本（2 kHz，约 2 ms）才锁存停机；任一有效样本达到 1.25 A 立即停机。ISR 包含第三相重建，故障快照保留首次触发电流和阈值。故障不会自动清除或重新使能。

以上是软件采样和闭环调节，不能宣称物理瞬时电流始终不超过设定值。当前接线未包含 nFAULT/DRVOFF，且此驱动器的逐周期限流模式不能同时提供 SOx。精确电流和瞬态峰值仍需确认 GAIN 并用外部仪器测量。采样不满足要求时固件会拒绝使能。

## 2026-10-30 原生 USB 验证

根因是接线与固件传输配置不一致：电脑连接原生 USB，但旧配置只在 UART0 接收命令；USB 日志输出不等于双向调参服务。已在 MAC `9c:13:9e:aa:f3:a4` 的同一开发板上烧录修复并取得 `transport:usb_serial_jtag` 的真实回复。

- ESP-IDF 5.5.4 构建及桌面 C++ 测试通过；新增力度调整保留档位、模式/档距重建中心、弹簧原点保持检查。
- 20 项原有板卡检查通过：四种模式、在线手感配置、点击、归零、参数拒绝、NVS 保存和断线停机。
- 6 项 USB 专项检查通过：逐字节/CRLF 分帧、连续四条请求、超长和控制字符帧丢弃、100 次查询、20 次重新打开串口、被控制状态机拒绝的合法命令不能续期心跳。
- 持续使能 24 秒，在线依次切换四种模式；210 个快照均为 active/无故障，结束恢复原配置并停机。该测试主要静置，不替代人手评估手感。
- 复位后配置与电角度校准和测试前一致，输出保持关闭、无故障；快照见 `build/commissioning/native-usb-reboot.json`。
- 报告在 `build/commissioning/native-usb-checks.json`、`usb-smoke.json` 和 `native-usb-mode-switch.jsonl`。烧录前额外备份启动、参数及应用分区至 `pre-native-usb-fix.bin`（Flash 0x0–0x10FFFF）。

```bash
python3 tools/usb_smoke.py --port /dev/ttyACM0
python3 tools/exercise.py --port /dev/ttyACM0 --seconds 24 --cycle-modes
```

## 此前执行的验证

- 实际烧录并完成零点、电流跟踪、电角度及 7 极对方向校准。
- 四种模式的限时运行、触觉点击、位置归零、在线手感调参及停机；这些自动检查不代替人手评估手感。
- 超限电流配置拒绝，运行中禁止修改电气参数及写 NVS；无效请求不会延长运行租约。
- 停止心跳后锁存 `host_timeout` 并关闭输出；显式清故障后才能重新运行。
- NVS 保存后复位核对，配置及校准恢复，输出保持关闭。
- 桌面 FOC 数学/电流极性测试，wl1-studio 前端 244 项测试，以及 Rust 回归与串口分帧/过期会话测试。
- 恢复调试后完成 600 秒持续运行（主要静止，无故障），以及连续 8 次校准扫描覆盖整圈机械角度；最终 28 项板卡检查通过，扫描期间编码器失败/重试均为零。静止检测要求连续两个 100 ms 窗口内角度变化小于 0.015 rad，对齐超时仍会停机。

手动转动的早期测试发现了电流响应偏慢、电压饱和和偶发 I²C 事务失败，已据此修改控制器及总线适配器。增强后的 12 档参数已保存，最后一版手感仍需使用者在桌面端实际确认；以上结果不宣称完成了满载电流或温升的仪器测量。完整结果见 `build/commissioning/final-checks.json` 和 `final-summary.json`。

可重新运行真机检查（先停用桌面串口连接；过程约 25 s，结束总会停机）：

```bash
python3 tools/commission.py --port /dev/ttyACM0
python3 tools/commission.py --port /dev/ttyACM0 --calibration-cycles 8
python3 tools/exercise.py --port /dev/ttyACM0 --seconds 600 --mode 1
```

实测快照和报告在 `build/commissioning/`。首次烧录前已完整备份原板 16 MB Flash：`build/commissioning/original-flash-16mb.bin`，SHA-256 为 `d6d160abcbefc54cbc7e56e9676cf9c5f824f513e9bdd065cdfc0bab5f9b9605`。

## 参考

- [SmartKnob：软件定义档位及端点交互](https://github.com/scottbez1/smartknob)。本工程独立实现硬件与电流控制，不依赖 Arduino/SimpleFOC。
- [TI DRV8316C 官方手册](https://www.ti.com/lit/ds/symlink/drv8316c.pdf)：§8.3.2、§8.3.11、Fig.8-30。
- [ESP-IDF MCPWM 文档](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/api-reference/peripherals/mcpwm.html)。
- 用户 NAS 的原始引脚表、DRV8316CTRGFR.pdf 和 AS5600-ASOM 手册，详见 `docs/hardware.md`。
