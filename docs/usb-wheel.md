# ESP32-S3 USB OTG 鼠标滚轮

`dma-v24-usb-wheel` 在原生 USB（GPIO19 D− / GPIO20 D+）上提供 CDC 调参串口与 HID 鼠标复合设备。原来的电流环、触觉算法、模式参数、校准记录与 NVS 布局保持不变。

## 使用

1. 连接板上的原生 USB 口，打开配套 `D:\kk\rust\cf-gui\src-tauri\target\release\wl1-studio.exe`。
2. 进入“智能旋钮”，连接新的 USB 串口。本机从原来的 COM7 变为 COM8；具体编号由 Windows 分配。
3. 在“USB 鼠标滚轮”区域点击“开启滚轮”或“关闭滚轮”。力反馈的启动、停止、模式选择仍使用原有入口。

开关只写运行时状态，不写 NVS，不修改或保存任何手感参数。重启默认关闭；开启后关闭上位机或关闭 CDC 端口，HID 滚轮继续独立工作。USB 暂停、未枚举、编码器失效，以及校准或故障状态会抑制滚轮并丢弃历史位移。恢复时从当前位置重新计量，不补发离线期间的转动。

当前滚轮固定为 **36 刻度/圈（10°/刻度）**，适用于所有手感模式。它独立于可编辑的手感档距；修改手感档距不会改变滚轮比例。正编码器方向对应正 HID Wheel，负方向对应负值，页面滚动方向和每刻度滚动行数由操作系统解释。

## 实现与时序

- C++20 / ESP-IDF 5.5，固定 `espressif/esp_tinyusb 2.2.0`，传递依赖由 `dependencies.lock` 锁定；使用 S3 USB OTG 内部 PHY 和 TinyUSB DMA。
- 编码器任务发布一个固定大小的角度快照。独立 CPU0 任务每 1 ms 处理跨圈、分数位移和 0.55 刻度施密特阈值，然后通过 HID interrupt IN 端点上报。空闲不发 HID 报告，发送路径没有逐帧堆分配。
- FOC 仍在 CPU1 以原频率运行，USB 线程优先级低于编码器任务。实时控制线程只发布一个原子许可位，不调用 USB、不等待 USB，也不执行滚轮计算。
- Windows 的鼠标类驱动要求描述符具有指针轴。兼容报告为四字节 `[buttons=0, x=0, y=0, wheel]`，无 Report ID。只有有符号相对滚轮字段变化；不发送角度、速度、力矩或模拟指针移动。上位机没有使用 `SendInput` 或任何软件滚动注入。
- USB 枚举始终包含 HID 接口，切换按钮不重新枚举设备。VID/PID 为 `303A:4005`，序列号来自芯片 MAC。CDC 保留协议 v1，新增通道名 `usb_otg_cdc`。
- CDC 发送按 FIFO 剩余容量推进，兼容 DTR/RTS 为低的旧调试客户端，完整回复仍有发送超时和换行重同步。USB 服务与电流控制分离。

官方接口说明：[Espressif USB Device Stack](https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32s3/api-reference/peripherals/usb_device.html)、[esp_tinyusb 2.2.0](https://components.espressif.com/components/espressif/esp_tinyusb/versions/2.2.0/readme)、[Microsoft HID 鼠标驱动](https://learn.microsoft.com/en-us/windows-hardware/drivers/hid/keyboard-and-mouse-hid-client-drivers)。

## 协议

```json
{"id":1,"op":"wheel_enable"}
{"id":2,"op":"wheel_disable"}
```

回复沿用完整状态结构，新增：

```json
"usb_wheel": {
  "supported": true,
  "enabled": false,
  "mounted": true,
  "suspended": false,
  "ready": false,
  "steps_per_turn": 36,
  "reports": 0,
  "steps": 0
}
```

`enabled` 是设备确认的开关状态，`ready` 表示当前满足输出条件；开关确认后的下一次 USB 任务更新才刷新 `ready`。`reports` 是成功提交给 HID 端点的累计报告数，`steps` 是成功提交的绝对滚轮刻度累计数；二者为 uint32，不能替代主机实际接收验证。禁用会清除待发位移，已交给 USB 硬件的一份报告可能仍完成传输。命令重复执行是幂等的，不重置已开启的角度基准。

## 构建、再次烧录与诊断

在 ESP-IDF 环境执行 `idf.py reconfigure build`。已有 sdkconfig 从 Serial/JTAG 版本升级时，要同步 `sdkconfig.defaults` 的 `CONFIG_TINYUSB_*` 设置，或在独立构建目录中生成新 sdkconfig。保留原电机配置，禁止为了获得 HID 而烧写 USB PHY 相关 eFuse。

应用运行在 USB OTG 模式时，原 Serial/JTAG 下载口不再同时可用。使用本工程的 1200-baud touch 入口进入 ROM 下载模式：

```powershell
python tools/usb_bootloader.py --port COM8
# 确认工具报告的 ROM 端口，例如 COM7，然后仅烧录应用分区：
python -m esptool --chip esp32s3 --port COM7 write_flash 0x10000 build/smartknob_s3.bin
```

固件先向电机线程请求停机并等待确认，再切换 PHY 和重启到 ROM。普通 115200 波特率的连接、关闭和 DTR/RTS 变化不会重启设备。若应用无法运行，仍可按住 BOOT、点按 RESET、松开 BOOT 进入 ROM。仅写应用分区保留现有 NVS。

正式镜像不启用 `CONFIG_KNOB_DIAGNOSTICS`。调试镜像新增 `CONFIG_KNOB_DIAGNOSTIC_CAPACITY`，默认 1024，减少轨迹缓冲占用，为 USB 与完整 JSON 回复保留堆空间；采集周期仍按总时长和容量计算。它不改变手感、电流参数或控制频率。

`tools/windows_wheel_capture.py` 使用 Windows Raw Input 监听真实硬件事件，按 VID/PID 过滤，记录滚轮增量、按钮标志和 X/Y。它不生成鼠标事件，也不向电机发送命令。

本轮备份、构建、逐次失败及验证记录保存在 `build/usb-wheel-20261113/`；桌面验证在配套仓库 `.local/usb-wheel-20261113/`。最终验收结果以 `validation-summary.json` 为准。

## 本轮验收结果

已烧录正式、无诊断入口的镜像，运行 ELF 散列与最终构建一致。C++ 30 项、Python 53 项、前端 384 项和 Rust 196 项测试通过；Rust 另有 18 项环境/硬件测试默认忽略。Biome、TypeScript、rustfmt、Clippy 与两个生产构建通过。

真实 Windows Raw Input：诊断镜像以 0.12 A 最大激励、500 ms 驱动做正反转，各收到 18 个 ±120 的滚轮事件；关闭滚轮后同样转动收到 0 个事件。X/Y 和按键均为零。关闭 CDC 后的独立转动另收到 19 个事件。两类动作均由编码器实际测量产生，未注入合成 HID 数据。

最终正式镜像在原配置下，滚轮关闭/开启分别运行 25.125 / 25.269 秒，控制频率为 4999.761 / 4999.763 Hz。两段分别有 6 / 5 组采样拒绝，没有 transport、pair_change、反馈恢复、编码器错误或故障；拒绝计数未隐藏，也不宣称全程零拒帧。

Release 上位机通过真实 COM8 的开关、断开和重连检查。最后恢复升级前未保存的 RAM 模式 1 / active，运行上限约 0.8 A，滚轮关闭；完整配置、已接受校准及质量数据与初始快照相同。NVS 的 24576 字节与备份逐字节一致，没有将 RAM 模式覆盖到原持久化配置。电脑休眠未在本轮实测，不将其列为实机通过项。
