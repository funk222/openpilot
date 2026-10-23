# 问题诊断与解决方案

## 当前症状
- openpilot 系统启动后进入 Dashcam mode（Car Unreconized）
- 有时能显示速度，有时不能（报文接收不稳定）

## 根本原因
1. **Fingerprint 匹配失败**：openpilot 启动时扫描 CAN 总线上的报文"指纹"来自动识别车型，但我们的 Arduino ECU 发送的报文集合与真实 Corolla 不完全匹配
2. **报文发送频率或内容不完整**：Arduino 可能缺少某些关键报文或发送内容不对

## 快速修复方案

### 方案 1：强制指定车型（推荐）
在 EON 的 `/data/openpilot/launch_env.sh` 中添加：
```bash
export FINGERPRINT="TOYOTA COROLLA 2017"
export SKIP_FW_QUERY=1
```

这会跳过自动识别，直接使用指定的车型参数。

### 方案 2：完整诊断与 Arduino 固件改进
1. 从 EON 获取最近的启动日志，看具体缺少哪些报文
2. 对比真实 Corolla 的报文清单与 Arduino 发送的报文清单
3. 在 Arduino 固件中补齐缺失的报文

## 需要发送的关键报文（按频率）

来自 DBC 文件的要求：
- **0x0AA (WHEEL_SPEEDS)** - 80Hz ✓ Arduino 已发送（12ms 周期）
- **0x1D2 (PCM_CRUISE)** - 33Hz ✓ Arduino 已发送（20ms 周期）
- **0x1D3 (PCM_CRUISE_2)** - 可选 ✓ Arduino 已发送
- **0x3BC (GEAR_PACKET)** - 1Hz ✗ Arduino 发送的是 0x3B7，可能 ID 错
- **0x260 (STEER_ANGLE_SENSOR)** - 80Hz ✗ Arduino 可能没发
- **0x260 (STEER_TORQUE_SENSOR)** - 50Hz ✗ Arduino 可能没发
- **0x620 (BODY_CONTROL_STATE)** - 3Hz ✓ Arduino 已发送（100ms 周期）
- **0x614 (BLINKERS_STATE)** - 0.15Hz ✓ Arduino 已发送（100ms 周期）

## 立即执行步骤

1. 先用 **方案 1** 快速修复（让系统能识别为 Corolla）
2. 重启 EON，验证是否进入 ACC 模式而不是 Dashcam
3. 如果还有问题（例如速度仍间歇性不显示），再执行方案 2 的诊断

## 长期改进
- 补充缺失的转向角和转向力矩报文到 Arduino
- 增加报文冗余和容错机制
- 提高报文发送频率的稳定性
