# 迁移清单：0.8.14 (EON/grey panda) → comma 4 / openpilot 0.11.x

> 目标分支：`011ODY`（comma 4 + 老 Odyssey，单 CAN，Toyota 信号 + Arduino ECU）
> 本文件是动手前的规划蓝图。先理清"0.11.x 架构变了什么"，再逐项把 0814ODY 的定制移植过去。
> 现状：comma 4 **尚未到手**，带 ⏳ 的项必须等实机验证。

---

## 0. 结论先行

你在 0.8.14 上为 Odyssey/Corolla 单 CAN 做的那套定制**不能直接 copy** 到 0.11.x——
车辆代码换了仓库、架构重写、panda 是内置现代 panda。`011ODY` 本质是**在新架构上重新实现**。
好消息：**参数值（steerRatio/wheelbase/mass 等）和设计思路都可复用**，只是落点和写法变了。

---

## 1. 架构差异总览（0.8.14 → 0.11.x）

| 方面 | 0.8.14（你现在的 EON） | 0.11.x（comma 4） |
|---|---|---|
| 系统 | NEOS | AGNOS |
| 设备 | EON / comma two | comma 4（内置现代 panda） |
| panda | grey（外置，boardd 伪装 black） | **内置 panda**，无需伪装 hack |
| 车辆代码位置 | `openpilot/selfdrive/car/toyota/` | **`opendbc/car/toyota/`（opendbc 子模块）** |
| interface 入口 | `CarInterface.get_params(...)` | **`_get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs)`** |
| 横向调参 | `set_lat_tune(..., LatTunes.PID_A)` + `pid.kpV/kiV` | **`configure_torque_tune(candidate, ret.lateralTuning)`**（torque-based，查 `torque_data/params.toml`） |
| centerToFront | `center_to_front_ratio` 变量 | `ret.centerToFront = ret.wheelbase * 0.44`（直接算） |
| CAN 总线连接 | grey panda 直插 OBD-II，全在 bus0 | **harness box → OBD-C**，CAN0=车、CAN2=摄像头（老车无） |

---

## 2. 逐项迁移清单

### 2.1 车辆动力学参数（Odyssey/Corolla 单 CAN）— ✅ 值可直接复用
**0814ODY 的黄金值（已实车验证"完美"）：**
```
wheelbase            = 3.00 m
steerRatio (静态)    = 18.8        # liveParameters 实际学到 ~17.9-18.8
mass                 = 4471 lb + cargo (~2164 kg)
tire_stiffness_factor= 0.82
centerToFront        = wheelbase * 0.41
steerActuatorDelay   = 0.10
angleOffset(学习)    ~ -6.5 ~ -7.0 deg（paramsd 在线学，不用静态给）
```
**0.11.x 落点**：`opendbc/car/toyota/interface.py` 的 `_get_params` 里，对应 candidate 分支设置
`ret.wheelbase / ret.steerRatio / ret.mass / ret.centerToFront / ret.steerActuatorDelay`。

### 2.2 横向 PID → torque tune — ⚠️ 写法变了，需转换
- 0.8.14：`ret.lateralTuning.pid.kpV=[0.28]; kiV=[0.08]`（直接 PID）。
- 0.11.x：默认走 **`configure_torque_tune(candidate, ret.lateralTuning)`**，参数来自
  `opendbc/car/torque_data/params.toml`（如 `"TOYOTA_COROLLA" = [LAT_ACCEL_FACTOR, FRICTION, ...]`）。
- **行动**：要么沿用 torque tune（更符合新架构，推荐先试默认），要么显式切回 PID。
  你那套 kp=0.28/ki=0.08 是 0.8.14 PID 语义，**不能直接塞进 torque tune**，需要重新标定或先用默认 torque tune 上车学。
- ⏳ 实机：torque tune 的手感要上车验证，很可能直接用官方 Corolla torque 数据就不错。

### 2.3 单 CAN 适配（SINGLE_CAN flag）— ⚠️ 机制要重建
- 0.8.14：自定义 `ToyotaFlags.SINGLE_CAN`，在 interface/carstate/carcontroller/radar 里特判。
- 0.11.x：`ToyotaFlags` 枚举完全不同（有 SECOC/TSS2/ANGLE_CONTROL 等，无 SINGLE_CAN）。
- **行动**：在 0.11.x 的 `ToyotaFlags` 里新增自定义 `SINGLE_CAN`，并在新代码的对应位置重做特判：
  - interface：套用 Odyssey 参数
  - carstate：单 CAN 的 EPS/非标 LKA_STATE 处理（见 2.5）
  - carcontroller：转向只发 CAN0（见 2.4）
  - radar_interface：单 CAN 关掉 radar gating

### 2.4 panda 转发 jam 修复（bus0→bus2）— ⏳ 必做，需实机定位
- 0.8.14 根因：Toyota 安全把 bus0 每帧转发到 bus2（摄像头），单 CAN 无 bus2 → TX 堵死。
  修复：`panda/board/safety/safety_toyota.h` 的 `toyota_fwd_hook` 把 `bus_num==0` 的 `bus_fwd=2` 改 `-1`。
- comma 4：**内置现代 panda**，`safety_toyota.h` 逻辑类似但 CAN 编号/harness 行为不同。
  comma 4 的 harness 本就把 CAN2=摄像头、CAN0=车分开，**单 CAN（无摄像头）下是否仍有同样的转发 jam，必须实机确认**。
- **行动**：⏳ comma 4 到手后，先不打补丁跑一次，看 `canFwdErrs` 是否爆 + 转向能否到 EPS；
  若复现，再在 comma 4 的 panda 固件做等价的"单 CAN 不转发"补丁（out-of-tree patch，同 0814 做法）。

### 2.5 非标 EPS 的 LKA_STATE 处理（允许 3，打火-17 预防）— ✅ 逻辑可复用
- 0.8.14：`single_can` 下 `steerWarning = lka_state not in (1,3,5)`（允许非标 3）。
- 打火-17：EPS 在 STEER_REQUEST=1 时硬断电会латch 故障 17；预防=打火前 CANCEL，恢复=干净长断电。
- **0.11.x 落点**：`opendbc/car/toyota/carstate.py` 的 EPS_STATUS/steerFault 判定处，重做单 CAN 允许列表。
- ⏳ 实机：这台非标 EPS 的码（1/3/5/17）在 comma 4 上应一致（EPS 没变），但代码位置要重找。

### 2.6 指纹 / 启动绕过 — ⚠️ 大部分不再需要
- 0.8.14 用了：`FINGERPRINT` 强制、`SKIP_FW_QUERY`、NEOS 绕过、无 LFS、grey→black 伪装、相机/DM/非官方硬件告警抑制。
- 0.11.x / comma 4：
  - **NEOS/LFS/grey 伪装：不需要**（AGNOS、内置 panda）。
  - **指纹强制**：0.11.x 指纹机制变了，单 CAN 仍可能要强制 candidate，⏳ 实机定。
  - **相机/DM 告警**：comma 4 有真实内录摄像头，单 CAN 无前视摄像头 → 可能仍要抑制部分告警，⏳ 实机定。
- **行动**：这些逐条在 comma 4 上验证哪些还需要，别盲目搬。

### 2.7 ECU（Arduino）— ✅ 基本不变
- Arduino 发 Toyota CAN 报文集（0x1D2/0x1D3/0xAA/0x2E4 等）+ 智能门控自动 engage + pump_rx + 0x701/0x702。
- comma 4 单 CAN 同样接在这条总线上，**ECU 固件逻辑不变**（Odyssey 用脉冲 VSS 版 `ECU_fixed_option1_tolerant.ino`）。
- ⏳ 唯一要确认：comma 4 内置 panda 在单 CAN 下对这些报文的收发时序，与 grey panda 是否一致。

---

## 3. 硬件接线（comma 4 + 老车 OBD-II）

- 车 CAN 全在 OBD-II：**CAN-H=Pin6、CAN-L=Pin14、12V=Pin16、GND=Pin4/5**。
- 自制 car harness（18-pin Molex 5016461800 + 压接针 5016471000）：
  | 车 OBD-II | → Molex Pin | 色 |
  |---|---|---|
  | CAN-H (Pin6) | **Pin 4**（CAN0_H） | 橙 |
  | CAN-L (Pin14) | **Pin 6**（CAN0_L） | 绿 |
  | 12V (Pin16) | **Pin 10**（或 10+12 更稳） | 红 |
  | GND (Pin4/5) | **Pin 1**（或 1+17 更稳） | 黑 |
- CAN2（Pin16/18 摄像头）不接；CAN1（雷达）不接。
- 终端电阻：量 OBD-II 的 CAN-H↔CAN-L，~60Ω 不用加；~120Ω 则 Molex Pin3↔Pin5 加 120Ω loopback。
- OBD-C cable：**用全针 USB-C 3.2 Gen2**（SBU1/SBU2 必须接全，驱动继电器+点火检测）；建议官方线。
- comma 4 内置 panda：无需外置 grey panda。

---

## 4. 建议执行顺序

1. **现在（无实机）**：建 `011ODY` 分支（从 commaai master clone），把 §2.1 参数落到
   `opendbc/car/toyota/interface.py` 的 Odyssey candidate，作为骨架。横向先用默认 torque tune。
2. **comma 4 到手**：装机 + 自制 harness → 上电看能否枚举内置 panda、识别单 CAN。
3. **复现转向链路**：跑一次看 `canFwdErrs` / 转向能否到 EPS；若 jam 复现，做 panda 转发补丁（§2.4）。
4. **EPS/打火**：重做非标 LKA_STATE 允许 + 打火-17 预防（§2.5）。
5. **调参**：用 liveParameters + rlog 拟合，复制 0814ODY 的"完美"路径。
6. **ECU**：沿用现有 Odyssey 固件，必要时按 comma 4 时序微调。
7. **E350**：011ODY 稳定后，再按同法做 `011E350`（换 §2.1 为 E350 参数 4.0m/17/11500lb/kf=7.8e-5 + 模拟电压 ECU）。

---

## 5. 复用速查（从 0814 搬值）

| 项 | Odyssey/Corolla（011ODY） | E350（后续 011E350） |
|---|---|---|
| wheelbase | 3.00 m | 4.00 m |
| steerRatio | 18.8 | 17 |
| mass | 4471 lb | 11500 lb |
| tire_stiffness | 0.82 | 0.444 |
| steerActuatorDelay | 0.10 | 0.12 |
| PID（旧语义，供参考） | kp0.28/ki0.08/kf3e-5 | kp0.9/ki0.01/kf7.8e-5 |
| ECU 固件 | 脉冲 VSS 版 | 模拟电压版（A4，a0.00008/b0.06/160avg） |

> 注：0.11.x 用 torque tune，上面 PID 值仅作手感参考，实际落 `torque_data` 或重标定。
