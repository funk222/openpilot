// =========================================================================
// Ford E350 单 CAN ECU（Toyota 信号 + Ford E350 车身，LATERAL-ONLY）
// 目标：匹配 openpilot 0813NoLFS / 0814E350 分支
//
// 与 Corolla 版（ECU_fixed_option1_tolerant.ino）唯一的区别：
//   车速来自 **A4 的模拟电压**（E350 没有脉冲 VSS），而不是脉冲计数。
//   标定沿用经过验证的 084FordE350LateralOnlyECU.cpp：
//     spd_kph = 1.609 * (a*V^2 + b*V), V=analogRead(A4), 160 点滑动平均, ×3/40。
//   其余（智能门控自动 engage、pump_rx、0x701/0x702 诊断、0x262 EPS 门控、
//   Toyota CAN 报文集与校验和）与 Corolla 版完全一致。
//
// 关键补齐（同 Corolla 版）：0x620 / 0x614 / 0x622 / 0x2C1。
// 注意：真实 BCM 已发这些报文时把 EMIT_BCM_FALLBACK 设 false。
// =========================================================================

#include <Arduino.h>
#include <CAN.h>

const byte CAN_CS_PIN = 10;
const byte CAN_INT_PIN = 2;
const byte pinVssAnalog = A4;

#define TIRE_CIRCUMFERENCE 1.993
#define PULSES_PER_REV 50
const float VSS_SCALE = (TIRE_CIRCUMFERENCE * 36.0f) / (float)PULSES_PER_REV;
// --- E350 analog speed sensor (replaces pulse VSS) ---
// Ford E350 feeds vehicle speed as an analog voltage into A4 (not a pulse train).
// Calibration ported from the proven 084FordE350LateralOnlyECU.cpp:
//   spd_kph = 1.609 * (a*V^2 + b*V), V = analogRead(A4) [0..1023], averaged over
//   ANALOG_FILTER_WINDOW samples and post-scaled by ANALOG_POST_SCALE (3/40).
const bool USE_ANALOG_VSS = true;
const float ANALOG_VSS_A = 0.00008f;   // quadratic term (E350 curve fit)
const float ANALOG_VSS_B = 0.06f;      // linear term (E350 curve fit)
const float ANALOG_TO_KPH = 1.609f;    // mph -> kph
const float ANALOG_POST_SCALE = 3.0f / 40.0f;
const bool USE_PEDAL_SWITCH_INPUTS = false;

const byte pinVssInterrupt = 3;
const int pinButtonMain = 8;
const int pinButtonPlus = 7;
const int pinButtonMinus = 6;
const int pinButtonCancel = 9;

const int pinBrakeInput = A0;
const int pinGasInput = A1;

unsigned long last_20ms_tick = 0;
unsigned long last_12ms_tick = 0;
unsigned long last_25ms_tick = 0;  // EPS_STATUS timing
unsigned long last_50ms_tick = 0;
unsigned long last_100ms_tick = 0;
unsigned long last_vss_tick = 0;
unsigned long last_btn_tick = 0;

const unsigned long BTN_DEBOUNCE_DELAY = 50;
const uint8_t MIN_SET_SPEED_KPH = 10;
const uint8_t MAX_SET_SPEED_KPH = 180;
const float TOYOTA_WHEEL_SPEED_SIGNAL_OFFSET_KPH = 67.67f;
// 速度显示微调系数：
// - 默认 1.00：按几何参数原始计算
// - 显示偏高：调小（如 0.96）
// - 显示偏低：调大（如 1.04）
const float TOYOTA_WHEEL_SPEED_SCALE = 0.25f;

// 如果真实 BCM 没有这些报文，保持 true；如果真实 BCM 已经发送，请改成 false
const bool EMIT_BCM_FALLBACK = true;

// 总线上若存在真实 Corolla EPS（会自己发 0x262 EPS_STATUS），必须保持 false，
// 否则 Arduino 伪造的 0x262 会与真实 EPS 抢同一 CAN ID，导致 LKA_STATE 在真/假值间乱跳，
// 无法判断真实 EPS 是否接受了转向命令。只有在完全没有真实 EPS 时才设为 true。
const bool EMIT_FAKE_EPS_STATUS = false;

// 自动 engage：检测到 openpilot（总线上出现 0x2E4 STEERING_LKA）稳定运行后，
// 自动制造一次 CRUISE_ACTIVE 的 0->1 跳变来触发 engage，省去每次手动按 MAIN。
// 关键：panda 的 controls_allowed 是上升沿锁存，开机必须先是 0、再跳到 1 才能锁住。
//
// 智能门控自动 engage：上电后，等「openpilot 就绪(总线上持续收到 0x2E4)」且
// 「真实 EPS 处于健康 standby(LKA_STATE==EPS_READY_LKA_STATE) 并稳定 ≥
// AUTO_ENGAGE_EPS_READY_MS」后，才一次性制造 CRUISE_ACTIVE 的 0->1 跳变挂上。
// 这样可避免对「刚上电/故障态(如打火硬断电后латch 的 LKA_STATE=17)」的 EPS 发命令：
//  - 冷启动 EPS 干净到 standby(1) -> 自动挂上；
//  - 热重启未 CANCEL 导致 EPS=17 -> eps_ready 永不成立 -> 不会自动挂(正确)。
const bool AUTO_ENGAGE_ON_OP = true;
const unsigned long OP_ALIVE_TIMEOUT_MS = 500;    // 多久没看到 0x2E4 视为 openpilot 掉线
const unsigned long AUTO_ENGAGE_STABLE_MS = 2000; // （保留）openpilot 稳定在线多久
const unsigned long AUTO_ENGAGE_MIN_2E4 = 40;     // 累计收到多少帧 0x2E4 视为 openpilot 就绪
const uint8_t EPS_READY_LKA_STATE = 1;            // 这台非标 EPS 的健康 standby 码(实测=1)
const unsigned long AUTO_ENGAGE_EPS_READY_MS = 3000; // EPS 连续处于 standby 多久才允许挂

// 模拟车速：台架测试 EPS 是否因无车速才不执行 LKA 时使用。
// 开启后 WHEEL_SPEEDS 输出固定车速。真实车辆上务必保持 false！
const bool SIM_SPEED_ENABLE = false;
const float SIM_SPEED_KPH = 20.0f;

unsigned long op_last_seen = 0;     // 最近一次看到 openpilot 0x2E4 的时间
unsigned long op_seen_since = 0;    // openpilot 开始稳定在线的时间（用于去抖）
boolean op_steer_req = false;       // openpilot 当前是否 active（0x2E4 byte0 bit0）
boolean auto_engaged_done = false;  // 一次性自动 engage 标志

unsigned long rx_any_count = 0;     // 收到任意 CAN 帧的总数（验证 RX 是否工作）
unsigned long rx_2e4_count = 0;     // 收到 openpilot 0x2E4 的总数

// 诊断：记录 Arduino 到底从总线上收到过哪些关键地址（用于区分
// "0x2E4 根本没上总线" 还是 "固件 RX 漏读"）。
boolean ever_2e4 = false;           // 收到过 openpilot 0x2E4 STEERING_LKA
boolean ever_25 = false;            // 收到过真实 EPS 的 0x25 STEER_ANGLE_SENSOR
boolean ever_260 = false;           // 收到过真实 EPS 的 0x260 STEER_TORQUE_SENSOR
boolean ever_262 = false;           // 收到过真实 EPS 的 0x262 EPS_STATUS
boolean ever_343 = false;           // 收到过 openpilot 的 0x343 ACC_CONTROL（判定 panda bus0 TX 是否出得去）
boolean ever_412 = false;           // 收到过 openpilot 的 0x412（同上，另一条 openpilot TX）
volatile uint16_t last_rx_id = 0;   // 最近一次收到的帧 ID

uint8_t eps_lka_state = 0xFF;       // 真实 EPS 0x262 的 LKA_STATE(7位解码)；0xFF=未知
unsigned long eps_ready_since = 0;  // EPS 连续处于健康 standby 的起始时间(0=当前不健康)

unsigned long can_tx_fail_count = 0;

boolean OP_ON = false;
boolean MAIN_ON = true;
uint8_t set_speed = 0;
float current_speed = 0.0;
uint8_t cruise_state = 7;  // 0=off, 7=standby, 8=active

boolean btn_main_last = HIGH;
boolean btn_plus_last = HIGH;
boolean btn_minus_last = HIGH;
boolean btn_cancel_last = HIGH;

boolean BRAKE_PRESSED = false;
boolean GAS_RELEASED = true;

volatile unsigned long pulse_count = 0;
void vss_pulse_counter() { pulse_count++; }

const int ANALOG_FILTER_WINDOW = 160;  // E350: 160-sample moving average (per reference ECU)
float analog_speed_hist[ANALOG_FILTER_WINDOW] = {0.0f};
float analog_speed_sum = 0.0f;
int analog_speed_idx = 0;

float read_analog_speed_kph() {
  int v = analogRead(pinVssAnalog);
  float inst_kph = ANALOG_TO_KPH * (ANALOG_VSS_A * (float)v * (float)v + ANALOG_VSS_B * (float)v);
  if (inst_kph < 0.0f) inst_kph = 0.0f;

  analog_speed_sum -= analog_speed_hist[analog_speed_idx];
  analog_speed_hist[analog_speed_idx] = inst_kph;
  analog_speed_sum += inst_kph;
  analog_speed_idx = (analog_speed_idx + 1) % ANALOG_FILTER_WINDOW;

  float filtered_kph = (analog_speed_sum / (float)ANALOG_FILTER_WINDOW) * ANALOG_POST_SCALE;
  if (filtered_kph < 0.0f) filtered_kph = 0.0f;
  return filtered_kph;
}

float get_ui_speed_kph() {
  if (SIM_SPEED_ENABLE) return SIM_SPEED_KPH;
  float ui_speed_kph = current_speed * TOYOTA_WHEEL_SPEED_SCALE;
  if (ui_speed_kph < 0.0f) ui_speed_kph = 0.0f;
  return ui_speed_kph;
}

uint8_t clamp_set_speed(float speed_kph) {
  int value = (int)speed_kph;
  if (value < MIN_SET_SPEED_KPH) return MIN_SET_SPEED_KPH;
  if (value > MAX_SET_SPEED_KPH) return MAX_SET_SPEED_KPH;
  return (uint8_t)value;
}

void can_send_8(uint16_t addr, const uint8_t *dat) {
  CAN.beginPacket(addr);
  CAN.write(dat, 8);
  if (!CAN.endPacket()) can_tx_fail_count++;
  pump_rx();
}

void can_send_n(uint16_t addr, const uint8_t *dat, uint8_t len) {
  CAN.beginPacket(addr);
  CAN.write(dat, len);
  if (!CAN.endPacket()) can_tx_fail_count++;
  pump_rx();
}

uint8_t toyota_checksum(uint8_t *dat, uint8_t len, uint16_t addr) {
  uint16_t checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  for (int i = 0; i < len; i++) checksum += dat[i];
  return (uint8_t)(checksum & 0xFF);
}

// 读空 MCP2515 的 RX 缓冲（最多 N 帧/次），解析 openpilot 的 0x2E4 并记录诊断。
// 关键：MCP2515 只有两级 RX 缓冲，总线繁忙（~500 帧/秒）时必须在每次 TX 之间
// 频繁调用本函数，否则缓冲溢出会把 0x2E4 等帧整类丢光。
void pump_rx() {
  for (uint8_t rxi = 0; rxi < 16; rxi++) {
    int rx_size = CAN.parsePacket();
    if (!rx_size) break;
    long rx_id = CAN.packetId();
    rx_any_count++;
    last_rx_id = (uint16_t)rx_id;
    if (rx_id == 0x2E4) {
      ever_2e4 = true;
      if (rx_size >= 1) op_steer_req = ((uint8_t)CAN.read() & 0x01);  // STEER_REQUEST
      op_last_seen = millis();
      rx_2e4_count++;
    } else if (rx_id == 0x25) {
      ever_25 = true;
    } else if (rx_id == 0x260) {
      ever_260 = true;
    } else if (rx_id == 0x262) {
      ever_262 = true;
      // 解码真实 EPS LKA_STATE(EPS_STATUS 31|7@0+ => (byte3>>1)&0x7F)，
      // 并维护「连续处于健康 standby」计时，供智能门控自动 engage 使用。
      if (rx_size >= 4) {
        (void)CAN.read();                      // byte0
        (void)CAN.read();                      // byte1
        (void)CAN.read();                      // byte2
        uint8_t b3 = (uint8_t)CAN.read();      // byte3 含 LKA_STATE
        eps_lka_state = (b3 >> 1) & 0x7F;
        if (eps_lka_state == EPS_READY_LKA_STATE) {
          if (eps_ready_since == 0) eps_ready_since = millis();
        } else {
          eps_ready_since = 0;                 // 非 standby(故障/active) -> 复位计时
        }
      }
    } else if (rx_id == 0x343) {
      ever_343 = true;
    } else if (rx_id == 0x412) {
      ever_412 = true;
    }
    while (CAN.available()) CAN.read();
  }
}

void setup() {
  CAN.setPins(CAN_CS_PIN, CAN_INT_PIN);
  if (!CAN.begin(500E3)) {
    while (1) { }
  }

  pinMode(pinVssInterrupt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinVssInterrupt), vss_pulse_counter, FALLING);

  pinMode(pinButtonMain, INPUT_PULLUP);
  pinMode(pinButtonPlus, INPUT_PULLUP);
  pinMode(pinButtonMinus, INPUT_PULLUP);
  pinMode(pinButtonCancel, INPUT_PULLUP);
  pinMode(pinBrakeInput, INPUT_PULLUP);
  pinMode(pinGasInput, INPUT_PULLUP);
}

void loop() {
  unsigned long now = millis();

  pump_rx();

  if (USE_ANALOG_VSS) {
    // E350: sample the analog speed voltage every loop iteration so the
    // 160-sample moving average settles in ~1-2s (faithful to the reference ECU,
    // which sampled fast). Gating this to 10Hz would add ~16s of lag.
    current_speed = read_analog_speed_kph();
  } else if (now - last_vss_tick >= 100) {
    last_vss_tick = now;
    noInterrupts();
    unsigned long pulses = pulse_count;
    pulse_count = 0;
    interrupts();
    current_speed = (float)pulses * VSS_SCALE;
  }

  if (USE_PEDAL_SWITCH_INPUTS) {
    BRAKE_PRESSED = (digitalRead(pinBrakeInput) == LOW);
    GAS_RELEASED = (digitalRead(pinGasInput) == LOW);
  } else {
    BRAKE_PRESSED = false;
    GAS_RELEASED = true;
  }
  if (BRAKE_PRESSED || !GAS_RELEASED) OP_ON = false;

  if (now - last_btn_tick >= BTN_DEBOUNCE_DELAY) {
    last_btn_tick = now;
    boolean btn_main = digitalRead(pinButtonMain);
    boolean btn_plus = digitalRead(pinButtonPlus);
    boolean btn_minus = digitalRead(pinButtonMinus);
    boolean btn_cancel = digitalRead(pinButtonCancel);

    // Single-key engage/disengage: MAIN button toggles OP_ON directly
    if (btn_main == LOW && btn_main_last == HIGH) {
      if (OP_ON) {
        OP_ON = false;
      } else {
        OP_ON = true;
        set_speed = clamp_set_speed(get_ui_speed_kph() + 3.0f);
      }
    }

    if (btn_minus == LOW && btn_minus_last == HIGH) {
      if (OP_ON && set_speed > 10) {
        set_speed -= 5;
        if (set_speed < MIN_SET_SPEED_KPH) set_speed = MIN_SET_SPEED_KPH;
      } 
    }

    if (OP_ON && btn_plus == LOW && btn_plus_last == HIGH) {
      if (set_speed < MAX_SET_SPEED_KPH) {
        set_speed += 5;
        if (set_speed > MAX_SET_SPEED_KPH) set_speed = MAX_SET_SPEED_KPH;
      }
    }

    if (btn_cancel == LOW && btn_cancel_last == HIGH) OP_ON = false;

    btn_main_last = btn_main;
    btn_plus_last = btn_plus;
    btn_minus_last = btn_minus;
    btn_cancel_last = btn_cancel;
  }

  // 智能门控自动 engage：openpilot 就绪(累计收到足够 0x2E4) 且 真实 EPS 已连续处于
  // 健康 standby(LKA_STATE==1) 稳定 ≥ AUTO_ENGAGE_EPS_READY_MS，当前未挂、无刹车/油门
  // 时一次性触发。EPS 处于故障(如打火后латch 的 17)时 eps_ready_since 恒为 0 -> 不会自动挂。
  boolean eps_ready = (eps_ready_since != 0) &&
                      ((millis() - eps_ready_since) >= AUTO_ENGAGE_EPS_READY_MS);
  if (AUTO_ENGAGE_ON_OP && !auto_engaged_done && !OP_ON &&
      !BRAKE_PRESSED && GAS_RELEASED &&
      rx_2e4_count >= AUTO_ENGAGE_MIN_2E4 &&
      eps_ready) {
    OP_ON = true;
    set_speed = clamp_set_speed(get_ui_speed_kph() + 3.0f);
    auto_engaged_done = true;
  }

  if (OP_ON) cruise_state = 8;
  else cruise_state = 7;

  // 50Hz: PCM_CRUISE / PCM_CRUISE_2 / GAS_PEDAL / BRAKE_MODULE
  if (now - last_20ms_tick >= 20) {
    last_20ms_tick = now;

    uint8_t dat_1d2[8] = {0};
    dat_1d2[0] = (OP_ON ? 0x20 : 0x00) | (GAS_RELEASED ? 0x10 : 0x00);
    dat_1d2[6] = (cruise_state & 0x0F) << 4;  // CRUISE_STATE nibble
    dat_1d2[7] = toyota_checksum(dat_1d2, 7, 0x1D2);
    can_send_8(0x1D2, dat_1d2);

    uint8_t dat_1d3[8] = {0};
    dat_1d3[1] = (MAIN_ON ? 0x80 : 0x00) | 0x28;  // MAIN_ON + Toyota baseline bits
    dat_1d3[2] = set_speed;                // SET_SPEED
    dat_1d3[7] = toyota_checksum(dat_1d3, 7, 0x1D3);
    can_send_8(0x1D3, dat_1d3);

    uint8_t dat_2c1[8] = {0};
    dat_2c1[0] = (GAS_RELEASED ? 0x08 : 0x00);  // GAS_RELEASED bit
    can_send_8(0x2C1, dat_2c1);

    uint8_t dat_224[8] = {0};
    dat_224[0] = (BRAKE_PRESSED ? 0x20 : 0x00); // BRAKE_PRESSED bit
    can_send_8(0x224, dat_224);
  }

  // 83Hz: WHEEL_SPEEDS
  if (now - last_12ms_tick >= 12) {
    last_12ms_tick = now;

    float wheel_spd_kph = get_ui_speed_kph();
    wheel_spd_kph += TOYOTA_WHEEL_SPEED_SIGNAL_OFFSET_KPH;
    if (wheel_spd_kph < 0.0f) wheel_spd_kph = 0.0f;
    if (wheel_spd_kph > 655.35f) wheel_spd_kph = 655.35f;
    uint16_t raw = (uint16_t)(wheel_spd_kph * 100.0f);

    uint8_t dat_aa[8] = {0};
    dat_aa[0] = (raw >> 8) & 0xFF;
    dat_aa[1] = raw & 0xFF;
    dat_aa[2] = (raw >> 8) & 0xFF;
    dat_aa[3] = raw & 0xFF;
    dat_aa[4] = (raw >> 8) & 0xFF;
    dat_aa[5] = raw & 0xFF;
    dat_aa[6] = (raw >> 8) & 0xFF;
    dat_aa[7] = raw & 0xFF;
    can_send_8(0xAA, dat_aa);
  }

  // 40Hz: EPS_STATUS (0x262) with valid LKA_STATE.
  // Corolla DBC: standby=1, active=5, temporary_fault=9/25
  // openpilot checks this signal at 25ms intervals.
  // 仅在没有真实 EPS 时才伪造；真实 EPS 在场时必须关闭以避免 0x262 ID 冲突。
  if (EMIT_FAKE_EPS_STATUS && (now - last_25ms_tick >= 25)) {
    last_25ms_tick = now;
    uint8_t dat_262[5] = {0};
    uint8_t lka_state = OP_ON ? 5 : 1;
    dat_262[3] = (uint8_t)(lka_state << 1);  // DBC pack: LKA_STATE at 31|7@0+
    dat_262[4] = toyota_checksum(dat_262, 4, 0x262);
    can_send_n(0x262, dat_262, 5);
  }

  // 20Hz: radar keepalive
  if (now - last_50ms_tick >= 50) {
    last_50ms_tick = now;
    uint8_t dat_430[8] = {0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00};
    dat_430[7] = toyota_checksum(dat_430, 7, 0x430);
    can_send_8(0x430, dat_430);
  }

  // 10Hz: low-rate + fallback body messages
  if (now - last_100ms_tick >= 100) {
    last_100ms_tick = now;

    uint8_t dat_3b7[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08};
    can_send_8(0x3B7, dat_3b7);

    if (EMIT_BCM_FALLBACK) {
      // 0x620 BODY_CONTROL_STATE: all doors closed, seatbelt latched, parking brake off
      uint8_t dat_620[8] = {0};
      can_send_8(0x620, dat_620);

      // 0x614 BLINKERS_STATE: TURN_SIGNALS=3 (none), hazard=0
      uint8_t dat_614[8] = {0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00};
      can_send_8(0x614, dat_614);

      // 0x622 LIGHT_STALK: AUTO_HIGH_BEAM=0
      uint8_t dat_622[8] = {0};
      can_send_8(0x622, dat_622);
    }

    uint8_t dat_3bc[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00};
    can_send_8(0x3BC, dat_3bc);

    // 调试心跳 0x701：上报 Arduino 的 RX 状态，便于在 EON 侧判断收包是否工作
    uint8_t hb[8] = {0};
    hb[0] = (rx_any_count > 0) ? 1 : 0;   // 是否收到过任意帧
    hb[1] = (rx_2e4_count > 0) ? 1 : 0;   // 是否收到过 openpilot 0x2E4
    hb[2] = op_steer_req ? 1 : 0;         // openpilot 是否 active
    hb[3] = auto_engaged_done ? 1 : 0;    // 是否已自动 engage
    hb[4] = OP_ON ? 1 : 0;                // 当前 OP_ON
    hb[5] = (uint8_t)(rx_any_count & 0xFF);
    hb[6] = (uint8_t)(rx_2e4_count & 0xFF);
    hb[7] = (op_seen_since != 0) ? 1 : 0; // openpilot 是否判定为稳定在线
    can_send_8(0x701, hb);

    // 诊断帧 0x702：上报 Arduino 收到过哪些关键地址，以及真实 EPS 当前状态与
    // 自动 engage 门控是否就绪。
    uint8_t hb2[8] = {0};
    hb2[0] = ever_2e4 ? 1 : 0;
    hb2[1] = ever_25 ? 1 : 0;
    hb2[2] = ever_260 ? 1 : 0;
    hb2[3] = ever_262 ? 1 : 0;
    hb2[4] = ever_343 ? 1 : 0;   // openpilot ACC_CONTROL 收到？
    hb2[5] = ever_412 ? 1 : 0;   // openpilot 0x412 收到？
    hb2[6] = eps_lka_state;      // 真实 EPS LKA_STATE(7位解码)：1=standby 5=active 17=fault
    hb2[7] = ((eps_ready_since != 0) &&
              ((millis() - eps_ready_since) >= AUTO_ENGAGE_EPS_READY_MS)) ? 1 : 0; // 门控就绪
    can_send_8(0x702, hb2);
  }

  // openpilot 的 0x2E4 解析已移到 pump_rx()，并在每次 TX 后及 loop 顶部调用，
  // 避免 MCP2515 两级 RX 缓冲在繁忙总线上溢出而整类丢掉 0x2E4。
  pump_rx();

  // 维护 openpilot 在线状态（供自动 engage 去抖使用）
  if (op_last_seen != 0 && (now - op_last_seen) < OP_ALIVE_TIMEOUT_MS) {
    if (op_seen_since == 0) op_seen_since = now;
  } else {
    op_seen_since = 0;
  }
}
