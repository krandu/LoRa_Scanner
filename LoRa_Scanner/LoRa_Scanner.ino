/**
 * ============================================================
 *  LoRa Scanner / Decoder  —  Heltec WiFi LoRa 32 V2
 * ============================================================
 *
 *  基于 OpenWebRX 瀑布图观测结果优化：
 *    · 确认目标信号为 LoRa / Meshtastic（125 kHz 实心矩形块）
 *    · 频段：435~436 MHz（弱），438.125 MHz（强）
 *    · 优先扫描 SF11/SF12（长包持续时间长，与图吻合）
 *    · Meshtastic 同步字 0x2B
 *
 *  按键操作（板载 PRG / GPIO0）：
 *    · 长按 ≥ 1.5 s  →  循环切换频率
 *    · 短按 < 1.5 s  →  解锁 / 手动切换下一个 SF·CR 组合
 *
 *  OLED 布局（128×64）：
 *    行0  频率(MHz)          SF·CR
 *    行1  SCAN n/N BW:125k  或  LOCK RSSI:-xx
 *    行2～63  RSSI 瀑布热力图（滚动）
 *    行54 解码成功时显示 Payload 前缀
 * ============================================================
 */

#include "heltec.h"

// ============================================================
//  可调参数
// ============================================================

// 扫描间隔：每个 SF/CR 组合停留时长 (ms)
static const unsigned long SCAN_INTERVAL_MS = 800;

// 按键消抖 / 长短按判定
static const unsigned long DEBOUNCE_MS    =   50;
static const unsigned long LONG_PRESS_MS  = 1500;

// RSSI 采样周期 (ms)
static const unsigned long RSSI_SAMPLE_MS =   50;

// OLED 刷新周期 (ms)
static const unsigned long DRAW_MS        =  100;

// ============================================================
//  频率表  (Hz)  —  根据瀑布图观测结果
// ============================================================
static const long FREQ_LIST[] = {
  435000000L,   // 435.0 MHz  弱信号区起点
  435500000L,   // 435.5 MHz  弱信号中心（第一张图）
  436000000L,   // 436.0 MHz  弱信号右侧亮斑
  438125000L,   // 438.125 MHz ★ 强 Meshtastic 信号（第三张图）
  433000000L,   // 433.0 MHz  ISM 常用
  434000000L,   // 434.0 MHz  ISM 常用
};
static const int FREQ_COUNT = (int)(sizeof(FREQ_LIST) / sizeof(FREQ_LIST[0]));

// ============================================================
//  SF / CR 扫描表  —  Meshtastic 常用组合优先
// ============================================================
struct Combo { uint8_t sf; uint8_t cr; };

static const Combo COMBO_LIST[] = {
  // ★ Meshtastic 优先组合
  {11, 8},  // LongSlow  (SF11 BW125 CR4/8)
  {12, 8},  // VeryLongSlow
  {11, 5},  // LongFast  (SF11 BW250, 此处 BW=125)
  {10, 5},  // MediumSlow
  {10, 8},
  { 9, 5},
  { 9, 8},
  // 补充常规组合
  { 7, 5},
  { 7, 8},
  { 8, 5},
  { 8, 8},
  {12, 5},
  {12, 6},
  {11, 6},
  {10, 6},
  { 9, 6},
};
static const int COMBO_COUNT = (int)(sizeof(COMBO_LIST) / sizeof(COMBO_LIST[0]));

// Meshtastic 私有同步字
static const uint8_t MESHTASTIC_SYNC = 0x2B;

// 带宽固定 125 kHz（与瀑布图观测一致）
static const long BW = 125000L;

// 按键引脚
#define BTN_PIN 0

// OLED 瀑布图区域
#define WF_Y   20
#define WF_H   44
#define WF_W  128
#define RSSI_MIN (-150)
#define RSSI_MAX  (-30)

// ============================================================
//  全局状态
// ============================================================
static int  freqIdx    = 3;     // 默认 438.125 MHz（最强信号）
static int  comboIdx   = 0;
static bool locked     = false;

// RX 结果
static String  lastPayload = "";
static int     lastRSSI    = 0;
static int     lastSNR     = 0;
static bool    newPacket   = false;

// 瀑布图（RSSI 历史循环缓冲）
static float rssiHistory[WF_W];
static int   rssiWrIdx = 0;

// 计时器
static unsigned long lastSwitchMs   = 0;
static unsigned long lastRSSISample = 0;
static unsigned long lastDrawMs     = 0;

// 按键状态
static unsigned long btnPressMs  = 0;
static bool          btnHeld     = false;

// ============================================================
//  辅助：取当前 SF / CR
// ============================================================
inline int curSF() { return COMBO_LIST[comboIdx].sf; }
inline int curCR() { return COMBO_LIST[comboIdx].cr; }

// ============================================================
//  LoRa 配置下发
// ============================================================
void applyLoRaConfig() {
  LoRa.idle();
  LoRa.setSyncWord(MESHTASTIC_SYNC);
  LoRa.setFrequency(FREQ_LIST[freqIdx]);
  LoRa.setSignalBandwidth(BW);
  LoRa.setSpreadingFactor(curSF());
  LoRa.setCodingRate4(curCR());
  LoRa.setPreambleLength(8);
  LoRa.disableCrc();   // 先关 CRC，提高捕获率
  LoRa.receive();

  Serial.printf("[CFG] %.3f MHz | SF%d | CR4/%d | BW:%ldkHz | SYNC:0x%02X\n",
    FREQ_LIST[freqIdx] / 1e6, curSF(), curCR(), BW / 1000, MESHTASTIC_SYNC);
}

// ============================================================
//  LoRa 接收回调
// ============================================================
void onReceive(int packetSize) {
  if (packetSize == 0) return;

  // 读取 payload（HEX 格式，兼容二进制 Meshtastic 帧）
  lastPayload = "";
  char hexBuf[4];
  int byteCount = 0;
  while (LoRa.available()) {
    uint8_t b = (uint8_t)LoRa.read();
    // 先尝试可打印字符，否则 HEX
    if (b >= 0x20 && b < 0x7F) {
      lastPayload += (char)b;
    } else {
      snprintf(hexBuf, sizeof(hexBuf), "%02X ", b);
      lastPayload += hexBuf;
    }
    if (++byteCount >= 64) { lastPayload += "..."; break; }  // 截断保护
  }

  lastRSSI  = LoRa.packetRssi();
  lastSNR   = (int)LoRa.packetSnr();
  newPacket = true;
  locked    = true;   // 自动锁定

  Serial.printf("[RX] %.3f MHz | SF%d CR4/%d | RSSI:%d SNR:%d | len:%d\n",
    FREQ_LIST[freqIdx] / 1e6, curSF(), curCR(), lastRSSI, lastSNR, byteCount);
  Serial.printf("     %s\n", lastPayload.c_str());
}

// ============================================================
//  RSSI 采样 → 瀑布图缓冲
// ============================================================
void sampleRSSI() {
  rssiHistory[rssiWrIdx] = LoRa.packetRssi();
  rssiWrIdx = (rssiWrIdx + 1) % WF_W;
}

// ============================================================
//  OLED 绘制
// ============================================================
void drawDisplay() {
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);

  // 行 0：频率 + SF/CR
  char buf[32];
  snprintf(buf, sizeof(buf), "%.3fM", FREQ_LIST[freqIdx] / 1e6);
  Heltec.display->drawString(0, 0, buf);
  snprintf(buf, sizeof(buf), "SF%d CR4/%d", curSF(), curCR());
  Heltec.display->drawString(75, 0, buf);

  // 行 1：状态
  if (locked) {
    snprintf(buf, sizeof(buf), "*LOCK* R:%d S:%d", lastRSSI, lastSNR);
  } else {
    snprintf(buf, sizeof(buf), "SCAN %d/%d BW:125k", comboIdx + 1, COMBO_COUNT);
  }
  Heltec.display->drawString(0, 10, buf);

  // 瀑布图：每列 = 一次 RSSI 历史，高度映射信号强度
  for (int col = 0; col < WF_W; col++) {
    int idx  = (rssiWrIdx + col) % WF_W;
    int h    = (int)map((long)rssiHistory[idx], RSSI_MIN, RSSI_MAX, 0, WF_H);
    h = constrain(h, 0, WF_H);
    if (h > 0) {
      Heltec.display->drawVerticalLine(col, WF_Y + WF_H - h, h);
    }
  }

  // 行 54：Payload 预览（锁定时）
  if (locked && lastPayload.length() > 0) {
    String preview = ">" + lastPayload.substring(0, 18);
    Heltec.display->drawString(0, 54, preview);
  }

  Heltec.display->display();
}

// ============================================================
//  按键处理
// ============================================================
void handleButton() {
  bool pressed = (digitalRead(BTN_PIN) == LOW);

  if (pressed && !btnHeld) {
    btnPressMs = millis();
    btnHeld    = true;
  }

  if (!pressed && btnHeld) {
    unsigned long dur = millis() - btnPressMs;
    btnHeld = false;

    if (dur < DEBOUNCE_MS) return;  // 抖动忽略

    if (dur >= LONG_PRESS_MS) {
      // ── 长按：切换频率，重置扫描 ──
      freqIdx    = (freqIdx + 1) % FREQ_COUNT;
      comboIdx   = 0;
      locked     = false;
      lastPayload = "";
      applyLoRaConfig();
      Serial.printf("[BTN] 长按 → %.3f MHz\n", FREQ_LIST[freqIdx] / 1e6);
    } else {
      // ── 短按：手动下一组合 / 解锁 ──
      locked     = false;
      lastPayload = "";
      comboIdx   = (comboIdx + 1) % COMBO_COUNT;
      applyLoRaConfig();
      Serial.printf("[BTN] 短按 → SF%d CR4/%d\n", curSF(), curCR());
    }
    lastSwitchMs = millis();
  }
}

// ============================================================
//  自动扫描
// ============================================================
void autoScan() {
  if (locked) return;
  if (millis() - lastSwitchMs >= SCAN_INTERVAL_MS) {
    lastSwitchMs = millis();
    comboIdx = (comboIdx + 1) % COMBO_COUNT;
    applyLoRaConfig();
  }
}

// ============================================================
//  setup
// ============================================================
void setup() {
  Serial.begin(115200);

  Heltec.begin(/*display*/true, /*LoRa*/true, /*Serial*/true,
               /*PABOOST*/true, FREQ_LIST[freqIdx]);
  Heltec.display->flipScreenVertically();

  // 启动画面
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0,  0, "LoRa Scanner v2");
  Heltec.display->drawString(0, 14, "Heltec WiFi LoRa 32 V2");
  Heltec.display->drawString(0, 28, "Target: Meshtastic");
  Heltec.display->drawString(0, 42, "Sync: 0x2B  BW: 125k");
  Heltec.display->display();

  pinMode(BTN_PIN, INPUT_PULLUP);

  // 清零 RSSI 历史
  for (int i = 0; i < WF_W; i++) rssiHistory[i] = RSSI_MIN;

  LoRa.onReceive(onReceive);
  applyLoRaConfig();

  lastSwitchMs = millis();

  Serial.println("========================================");
  Serial.println(" LoRa Scanner  —  Heltec WiFi LoRa 32 V2");
  Serial.println(" 长按 PRG : 切换频率");
  Serial.println(" 短按 PRG : 切换 SF/CR 或解锁");
  Serial.println("========================================");

  delay(1500);
}

// ============================================================
//  loop
// ============================================================
void loop() {
  LoRa.parsePacket();   // 非阻塞接收

  handleButton();
  autoScan();

  unsigned long now = millis();

  if (now - lastRSSISample >= RSSI_SAMPLE_MS) {
    lastRSSISample = now;
    sampleRSSI();
  }

  if (now - lastDrawMs >= DRAW_MS) {
    lastDrawMs = now;
    drawDisplay();
  }

  if (newPacket) {
    newPacket = false;
    // 已在 onReceive() 中打印，此处可扩展（写 SD、发 MQTT 等）
  }
}
