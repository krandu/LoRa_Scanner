/**
 * ============================================================
 *  LoRa Scanner / Spectrum Analyzer  —  Heltec WiFi LoRa 32 V2
 * ============================================================
 *  硬件管脚映射:
 *    · VEXT 电源控制 : GPIO 21 (低电平开启 3.3V 供电)
 *    · OLED SDA      : GPIO 4
 *    · OLED SCL      : GPIO 15
 *    · OLED RST      : GPIO 16
 *    · LoRa CS/RST/D0: GPIO 18, 14, 26
 *    · PRG 按键      : GPIO 0
 * ============================================================
 */

#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include "SSD1306Wire.h"

// ============================================================
//  硬件引脚定义 (Heltec WiFi LoRa 32 V2)
// ============================================================
#define VEXT_PIN  21  // 外部 3.3V 供电开关 (低电平开启)
#define OLED_RST  16  // OLED 硬件复位引脚
#define OLED_SDA   4  // 独立 I2C SDA 引脚
#define OLED_SCL  15  // 独立 I2C SCL 引脚

#define LORA_SCK   5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS   18
#define LORA_RST  14
#define LORA_DIO0 26

#define BTN_PIN    0  // 板载 PRG 按键

// SSD1306Wire 屏幕实例 (I2C 地址 0x3C)
SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL, GEOMETRY_128_64);

// ============================================================
//  UI 频谱图框布局常量
// ============================================================
#define BOX_X          8
#define BOX_Y         20
#define BOX_W        112
#define BOX_H         32
#define RSSI_HIST_LEN 112

// ============================================================
//  扫描与配置表
// ============================================================
static const unsigned long SCAN_INTERVAL_MS = 800;
static const unsigned long DEBOUNCE_MS      = 50;
static const unsigned long LONG_PRESS_MS    = 1500;
static const unsigned long RSSI_SAMPLE_MS   = 30;
static const unsigned long DRAW_MS          = 80;

static const long FREQ_LIST[] = {
  438125000L,
  435000000L,
  435500000L,
  436000000L,
  433000000L,
  434000000L,
};
static const int FREQ_COUNT = (int)(sizeof(FREQ_LIST) / sizeof(FREQ_LIST[0]));

struct Combo { uint8_t sf; uint8_t cr; };
static const Combo COMBO_LIST[] = {
  {11, 8}, {12, 8}, {11, 5}, {10, 5},
  {10, 8}, { 9, 5}, { 9, 8}, { 7, 5},
  { 7, 8}, { 8, 5}, { 8, 8}, {12, 5},
  {12, 6}, {11, 6}, {10, 6}, { 9, 6},
};
static const int COMBO_COUNT = (int)(sizeof(COMBO_LIST) / sizeof(COMBO_LIST[0]));

static const uint8_t MESHTASTIC_SYNC = 0x2B;
static const long BW = 125000L;

// ============================================================
//  全局状态
// ============================================================
static int  freqIdx    = 0;
static int  comboIdx   = 0;
static bool locked     = false;

static String  lastPayload = "";
static int     lastRSSI    = 0;
static int     lastSNR     = 0;

static float rssiHistory[RSSI_HIST_LEN];
static int   rssiWrIdx       = 0;
static float baseNoiseFloor  = -120.0; // 动态底噪基准 (dBm)

static unsigned long lastSwitchMs   = 0;
static unsigned long lastRSSISample = 0;
static unsigned long lastDrawMs     = 0;

static unsigned long btnPressMs  = 0;
static bool          btnHeld     = false;

inline int curSF() { return COMBO_LIST[comboIdx].sf; }
inline int curCR() { return COMBO_LIST[comboIdx].cr; }

// ============================================================
//  OLED 供电与硬件初始化
// ============================================================
void initOLED() {
  // 1. 开启 Vext 供电 (低电平开启 3.3V)
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);
  delay(100);

  // 2. 硬件复位 OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  delay(50);

  // 3. 初始化 Wire 总线与屏幕
  Wire.begin(OLED_SDA, OLED_SCL);
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0,  0, "LoRa Spectrum Scanner");
  display.drawString(0, 14, "Heltec WiFi LoRa 32 V2");
  display.drawString(0, 28, "Target: Meshtastic");
  display.drawString(0, 42, "Sync: 0x2B  BW: 125k");
  display.display();
}

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
  LoRa.disableCrc();
  LoRa.receive();

  Serial.printf("[CFG] %.3f MHz | SF%d | CR4/%d | BW:%ldkHz | SYNC:0x%02X\n",
    FREQ_LIST[freqIdx] / 1e6, curSF(), curCR(), BW / 1000, MESHTASTIC_SYNC);
}

// ============================================================
//  LoRa 接收中断/回调
// ============================================================
void onReceive(int packetSize) {
  if (packetSize == 0) return;

  lastPayload = "";
  char hexBuf[4];
  int byteCount = 0;
  while (LoRa.available()) {
    uint8_t b = (uint8_t)LoRa.read();
    if (b >= 0x20 && b < 0x7F) {
      lastPayload += (char)b;
    } else {
      snprintf(hexBuf, sizeof(hexBuf), "%02X ", b);
      lastPayload += hexBuf;
    }
    if (++byteCount >= 32) { lastPayload += "..."; break; }
  }

  lastRSSI = LoRa.packetRssi();
  lastSNR  = (int)LoRa.packetSnr();
  locked   = true;

  Serial.printf("[RX] %.3f MHz | SF%d CR4/%d | RSSI:%d SNR:%d | len:%d\n",
    FREQ_LIST[freqIdx] / 1e6, curSF(), curCR(), lastRSSI, lastSNR, byteCount);
  Serial.printf("     %s\n", lastPayload.c_str());
}

// ============================================================
//  环境 RSSI 实时采样与底噪更新
// ============================================================
void sampleRSSI() {
  int rawRssi = LoRa.rssi(); // 实时读取当前信道的环境电平

  // 动态跟踪最低底噪 (指数平滑)
  if (rawRssi < baseNoiseFloor) {
    baseNoiseFloor = rawRssi; 
  } else {
    baseNoiseFloor = baseNoiseFloor * 0.998 + rawRssi * 0.002;
  }

  rssiHistory[rssiWrIdx] = rawRssi;
  rssiWrIdx = (rssiWrIdx + 1) % RSSI_HIST_LEN;
}

// ============================================================
//  OLED 界面与频谱框绘制
// ============================================================
void drawDisplay() {
  display.clear();
  display.setFont(ArialMT_Plain_10);
  char buf[32];

  // 第一行：当前频率与参数
  snprintf(buf, sizeof(buf), "%.3fM SF%d C4/%d", FREQ_LIST[freqIdx] / 1e6, curSF(), curCR());
  display.drawString(0, 0, buf);

  // 右上角：接收锁定状态或当前底噪
  if (locked) {
    snprintf(buf, sizeof(buf), "*LOCK*");
  } else {
    snprintf(buf, sizeof(buf), "N:%.0f", baseNoiseFloor);
  }
  display.drawString(92, 0, buf);

  // 1. 绘制长方形频谱外框
  display.drawRect(BOX_X - 1, BOX_Y - 1, BOX_W + 2, BOX_H + 2);

  // 2. 在框内绘制以底噪为基准的相对频谱图
  for (int col = 0; col < BOX_W; col++) {
    int idx = (rssiWrIdx + col) % RSSI_HIST_LEN;
    float val = rssiHistory[idx];

    // 计算高出底噪的相对增量 (0~25dB 动态映射)
    float delta = val - baseNoiseFloor;
    if (delta < 0) delta = 0;

    int lineH = (int)map((long)(delta * 10), 0, 250, 0, BOX_H);
    lineH = constrain(lineH, 0, BOX_H);

    if (lineH > 0) {
      display.drawVerticalLine(BOX_X + col, BOX_Y + BOX_H - lineH, lineH);
    }
  }

  // 3. 长方形底部左右两侧标注中心频点两侧的扫描起始/截止频率 (以 125kHz 带宽算)
  float startFreq = (FREQ_LIST[freqIdx] - BW / 2) / 1e6;
  float endFreq   = (FREQ_LIST[freqIdx] + BW / 2) / 1e6;

  snprintf(buf, sizeof(buf), "%.2f", startFreq);
  display.drawString(0, 53, buf);

  snprintf(buf, sizeof(buf), "%.2f", endFreq);
  display.drawString(98, 53, buf);

  // 4. 底部中间：显示数据包预览或信号强度
  if (locked && lastPayload.length() > 0) {
    String preview = ">" + lastPayload.substring(0, 8);
    display.drawString(34, 53, preview);
  } else if (locked) {
    snprintf(buf, sizeof(buf), "R:%d S:%d", lastRSSI, lastSNR);
    display.drawString(34, 53, buf);
  }

  display.display();
}

// ============================================================
//  按键与扫描逻辑
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

    if (dur < DEBOUNCE_MS) return;

    if (dur >= LONG_PRESS_MS) {
      // 长按：切换频率
      freqIdx     = (freqIdx + 1) % FREQ_COUNT;
      comboIdx    = 0;
      locked      = false;
      lastPayload = "";
      applyLoRaConfig();
      Serial.printf("[BTN] 长按 → %.3f MHz\n", FREQ_LIST[freqIdx] / 1e6);
    } else {
      // 短按：切换 SF/CR 组合或解锁
      locked      = false;
      lastPayload = "";
      comboIdx    = (comboIdx + 1) % COMBO_COUNT;
      applyLoRaConfig();
      Serial.printf("[BTN] 短按 → SF%d CR4/%d\n", curSF(), curCR());
    }
    lastSwitchMs = millis();
  }
}

void autoScan() {
  if (locked) return;
  if (millis() - lastSwitchMs >= SCAN_INTERVAL_MS) {
    lastSwitchMs = millis();
    comboIdx = (comboIdx + 1) % COMBO_COUNT;
    applyLoRaConfig();
  }
}

// ============================================================
//  setup & loop
// ============================================================
void setup() {
  Serial.begin(115200);

  initOLED();

  pinMode(BTN_PIN, INPUT_PULLUP);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(FREQ_LIST[freqIdx])) {
    Serial.println("[ERROR] LoRa init failed!");
    while (1);
  }

  for (int i = 0; i < RSSI_HIST_LEN; i++) rssiHistory[i] = baseNoiseFloor;

  LoRa.onReceive(onReceive);
  applyLoRaConfig();

  lastSwitchMs = millis();
  delay(1500);
}

void loop() {
  LoRa.parsePacket();

  handleButton();
  autoScan();

  unsigned long now = millis();

  // 1. 实时读取 RSSI
  if (now - lastRSSISample >= RSSI_SAMPLE_MS) {
    lastRSSISample = now;
    sampleRSSI();
  }

  // 2. 刷新屏幕显示
  if (now - lastDrawMs >= DRAW_MS) {
    lastDrawMs = now;
    drawDisplay();
  }
}
