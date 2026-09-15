/**
 * ============================================================
 *  LoRa Scanner / Decoder  —  Heltec WiFi LoRa 32 V2
 * ============================================================
 */

#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include "SSD1306Wire.h"

// ============================================================
//  硬件引脚定义 (Heltec WiFi LoRa 32 V2)
// ============================================================
#define OLED_RST  16
#define OLED_SDA  21
#define OLED_SCL  22

#define LORA_SCK   5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS   18
#define LORA_RST  14
#define LORA_DIO0 26

#define BTN_PIN    0

// OLED 实例 (0x3c, SDA=21, SCL=22)
SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL, GEOMETRY_128_64);

// ============================================================
//  可调参数与表定义
// ============================================================
static const unsigned long SCAN_INTERVAL_MS = 800;
static const unsigned long DEBOUNCE_MS      = 50;
static const unsigned long LONG_PRESS_MS    = 1500;
static const unsigned long RSSI_SAMPLE_MS   = 50;
static const unsigned long DRAW_MS          = 100;

static const long FREQ_LIST[] = {
  435000000L,
  435500000L,
  436000000L,
  438125000L,
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

#define WF_Y   20
#define WF_H   44
#define WF_W  128
#define RSSI_MIN (-150)
#define RSSI_MAX  (-30)

// ============================================================
//  全局状态
// ============================================================
static int  freqIdx    = 3;
static int  comboIdx   = 0;
static bool locked     = false;

static String  lastPayload = "";
static int     lastRSSI    = 0;
static int     lastSNR     = 0;
static bool    newPacket   = false;

static float rssiHistory[WF_W];
static int   rssiWrIdx = 0;

static unsigned long lastSwitchMs   = 0;
static unsigned long lastRSSISample = 0;
static unsigned long lastDrawMs     = 0;

static unsigned long btnPressMs  = 0;
static bool          btnHeld     = false;

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
  LoRa.disableCrc();
  LoRa.receive();

  Serial.printf("[CFG] %.3f MHz | SF%d | CR4/%d | BW:%ldkHz | SYNC:0x%02X\n",
    FREQ_LIST[freqIdx] / 1e6, curSF(), curCR(), BW / 1000, MESHTASTIC_SYNC);
}

// ============================================================
//  LoRa 接收回调
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
    if (++byteCount >= 64) { lastPayload += "..."; break; }
  }

  lastRSSI  = LoRa.packetRssi();
  lastSNR   = (int)LoRa.packetSnr();
  newPacket = true;
  locked    = true;

  Serial.printf("[RX] %.3f MHz | SF%d CR4/%d | RSSI:%d SNR:%d | len:%d\n",
    FREQ_LIST[freqIdx] / 1e6, curSF(), curCR(), lastRSSI, lastSNR, byteCount);
  Serial.printf("     %s\n", lastPayload.c_str());
}

void sampleRSSI() {
  rssiHistory[rssiWrIdx] = LoRa.packetRssi();
  rssiWrIdx = (rssiWrIdx + 1) % WF_W;
}

// ============================================================
//  OLED 绘制
// ============================================================
void drawDisplay() {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  char buf[32];
  snprintf(buf, sizeof(buf), "%.3fM", FREQ_LIST[freqIdx] / 1e6);
  display.drawString(0, 0, buf);
  snprintf(buf, sizeof(buf), "SF%d CR4/%d", curSF(), curCR());
  display.drawString(75, 0, buf);

  if (locked) {
    snprintf(buf, sizeof(buf), "*LOCK* R:%d S:%d", lastRSSI, lastSNR);
  } else {
    snprintf(buf, sizeof(buf), "SCAN %d/%d BW:125k", comboIdx + 1, COMBO_COUNT);
  }
  display.drawString(0, 10, buf);

  for (int col = 0; col < WF_W; col++) {
    int idx  = (rssiWrIdx + col) % WF_W;
    int h    = (int)map((long)rssiHistory[idx], RSSI_MIN, RSSI_MAX, 0, WF_H);
    h = constrain(h, 0, WF_H);
    if (h > 0) {
      display.drawVerticalLine(col, WF_Y + WF_H - h, h);
    }
  }

  if (locked && lastPayload.length() > 0) {
    String preview = ">" + lastPayload.substring(0, 18);
    display.drawString(0, 54, preview);
  }

  display.display();
}

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
      freqIdx    = (freqIdx + 1) % FREQ_COUNT;
      comboIdx   = 0;
      locked     = false;
      lastPayload = "";
      applyLoRaConfig();
      Serial.printf("[BTN] 长按 → %.3f MHz\n", FREQ_LIST[freqIdx] / 1e6);
    } else {
      locked     = false;
      lastPayload = "";
      comboIdx   = (comboIdx + 1) % COMBO_COUNT;
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

  // 初始化 OLED 复位引脚
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);

  // 初始化 OLED 屏幕
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0,  0, "LoRa Scanner v2");
  display.drawString(0, 14, "Heltec WiFi LoRa 32 V2");
  display.drawString(0, 28, "Target: Meshtastic");
  display.drawString(0, 42, "Sync: 0x2B  BW: 125k");
  display.display();

  pinMode(BTN_PIN, INPUT_PULLUP);

  // 初始化 SPI & LoRa 引脚
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(FREQ_LIST[freqIdx])) {
    Serial.println("[ERROR] LoRa init failed!");
    while (1);
  }

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

void loop() {
  LoRa.parsePacket();

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
  }
}
