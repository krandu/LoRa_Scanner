#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_sleep.h>
#include <esp_adc_cal.h>
#include <Preferences.h>  // 使用 NVS 闪存存储校准系数

// ================= 硬件引脚配置 =================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

#define VEXT_CTRL_PIN 21   // 控制屏幕及电池分压电路供电 (LOW 为激活)
#define OLED_SDA      4
#define OLED_SCL      15
#define OLED_RST      16
#define VBAT_ADC_PIN  37   // Heltec V2 板载电池采样引脚 (ADC1_CH1)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
Preferences prefs;         // NVS 存储对象
esp_adc_cal_characteristics_t adc_chars;

#define SCK_PIN   5
#define MISO_PIN  19
#define MOSI_PIN  27
#define SS_PIN    18
#define RST_PIN   14
#define DIO0_PIN  26

#define PRG_BUTTON_PIN 0

// ================= NVS 闪存校准参数 =================
float vbatCalFactor = 4.90f; // 默认分压校准系数

// ================= 定时与休眠配置 =================
const unsigned long AUTO_POWER_OFF_MS = 10 * 60 * 1000UL; // 10分钟无操作自动关机
unsigned long lastActivityMs = 0;

// ================= 模式与状态 =================
enum SystemMode { MODE_SPECTRUM, MODE_LORA_ANALYZER };
SystemMode currentMode = MODE_SPECTRUM;

// 跨度/步进结构
struct SpectrumSpanOption {
  float spanMHz;
  float stepkHz;
};
SpectrumSpanOption spanOptions[] = {
  {10.0, 250.0}, // 全景扫描
  {5.0,  125.0}, // 中等细化
  {2.0,  50.0}   // 高精窄带细化
};
const int SPAN_COUNT = sizeof(spanOptions) / sizeof(spanOptions[0]);
int spanIdx = 0;

// 中心频率与扫描范围控制
float specCenterFreq = 435.0;
float specStartFreq  = 430.0;
float specEndFreq    = 440.0;
#define SPEC_CHANNELS    40

#define SPEC_BOX_X       3
#define SPEC_BOX_Y       11
#define SPEC_BOX_W       122
#define SPEC_BOX_H       18

#define WATERFALL_X      3
#define WATERFALL_Y      32
#define WATERFALL_W      122
#define WATERFALL_H      20

float specRssi[SPEC_CHANNELS];
uint8_t waterfallBuf[WATERFALL_H - 2][SPEC_CHANNELS];

// ===== 频谱平滑与门限 =====
float specSmooth[SPEC_CHANNELS];
bool  specInit = false;
bool  peakValid = false;
bool  displayedPeakValid = false;

const float PEAK_MIN_SNR   = 8.0;

float rawPeakRssi = -160.0;
float rawPeakFreq = 435.0;
float displayedPeakFreq = 435.0;
float displayedPeakRssi = -160.0;
unsigned long lastPeakUpdateMs = 0;

float smoothedNoiseFloor = -100.0;
float displayedNoiseFloor = -100.0;
unsigned long lastNoiseUpdateMs = 0;
const unsigned long NOISE_HOLD_TIME_MS = 1500;

float maxFoundRssi = -160.0;
float minFoundRssi = 0.0;

// ================= LoRa 分析模式变量 =================
float currentFreq = 438.150; // 当前接收频率 (MHz)
long signalBandwidth = 125E3;

// 手动输入调频控制相关变量
bool isFreqEditing = false;
int  digitCursor = 0; // 0: 100M, 1: 10M, 2: 1M, 3: 100k, 4: 10k, 5: 1k
int  freqDigits[6] = {4, 3, 8, 1, 5, 0}; // 对应 438.150 MHz 的各个数位

struct LoRaCombo { uint8_t sf; uint8_t cr; };
const LoRaCombo COMBO_LIST[] = {
  {12, 5}, {7, 5}, {8, 5}, {9, 5}, {10, 5}, {11, 5}, {7, 8}, {12, 8}
};
const int COMBO_COUNT = sizeof(COMBO_LIST) / sizeof(COMBO_LIST[0]);
int comboIdx = 0;

#define BOX_X 8
#define BOX_Y 12
#define BOX_W 112
#define BOX_H 32
#define RSSI_HIST_LEN 112

float rssiHistory[RSSI_HIST_LEN];
int rssiWrIdx = 0;
unsigned long lastSampleMs = 0;

bool isLocked = false;
String decodedPayload = "";
int lastPacketRssi = 0;
float lastPacketSnr = 0.0;

// 电池滤波平滑变量
float smoothedVbat = 0.0;
int displayedBatPct = -1;
unsigned long lastBatSampleMs = 0;
unsigned long lastBatPctMs = 0;

// 按键与菜单
enum BtnEvent { NONE, SINGLE_CLICK, DOUBLE_CLICK, LONG_PRESS };
unsigned long btnPressTime = 0;
unsigned long lastReleaseTime = 0;
bool lastBtnState = HIGH;
bool isWaitingForClick = false;

bool inMenu = false;
bool inCalibUI = false;
int menuSelection = 0;
const int MENU_ITEMS = 4;

float targetCalibVoltage = 3.80f;

// 函数声明
void applyLoRaConfig();
void applySpectrumBandwidth();
void runSpectrumScan();
void sampleRSSI();
void drawMainDisplay();
void drawSpectrumDisplay();
void drawMenuDisplay();
void drawCalibDisplay();
BtnEvent checkButton();
void batteryTick(bool force);
float readBatteryVoltage();
uint32_t readRawPinMillivolts();
int getBatteryPercent(float vbat);
void powerOff();
void updateSpanFreqs();
void saveCalibFactor(float factor);
void loadCalibFactor();
void freqDigitsToFloat();
void floatToFreqDigits();

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- Heltec WiFi LoRa 32 V2 (Manual Freq Tuning) ---");

  pinMode(PRG_BUTTON_PIN, INPUT_PULLUP);

  pinMode(VEXT_CTRL_PIN, OUTPUT);
  digitalWrite(VEXT_CTRL_PIN, LOW);
  delay(50);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[ERR] SSD1306 OLED init failed!");
    for(;;);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(20, 25);
  display.println(F("Initializing..."));
  display.display();

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  pinMode(VBAT_ADC_PIN, INPUT);

  loadCalibFactor();
  batteryTick(true);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  updateSpanFreqs();

  if (!LoRa.begin(currentFreq * 1E6)) {
    Serial.println("[ERR] LoRa Chip Init Failed!");
    display.clearDisplay();
    display.setCursor(20, 25);
    display.println(F("LoRa Init Failed!"));
    display.display();
    while (1);
  }

  applySpectrumBandwidth();
  LoRa.receive();
  for (int i = 0; i < RSSI_HIST_LEN; i++) rssiHistory[i] = -120.0;
  memset(waterfallBuf, 0, sizeof(waterfallBuf));

  floatToFreqDigits(); // 将初始频率转换为数位数组
  lastActivityMs = millis();
}

void loadCalibFactor() {
  prefs.begin("bat_cal", true);
  vbatCalFactor = prefs.getFloat("factor", 4.90f);
  prefs.end();
}

void saveCalibFactor(float factor) {
  prefs.begin("bat_cal", false);
  prefs.putFloat("factor", factor);
  prefs.end();
  vbatCalFactor = factor;
}

void updateSpanFreqs() {
  float halfSpan = spanOptions[spanIdx].spanMHz / 2.0;
  specStartFreq = specCenterFreq - halfSpan;
  specEndFreq   = specCenterFreq + halfSpan;
}

void applySpectrumBandwidth() {
  float k = spanOptions[spanIdx].stepkHz;
  long bw = (k >= 250) ? 250E3 : (k >= 125) ? 125E3 : 41.7E3;
  LoRa.setSignalBandwidth(bw);
  LoRa.receive();
  specInit = false;
}

// 辅助：将 currentFreq 转为数位数组
void floatToFreqDigits() {
  long khz = round(currentFreq * 1000.0f);
  freqDigits[0] = (khz / 100000) % 10;
  freqDigits[1] = (khz / 10000) % 10;
  freqDigits[2] = (khz / 1000) % 10;
  freqDigits[3] = (khz / 100) % 10;
  freqDigits[4] = (khz / 10) % 10;
  freqDigits[5] = khz % 10;
}

// 辅助：将数位数组转回 currentFreq 并限制合法范围
void freqDigitsToFloat() {
  long khz = freqDigits[0] * 100000L +
             freqDigits[1] * 10000L +
             freqDigits[2] * 1000L +
             freqDigits[3] * 100L +
             freqDigits[4] * 10L +
             freqDigits[5];
  currentFreq = khz / 1000.0f;
  if (currentFreq < 410.0f) currentFreq = 410.0f;
  if (currentFreq > 525.0f) currentFreq = 525.0f;
  floatToFreqDigits(); // 确保数值规整
}

void loop() {
  batteryTick(false);

  if (millis() - lastActivityMs > AUTO_POWER_OFF_MS) {
    powerOff();
  }

  BtnEvent evt = checkButton();
  if (evt != NONE) {
    lastActivityMs = millis();
  }

  // 双击：未在编辑频率时，进入/退出主菜单
  if (evt == DOUBLE_CLICK && !isFreqEditing) {
    if (inCalibUI) {
      inCalibUI = false;
      inMenu = true;
    } else {
      inMenu = !inMenu;
      if (inMenu) menuSelection = (currentMode == MODE_SPECTRUM) ? 0 : 1;
    }
  }

  // 1. 校准界面
  if (inCalibUI) {
    if (evt == SINGLE_CLICK) {
      targetCalibVoltage += 0.05f;
      if (targetCalibVoltage > 4.25f) targetCalibVoltage = 3.30f;
    } else if (evt == LONG_PRESS) {
      uint32_t pinmV = readRawPinMillivolts();
      if (pinmV > 0) {
        float newFactor = (targetCalibVoltage * 1000.0f) / (float)pinmV;
        saveCalibFactor(newFactor);
        smoothedVbat = 0.0f;
        batteryTick(true);
      }
      inCalibUI = false;
      inMenu = false;
    }
    drawCalibDisplay();
  }
  // 2. 主菜单界面
  else if (inMenu) {
    if (evt == SINGLE_CLICK) {
      menuSelection = (menuSelection + 1) % MENU_ITEMS;
    } else if (evt == LONG_PRESS) {
      if (menuSelection == 0) {
        currentMode = MODE_SPECTRUM;
        applySpectrumBandwidth();
        inMenu = false;
      } else if (menuSelection == 1) {
        currentMode = MODE_LORA_ANALYZER;
        applyLoRaConfig();
        inMenu = false;
      } else if (menuSelection == 2) {
        inCalibUI = true;
        targetCalibVoltage = readBatteryVoltage();
        if (targetCalibVoltage < 3.3f) targetCalibVoltage = 3.80f;
      } else if (menuSelection == 3) {
        powerOff();
      }
    }
    drawMenuDisplay();
  }
  // 3. 频谱扫描模式
  else if (currentMode == MODE_SPECTRUM) {
    if (evt == SINGLE_CLICK) {
      specCenterFreq += 1.0;
      if (specCenterFreq > 439.0) specCenterFreq = 431.0;
      updateSpanFreqs();
      specInit = false;
    } else if (evt == LONG_PRESS) {
      spanIdx = (spanIdx + 1) % SPAN_COUNT;
      updateSpanFreqs();
      applySpectrumBandwidth();
    }

    runSpectrumScan();
    drawSpectrumDisplay();
  }
  // 4. LoRa 接收解码模式 (包含手动调频控制)
  else {
    if (isLocked) {
      // 解码锁定状态下按键解锁
      if (evt == LONG_PRESS || evt == SINGLE_CLICK) {
        isLocked = false;
        decodedPayload = "";
      }
    } 
    // 手动调频编辑模式中
    else if (isFreqEditing) {
      if (evt == SINGLE_CLICK) {
        // 单击：移动输入光标到下一个数位
        digitCursor = (digitCursor + 1) % 6;
      } else if (evt == DOUBLE_CLICK) {
        // 双击：当前数位 +1
        freqDigits[digitCursor] = (freqDigits[digitCursor] + 1) % 10;
        freqDigitsToFloat(); // 更新计算
      } else if (evt == LONG_PRESS) {
        // 长按：保存设置，退出编辑模式并应用新频率
        freqDigitsToFloat();
        applyLoRaConfig();
        isFreqEditing = false;
      }
    } 
    // 正常 LoRa 接收监听状态
    else {
      if (evt == SINGLE_CLICK) {
        // 单击：切换 SF/CR 组合
        comboIdx = (comboIdx + 1) % COMBO_COUNT;
        applyLoRaConfig();
      } else if (evt == LONG_PRESS) {
        // 长按：进入手动频率调整模式
        isFreqEditing = true;
        digitCursor = 0;
        floatToFreqDigits();
      }
    }

    if (millis() - lastSampleMs >= 30) {
      lastSampleMs = millis();
      sampleRSSI();
    }

    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String incoming = "";
      while (LoRa.available()) incoming += (char)LoRa.read();

      isLocked = true;
      int aprsMsgIdx = incoming.indexOf("::");
      decodedPayload = (aprsMsgIdx != -1) ? incoming.substring(aprsMsgIdx + 2) : incoming;

      lastPacketRssi = LoRa.packetRssi();
      lastPacketSnr = LoRa.packetSnr();
    }

    drawMainDisplay();
  }
}

uint32_t readRawPinMillivolts() {
  digitalWrite(VEXT_CTRL_PIN, LOW);
  delayMicroseconds(3000);
  analogRead(VBAT_ADC_PIN);

  uint16_t s[32];
  for (int i = 0; i < 32; i++) {
    s[i] = analogRead(VBAT_ADC_PIN);
    delayMicroseconds(100);
  }
  for (int i = 1; i < 32; i++) {
    uint16_t k = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
    s[j + 1] = k;
  }
  uint32_t sum = 0;
  for (int i = 6; i < 26; i++) sum += s[i];
  return esp_adc_cal_raw_to_voltage(sum / 20, &adc_chars);
}

void batteryTick(bool force) {
  unsigned long now = millis();
  if (!force && now - lastBatSampleMs < 500) return;
  lastBatSampleMs = now;

  uint32_t pinmV = readRawPinMillivolts();
  float instant = ((float)pinmV * vbatCalFactor) / 1000.0f;

  if (smoothedVbat <= 0.1f) {
    smoothedVbat = instant;
  } else if (fabsf(instant - smoothedVbat) < 0.30f) {
    smoothedVbat = smoothedVbat * 0.95f + instant * 0.05f;
  } else {
    smoothedVbat = smoothedVbat * 0.99f + instant * 0.01f;
  }
}

float readBatteryVoltage() {
  if (smoothedVbat <= 0.1f) batteryTick(true);
  return smoothedVbat;
}

int getBatteryPercent(float vbat) {
  int calcPct;
  if (vbat >= 4.18f)      calcPct = 100;
  else if (vbat >= 3.82f) calcPct = 65 + (int)((vbat - 3.82f) / (4.18f - 3.82f) * 35.0f);
  else if (vbat >= 3.60f) calcPct = 20 + (int)((vbat - 3.60f) / (3.82f - 3.60f) * 45.0f);
  else if (vbat >= 3.30f) calcPct = (int)((vbat - 3.30f) / (3.60f - 3.30f) * 20.0f);
  else                    calcPct = 0;
  calcPct = constrain(calcPct, 0, 100);

  unsigned long now = millis();
  if (displayedBatPct == -1) {
    displayedBatPct = calcPct;
    lastBatPctMs = now;
  } else if (now - lastBatPctMs >= 4000 && calcPct != displayedBatPct) {
    displayedBatPct += (calcPct > displayedBatPct) ? 1 : -1;
    lastBatPctMs = now;
  }
  return displayedBatPct;
}

void drawCalibDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(10, 0);
  display.print("= BAT CALIBRATE =");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  float nowV = readBatteryVoltage();
  display.setCursor(0, 15);
  display.printf("Current: %.3fV", nowV);

  display.setCursor(0, 27);
  display.printf("Factor : %.4f", vbatCalFactor);

  display.fillRect(0, 39, 128, 13, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(2, 42);
  display.printf("Meter : > %.2fV <", targetCalibVoltage);

  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.setCursor(0, 55);
  display.print("Click:Adj Hold:Save");

  display.display();
}

void drawMenuDisplay() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(20, 0);
  display.println("= SELECT MODE =");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  const char* items[] = {"1. Spectrum Scan", "2. LoRa Receiver", "3. Calib Battery", "4. Power Off"};

  for (int i = 0; i < MENU_ITEMS; i++) {
    int yPos = 13 + i * 12;
    if (menuSelection == i) {
      display.fillRect(5, yPos, 118, 11, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    display.setCursor(8, yPos + 2);
    display.println(items[i]);
  }

  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.display();
}

void runSpectrumScan() {
  float step = (specEndFreq - specStartFreq) / SPEC_CHANNELS;
  float raw[SPEC_CHANNELS];

  for (int i = 0; i < SPEC_CHANNELS; i++) {
    float freq = specStartFreq + (i + 0.5f) * step;
    LoRa.setFrequency(freq * 1E6);
    LoRa.receive();
    delayMicroseconds(4500); 
    raw[i] = LoRa.rssi();
  }

  for (int i = 0; i < SPEC_CHANNELS; i++) {
    specSmooth[i] = specInit ? (specSmooth[i] * 0.5f + raw[i] * 0.5f) : raw[i];
    specRssi[i] = specSmooth[i];
  }

  float tmp[SPEC_CHANNELS];
  memcpy(tmp, specRssi, sizeof(tmp));
  for (int i = 1; i < SPEC_CHANNELS; i++) {
    float k = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = k;
  }
  
  float currentFrameNoise = tmp[SPEC_CHANNELS / 4]; 
  minFoundRssi = tmp[0];

  if (!specInit) {
    smoothedNoiseFloor = currentFrameNoise;
    displayedNoiseFloor = smoothedNoiseFloor;
    specInit = true;
  } else {
    if (currentFrameNoise < smoothedNoiseFloor) {
      smoothedNoiseFloor = smoothedNoiseFloor * 0.7f + currentFrameNoise * 0.3f;
    } else {
      smoothedNoiseFloor = smoothedNoiseFloor * 0.98f + currentFrameNoise * 0.02f;
    }
  }

  if (millis() - lastNoiseUpdateMs > NOISE_HOLD_TIME_MS) {
    displayedNoiseFloor = smoothedNoiseFloor;
    lastNoiseUpdateMs = millis();
  }

  maxFoundRssi = -160.0;
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    if (specRssi[i] > maxFoundRssi) {
      maxFoundRssi = specRssi[i];
      rawPeakFreq = specStartFreq + (i + 0.5f) * step;
    }
  }
  peakValid = (maxFoundRssi - smoothedNoiseFloor) >= PEAK_MIN_SNR;

  if (peakValid) {
    if (!displayedPeakValid || maxFoundRssi > displayedPeakRssi + 3.0f ||
        millis() - lastPeakUpdateMs > 1500) {
      displayedPeakFreq = rawPeakFreq;
      displayedPeakRssi = maxFoundRssi;
      displayedPeakValid = true;
      lastPeakUpdateMs = millis();
    }
  } else if (millis() - lastPeakUpdateMs > 1500) {
    displayedPeakValid = false;
  }

  int wfLines = WATERFALL_H - 2;
  for (int y = wfLines - 1; y > 0; y--) {
    for (int x = 0; x < SPEC_CHANNELS; x++) {
      waterfallBuf[y][x] = waterfallBuf[y - 1][x];
    }
  }

  const float WF_THRESH_LOW_ADJ  = 4.0f;
  const float WF_THRESH_HIGH_ADJ = 10.0f;

  for (int x = 0; x < SPEC_CHANNELS; x++) {
    float delta = specRssi[x] - smoothedNoiseFloor;
    if (delta >= WF_THRESH_HIGH_ADJ) {
      waterfallBuf[0][x] = 2;
    } else if (delta >= WF_THRESH_LOW_ADJ) {
      waterfallBuf[0][x] = 1;
    } else {
      waterfallBuf[0][x] = 0;
    }
  }
}

void drawSpectrumDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  char topBuf[24];
  if (displayedPeakValid)
    snprintf(topBuf, sizeof(topBuf), "P:%.2f %.0fdBm", displayedPeakFreq, displayedPeakRssi);
  else
    snprintf(topBuf, sizeof(topBuf), "P:--.--");
  display.setCursor(0, 0);
  display.print(topBuf);

  float vbat = readBatteryVoltage();
  int pct = getBatteryPercent(vbat);
  char batStr[8];
  snprintf(batStr, sizeof(batStr), "%d%%", pct);
  int batX = SCREEN_WIDTH - (strlen(batStr) * 6);
  display.setCursor(batX, 0);
  display.print(batStr);

  display.drawRect(SPEC_BOX_X, SPEC_BOX_Y, SPEC_BOX_W, SPEC_BOX_H, SSD1306_WHITE);
  int innerX = SPEC_BOX_X + 1;
  int innerY = SPEC_BOX_Y + 1;
  int innerH = SPEC_BOX_H - 2;
  int colWidth = (SPEC_BOX_W - 2) / SPEC_CHANNELS;

  float lowRef = displayedNoiseFloor - 6.0f;
  if (lowRef > -45.0f) lowRef = -45.0f;

  for (int i = 0; i < SPEC_CHANNELS; i++) {
    int lineH = map((int)constrain(specRssi[i], lowRef, -30.0f), (int)lowRef, -30, 1, innerH);
    int xPos = innerX + i * colWidth;
    if (lineH > 0) {
      display.fillRect(xPos, innerY + innerH - lineH, colWidth - 1, lineH, SSD1306_WHITE);
    }
  }

  char noiseBuf[12];
  snprintf(noiseBuf, sizeof(noiseBuf), "NF:%.0f", displayedNoiseFloor);
  int noiseTextW = strlen(noiseBuf) * 6;
  int noiseBoxX = (SPEC_BOX_X + SPEC_BOX_W) - noiseTextW - 3;

  display.fillRect(noiseBoxX - 1, SPEC_BOX_Y + 2, noiseTextW + 2, 9, SSD1306_BLACK);
  display.setCursor(noiseBoxX, SPEC_BOX_Y + 3);
  display.print(noiseBuf);

  display.drawRect(WATERFALL_X, WATERFALL_Y, WATERFALL_W, WATERFALL_H, SSD1306_WHITE);
  int wfInnerX = WATERFALL_X + 1;
  int wfInnerY = WATERFALL_Y + 1;
  int wfLines = WATERFALL_H - 2;

  for (int y = 0; y < wfLines; y++) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      int xPos = wfInnerX + i * colWidth;
      uint8_t val = waterfallBuf[y][i];
      if (val == 2) {
        display.fillRect(xPos, wfInnerY + y, colWidth - 1, 1, SSD1306_WHITE);
      } else if (val == 1) {
        if ((i + y) % 2 == 0) {
          display.drawPixel(xPos, wfInnerY + y, SSD1306_WHITE);
        }
      }
    }
  }

  display.setCursor(0, 56);
  display.printf("C:%.1fM", specCenterFreq);

  display.setCursor(56, 56);
  display.printf("S:%.0fM", spanOptions[spanIdx].spanMHz);

  display.setCursor(94, 56);
  display.printf("K:%.0fk", spanOptions[spanIdx].stepkHz);

  display.display();
}

void applyLoRaConfig() {
  LoRa.setFrequency(currentFreq * 1E6);
  LoRa.setSignalBandwidth(signalBandwidth);
  LoRa.setSpreadingFactor(COMBO_LIST[comboIdx].sf);
  LoRa.setCodingRate4(COMBO_LIST[comboIdx].cr);
  LoRa.receive();
}

void sampleRSSI() {
  float rawRssi = LoRa.rssi();
  rssiHistory[rssiWrIdx] = rawRssi;
  rssiWrIdx = (rssiWrIdx + 1) % RSSI_HIST_LEN;
}

void drawMainDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // 1. 顶部左侧：显示频率及手动调频 UI
  if (isFreqEditing) {
    // 调频编辑模式下：绘制 6 个数位并对当前光标位置显示下划线
    int xStart = 0;
    int digitWidths[] = {0, 6, 12, 24, 30, 36}; // 对应 438.150 各数字偏移位置 (含小数点)
    
    char freqStr[12];
    snprintf(freqStr, sizeof(freqStr), "%d%d%d.%d%d%dM", 
             freqDigits[0], freqDigits[1], freqDigits[2], 
             freqDigits[3], freqDigits[4], freqDigits[5]);
    display.setCursor(0, 0);
    display.print(freqStr);

    // 绘制指示当前选中数位的下划线
    int cursorX = digitWidths[digitCursor];
    display.drawFastHLine(cursorX, 9, 5, SSD1306_WHITE);
  } else {
    // 正常显示当前频率
    display.setCursor(0, 0);
    display.printf("%.3fM", currentFreq);
  }

  // 2. 顶部右侧：电池电量
  float vbat = readBatteryVoltage();
  int pct = getBatteryPercent(vbat);
  char batStr[8];
  snprintf(batStr, sizeof(batStr), "%d%%", pct);
  int batX = SCREEN_WIDTH - (strlen(batStr) * 6);
  display.setCursor(batX, 0);
  display.print(batStr);

  // 3. 中间数据框/波形图
  display.drawRect(BOX_X - 1, BOX_Y - 1, BOX_W + 2, BOX_H + 2, SSD1306_WHITE);

  if (isLocked) {
    display.setTextWrap(false);
    String line1 = "", line2 = "";
    if (decodedPayload.length() <= 17) {
      line1 = decodedPayload;
    } else {
      line1 = decodedPayload.substring(0, 17);
      line2 = (decodedPayload.length() > 32) ? (decodedPayload.substring(17, 31) + "...") : decodedPayload.substring(17);
    }
    display.setCursor(BOX_X + 2, BOX_Y + 2);
    display.print(line1.length() > 0 ? line1 : "<EMPTY>");
    if (line2.length() > 0) {
      display.setCursor(BOX_X + 2, BOX_Y + 11);
      display.print(line2);
    }
    display.setCursor(BOX_X + 2, BOX_Y + 22);
    display.printf("R:%ddBm S:%.1fdB", lastPacketRssi, lastPacketSnr);
  } else {
    display.setTextWrap(false);
    float minRssi = 0.0;
    for (int i = 0; i < RSSI_HIST_LEN; i++) {
      if (rssiHistory[i] < minRssi) minRssi = rssiHistory[i];
    }
    if (minRssi > -60.0) minRssi = -120.0;

    for (int col = 0; col < BOX_W; col++) {
      int idx = (rssiWrIdx + col) % RSSI_HIST_LEN;
      float val = rssiHistory[idx];
      int lineH = map((int)constrain(val, minRssi, -30.0), (int)minRssi, -30, 0, BOX_H - 1);
      if (lineH > 0) {
        display.drawFastVLine(BOX_X + col, BOX_Y + BOX_H - lineH, lineH, SSD1306_WHITE);
      }
    }
  }

  // 4. 底部栏提示
  int bottomY = 52;
  display.setCursor(0, bottomY);

  if (isFreqEditing) {
    // 调频状态下显示提示
    display.print("1Click:Pos 2Click:+1");
  } else {
    display.printf("BW:%.0fK", signalBandwidth / 1000.0);

    char sfCrBuf[16];
    snprintf(sfCrBuf, sizeof(sfCrBuf), "SF%d/CR%d", COMBO_LIST[comboIdx].sf, COMBO_LIST[comboIdx].cr);
    int xPos = SCREEN_WIDTH - (strlen(sfCrBuf) * 6);
    display.setCursor(xPos, bottomY);
    display.print(sfCrBuf);
  }

  display.display();
}

void powerOff() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(25, 28);
  display.println(F("POWERING OFF..."));
  display.display();
  delay(1000);

  display.clearDisplay();
  display.display();

  digitalWrite(VEXT_CTRL_PIN, HIGH);
  esp_deep_sleep_start();
}

BtnEvent checkButton() {
  bool currentState = digitalRead(PRG_BUTTON_PIN);
  unsigned long now = millis();
  BtnEvent event = NONE;

  if (lastBtnState == HIGH && currentState == LOW) {
    btnPressTime = now;
  } else if (lastBtnState == LOW && currentState == HIGH) {
    unsigned long pressDuration = now - btnPressTime;

    if (pressDuration >= 1200) {
      event = LONG_PRESS;
      isWaitingForClick = false;
    } else if (pressDuration > 50) {
      if (isWaitingForClick && (now - lastReleaseTime < 350)) {
        event = DOUBLE_CLICK;
        isWaitingForClick = false;
      } else {
        isWaitingForClick = true;
      }
      lastReleaseTime = now;
    }
  }

  if (isWaitingForClick && (now - lastReleaseTime >= 350)) {
    event = SINGLE_CLICK;
    isWaitingForClick = false;
  }

  lastBtnState = currentState;
  return event;
}