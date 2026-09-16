#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_sleep.h>
#include <esp_adc_cal.h>
#include <Preferences.h>

// ================= 硬件引脚配置 (Heltec V2) =================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

#define VEXT_CTRL_PIN 21   // 控制屏幕及分压电路供电 (LOW 为激活)
#define OLED_SDA      4
#define OLED_SCL      15
#define OLED_RST      16 
#define VBAT_ADC_PIN  37   // Heltec V2 板载电池采样引脚 (ADC1_CH1)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
Preferences prefs;
esp_adc_cal_characteristics_t adc_chars;

// RadioLib SX1276 引脚配置: NSS=18, DIO0=26, RESET=14, DIO1=35
SX1276 radio = new Module(18, 26, 14, 35);

#define PRG_BUTTON_PIN 0 

// ================= NVS 闪存校准参数 =================
float vbatCalFactor = 4.90f;

// ================= 定时与休眠配置 =================
const unsigned long AUTO_POWER_OFF_MS = 10 * 60 * 1000UL; // 10分钟无操作自动关机
unsigned long lastActivityMs = 0;

// ================= 模式与状态 =================
enum SystemMode { MODE_SPECTRUM, MODE_FSK_ANALYZER };
SystemMode currentMode = MODE_SPECTRUM; 

// 频谱扫描配置
struct SpectrumSpanOption {
  float spanMHz;
  float stepkHz;
};
SpectrumSpanOption spanOptions[] = {
  {10.0, 250.0},
  {5.0,  125.0},
  {2.0,  50.0}
};
const int SPAN_COUNT = sizeof(spanOptions) / sizeof(spanOptions[0]);
int spanIdx = 0;

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

// ---------------- FSK 分析模式变量 ----------------
const float FSK_FREQ_LIST[] = { 434.000, 436.000, 437.000, 438.000, 440.000 };
const int FSK_FREQ_COUNT = sizeof(FSK_FREQ_LIST) / sizeof(FSK_FREQ_LIST[0]);
int fskFreqIdx = 0;
float currentFskFreq = FSK_FREQ_LIST[0];
float fskRxBandwidth = 62.5; // 固定带宽 62.5kHz

struct FskCombo { float bitrate; float freqDev; const char* label; };
const FskCombo FSK_COMBO_LIST[] = {
  { 4.8,  25.0, "4.8k/25k" },
  { 1.2,   5.0, "1.2k/5k"  },
  { 9.6,  19.2, "9.6k/19.2k"},
  {19.2,  25.0, "19.2k/25k"}
};
const int FSK_COMBO_COUNT = sizeof(FSK_COMBO_LIST) / sizeof(FSK_COMBO_LIST[0]);
int fskComboIdx = 0;

// 通用波形与历史接收数据框
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

// 电池滤波平滑变量
float smoothedVbat = 0.0;
int displayedBatPct = -1;

// 按键与菜单
enum BtnEvent { NONE, SINGLE_CLICK, DOUBLE_CLICK, LONG_PRESS };
unsigned long btnPressTime = 0;
unsigned long lastReleaseTime = 0;
bool lastBtnState = HIGH;
bool isWaitingForClick = false;

bool inMenu = false;
bool inCalibUI = false;
int menuSelection = 0; 
const int MENU_ITEMS = 4; // 1.Spectrum 2.FSK Receiver 3.Calib Bat 4.Power Off

float targetCalibVoltage = 3.80f;

// 函数声明
void applyFskConfig();
void runSpectrumScan();
void sampleRSSI();
void drawMainDisplay();
void drawSpectrumDisplay();
void drawMenuDisplay();
void drawCalibDisplay();
BtnEvent checkButton();
float readBatteryVoltage();
uint32_t readRawPinMillivolts();
int getBatteryPercent(float vbat);
void powerOff();
void updateSpanFreqs();
void saveCalibFactor(float factor);
void loadCalibFactor();

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- Heltec WiFi LoRa 32 V2 (FSK Receiver Only) ---");

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
  updateSpanFreqs();

  // 初始化 RadioLib 模块为 FSK 模式
  int state = radio.beginFSK(currentFskFreq, 
                             FSK_COMBO_LIST[fskComboIdx].bitrate, 
                             FSK_COMBO_LIST[fskComboIdx].freqDev, 
                             fskRxBandwidth);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[ERR] Radio Init Failed, code: %d\n", state);
    display.clearDisplay();
    display.setCursor(10, 25);
    display.println(F("Radio Init Failed!"));
    display.display();
    while (1);
  }
  
  radio.startReceive();
  for (int i = 0; i < RSSI_HIST_LEN; i++) rssiHistory[i] = -120.0;
  memset(waterfallBuf, 0, sizeof(waterfallBuf));

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

void applyFskConfig() {
  radio.setFrequency(currentFskFreq);
  radio.setBitRate(FSK_COMBO_LIST[fskComboIdx].bitrate);
  radio.setFrequencyDeviation(FSK_COMBO_LIST[fskComboIdx].freqDev);
  radio.setRxBandwidth(fskRxBandwidth);
  radio.startReceive();
}

void loop() {
  if (millis() - lastActivityMs > AUTO_POWER_OFF_MS) {
    powerOff();
  }

  BtnEvent evt = checkButton();
  if (evt != NONE) {
    lastActivityMs = millis();
  }

  if (evt == DOUBLE_CLICK) {
    if (inCalibUI) {
      inCalibUI = false;
      inMenu = true;
    } else {
      inMenu = !inMenu;
      if (inMenu) {
        if (currentMode == MODE_SPECTRUM) menuSelection = 0;
        else if (currentMode == MODE_FSK_ANALYZER) menuSelection = 1;
      }
    }
  }

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
      }
      inCalibUI = false;
      inMenu = false;
    }
    drawCalibDisplay();
  }
  else if (inMenu) {
    if (evt == SINGLE_CLICK) {
      menuSelection = (menuSelection + 1) % MENU_ITEMS;
    } else if (evt == LONG_PRESS) {
      if (menuSelection == 0) {
        currentMode = MODE_SPECTRUM;
        inMenu = false;
      } else if (menuSelection == 1) {
        currentMode = MODE_FSK_ANALYZER;
        applyFskConfig();
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
  else if (currentMode == MODE_SPECTRUM) {
    if (evt == SINGLE_CLICK) {
      specCenterFreq += 1.0;
      if (specCenterFreq > 439.0) specCenterFreq = 431.0;
      updateSpanFreqs();
    } else if (evt == LONG_PRESS) {
      spanIdx = (spanIdx + 1) % SPAN_COUNT;
      updateSpanFreqs();
    }

    runSpectrumScan();
    drawSpectrumDisplay();
  } 
  else if (currentMode == MODE_FSK_ANALYZER) {
    if (isLocked) {
      if (evt == LONG_PRESS || evt == SINGLE_CLICK) {
        isLocked = false;
        decodedPayload = "";
        radio.startReceive();
      }
    } else {
      if (evt == SINGLE_CLICK) {
        fskFreqIdx = (fskFreqIdx + 1) % FSK_FREQ_COUNT;
        currentFskFreq = FSK_FREQ_LIST[fskFreqIdx];
        applyFskConfig();
      } else if (evt == LONG_PRESS) {
        fskComboIdx = (fskComboIdx + 1) % FSK_COMBO_COUNT;
        applyFskConfig();
      }
    }

    if (millis() - lastSampleMs >= 30) {
      lastSampleMs = millis();
      sampleRSSI();
    }

    // FSK 报文解包
    String incoming = "";
    int state = radio.readData(incoming);
    if (state == RADIOLIB_ERR_NONE && incoming.length() > 0) {
      isLocked = true;
      decodedPayload = incoming;
      lastPacketRssi = radio.getRSSI();
    }

    drawMainDisplay();
  }
}

void sampleRSSI() {
  float rawRssi = radio.getRSSI(); 
  rssiHistory[rssiWrIdx] = rawRssi;
  rssiWrIdx = (rssiWrIdx + 1) % RSSI_HIST_LEN;
}

void drawMainDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("%.3fM FSK", currentFskFreq);
  
  float vbat = readBatteryVoltage();
  int pct = getBatteryPercent(vbat);
  char batStr[8];
  snprintf(batStr, sizeof(batStr), "%d%%", pct);
  int batX = SCREEN_WIDTH - (strlen(batStr) * 6);
  display.setCursor(batX, 0);
  display.print(batStr);

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
    display.printf("RSSI: %d dBm", lastPacketRssi);
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

  int bottomY = 52;
  display.setCursor(0, bottomY);
  display.print("BW:62.5K");
  const char* fskParamStr = FSK_COMBO_LIST[fskComboIdx].label;
  int xPos = SCREEN_WIDTH - (strlen(fskParamStr) * 6);
  display.setCursor(xPos, bottomY);
  display.print(fskParamStr);

  display.display();
}

void drawMenuDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(18, 0);
  display.println("= SELECT MODE =");
  display.drawFastHLine(0, 9, 128, SSD1306_WHITE);

  const char* items[] = {
    "1. Spectrum Scan", 
    "2. FSK Receiver", 
    "3. Calib Battery", 
    "4. Power Off"
  };

  for (int i = 0; i < MENU_ITEMS; i++) {
    int yPos = 13 + i * 11;
    if (menuSelection == i) {
      display.fillRect(4, yPos, 120, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    display.setCursor(6, yPos + 1);
    display.println(items[i]);
  }

  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.display();
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

void runSpectrumScan() {
  float step = (specEndFreq - specStartFreq) / SPEC_CHANNELS;
  maxFoundRssi = -160.0;
  minFoundRssi = 0.0;
  float rssiSum = 0.0;

  for (int i = 0; i < SPEC_CHANNELS; i++) {
    float freq = specStartFreq + i * step;
    radio.setFrequency(freq);
    delayMicroseconds(1800); 
    
    float val = radio.getRSSI();
    specRssi[i] = val;
    rssiSum += val;

    if (val > maxFoundRssi) {
      maxFoundRssi = val;
      rawPeakFreq = freq;
    }
    if (val < minFoundRssi) {
      minFoundRssi = val;
    }
  }

  float instantAvgNoise = rssiSum / SPEC_CHANNELS;
  smoothedNoiseFloor = (smoothedNoiseFloor * 0.85) + (instantAvgNoise * 0.15);
  
  if (millis() - lastNoiseUpdateMs > NOISE_HOLD_TIME_MS) {
    displayedNoiseFloor = smoothedNoiseFloor;
    lastNoiseUpdateMs = millis();
  }

  if ((maxFoundRssi > displayedPeakRssi + 3.0) || (millis() - lastPeakUpdateMs > 1500)) {
    displayedPeakFreq = rawPeakFreq;
    displayedPeakRssi = maxFoundRssi;
    lastPeakUpdateMs = millis();
  }

  int wfLines = WATERFALL_H - 2;
  for (int y = wfLines - 1; y > 0; y--) {
    for (int x = 0; x < SPEC_CHANNELS; x++) {
      waterfallBuf[y][x] = waterfallBuf[y - 1][x];
    }
  }

  for (int x = 0; x < SPEC_CHANNELS; x++) {
    float delta = specRssi[x] - minFoundRssi;
    if (delta > 20.0)      waterfallBuf[0][x] = 2; 
    else if (delta > 8.0)  waterfallBuf[0][x] = 1; 
    else                   waterfallBuf[0][x] = 0; 
  }
}

void drawSpectrumDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  char topBuf[16];
  snprintf(topBuf, sizeof(topBuf), "P:%.2f", displayedPeakFreq);
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

  for (int i = 0; i < SPEC_CHANNELS; i++) {
    int lineH = map((int)constrain(specRssi[i], minFoundRssi, -30.0), (int)minFoundRssi, -30, 1, innerH);
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

uint32_t readRawPinMillivolts() {
  digitalWrite(VEXT_CTRL_PIN, LOW);
  delayMicroseconds(3000);

  analogRead(VBAT_ADC_PIN);

  uint32_t rawSum = 0;
  for (int i = 0; i < 32; i++) {
    rawSum += analogRead(VBAT_ADC_PIN);
    delayMicroseconds(100);
  }
  uint32_t rawAvg = rawSum / 32;

  return esp_adc_cal_raw_to_voltage(rawAvg, &adc_chars);
}

float readBatteryVoltage() {
  uint32_t pinmV = readRawPinMillivolts();
  float instantVbat = ((float)pinmV * vbatCalFactor) / 1000.0f;

  if (smoothedVbat <= 0.1f) {
    smoothedVbat = instantVbat;
  } else {
    smoothedVbat = (smoothedVbat * 0.90f) + (instantVbat * 0.10f);
  }

  return smoothedVbat;
}

int getBatteryPercent(float vbat) {
  int calcPct = 0;

  if (vbat >= 4.18f) {
    calcPct = 100;
  } else if (vbat >= 3.82f) {
    calcPct = 65 + (int)((vbat - 3.82f) / (4.18f - 3.82f) * 35.0f);
  } else if (vbat >= 3.60f) {
    calcPct = 20 + (int)((vbat - 3.60f) / (3.82f - 3.60f) * 45.0f);
  } else if (vbat >= 3.30f) {
    calcPct = 0 + (int)((vbat - 3.30f) / (3.60f - 3.30f) * 20.0f);
  } else {
    calcPct = 0;
  }

  calcPct = constrain(calcPct, 0, 100);

  if (displayedBatPct == -1 || abs(calcPct - displayedBatPct) >= 2) {
    displayedBatPct = calcPct;
  }
  return displayedBatPct;
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