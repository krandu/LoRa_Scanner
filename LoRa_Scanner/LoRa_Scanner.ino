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
bool  specInit = false;           // 参数变化后重新初始化平滑缓冲
bool  peakValid = false;
bool  displayedPeakValid = false;

const float WF_THRESH_LOW  = 6.0;   // 高出底噪 6dB 画灰点
const float WF_THRESH_HIGH = 15.0;  // 高出底噪 15dB 画实点
const float PEAK_MIN_SNR   = 8.0;   // 峰值有效门限

// 峰值防抖相关变量
float rawPeakRssi = -160.0;
float rawPeakFreq = 435.0;
float displayedPeakFreq = 435.0;
float displayedPeakRssi = -160.0;
unsigned long lastPeakUpdateMs = 0;

// 底噪平滑与保持变量
float smoothedNoiseFloor = -100.0;
float displayedNoiseFloor = -100.0;
unsigned long lastNoiseUpdateMs = 0;
const unsigned long NOISE_HOLD_TIME_MS = 1500;

float maxFoundRssi = -160.0;
float minFoundRssi = 0.0;

// LoRa 分析模式变量
const float FREQ_LIST[] = { 438.150, 438.125, 438.000, 438.500 };
const int FREQ_COUNT = sizeof(FREQ_LIST) / sizeof(FREQ_LIST[0]);
int freqIdx = 0;
float currentFreq = FREQ_LIST[0];
long signalBandwidth = 125E3;

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
unsigned long lastBtnChangeMs = 0;
const unsigned long DOUBLE_CLICK_GAP_MS = 400;  // 双击间隔窗口
const unsigned long BTN_DEBOUNCE_MS = 20;       // 消抖时间
QueueHandle_t btnQueue = NULL;                  // 按键事件队列（由独立任务写入）

bool inMenu = false;
bool inCalibUI = false;
int menuSelection = 0;
const int MENU_ITEMS = 4; // 1.Spectrum 2.LoRa 3.Calib Bat 4.Power Off

float targetCalibVoltage = 3.80f; // 万用表测量参考电压设定值

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
void buttonTask(void* param);
void batteryTick(bool force);
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
  Serial.println("\n--- Heltec WiFi LoRa 32 V2 (Auto Calib) ---");

  pinMode(PRG_BUTTON_PIN, INPUT_PULLUP);

  // 按键独立任务：每 5ms 轮询，不受扫描/刷屏耗时影响
  btnQueue = xQueueCreate(8, sizeof(BtnEvent));
  xTaskCreatePinnedToCore(buttonTask, "btn", 2048, NULL, 2, NULL, 0);

  // 开启 VEXT (低电平开启屏幕及分压测量电路供电)
  pinMode(VEXT_CTRL_PIN, OUTPUT);
  digitalWrite(VEXT_CTRL_PIN, LOW);
  delay(50);

  // 初始化 OLED 复位引脚
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

  // 配置 ESP32 ADC1 & 工厂 eFuse 校准
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  pinMode(VBAT_ADC_PIN, INPUT);

  // 读取 Flash 保存的校准参数
  loadCalibFactor();
  batteryTick(true);   // 初始化电池读数

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

  applySpectrumBandwidth();   // 频谱模式按步进选择接收带宽
  LoRa.receive();
  for (int i = 0; i < RSSI_HIST_LEN; i++) rssiHistory[i] = -120.0;
  memset(waterfallBuf, 0, sizeof(waterfallBuf));

  lastActivityMs = millis();
}

// 加载 Flash 存储的参数
void loadCalibFactor() {
  prefs.begin("bat_cal", true); // 只读模式
  vbatCalFactor = prefs.getFloat("factor", 4.90f);
  prefs.end();
  Serial.printf("[NVS] Loaded Calibration Factor: %.4f\n", vbatCalFactor);
}

// 保存参数到 Flash
void saveCalibFactor(float factor) {
  prefs.begin("bat_cal", false); // 读写模式
  prefs.putFloat("factor", factor);
  prefs.end();
  vbatCalFactor = factor;
  Serial.printf("[NVS] Saved New Calibration Factor: %.4f\n", vbatCalFactor);
}

void updateSpanFreqs() {
  float halfSpan = spanOptions[spanIdx].spanMHz / 2.0;
  specStartFreq = specCenterFreq - halfSpan;
  specEndFreq   = specCenterFreq + halfSpan;
}

// 根据步进自动选择扫描带宽，避免漏扫/串扰
void applySpectrumBandwidth() {
  float k = spanOptions[spanIdx].stepkHz;
  long bw = (k >= 250) ? 250E3 : (k >= 125) ? 125E3 : 41.7E3;
  LoRa.setSignalBandwidth(bw);
  LoRa.receive();
  specInit = false;
}

void loop() {
  batteryTick(false);   // 每 500ms 采样一次

  // 1. 自动关机检测
  if (millis() - lastActivityMs > AUTO_POWER_OFF_MS) {
    powerOff();
  }

  // 2. 检测按键事件
  BtnEvent evt = NONE;
  xQueueReceive(btnQueue, &evt, 0);   // 取出按键任务产生的事件
  if (evt != NONE) {
    lastActivityMs = millis();
  }

  // 双击随时进入/退出主菜单
  if (evt == DOUBLE_CLICK) {
    if (inCalibUI) {
      inCalibUI = false;
      inMenu = true;
    } else {
      inMenu = !inMenu;
      if (inMenu) menuSelection = (currentMode == MODE_SPECTRUM) ? 0 : 1;
    }
  }

  // 3. 校准界面交互
  if (inCalibUI) {
    if (evt == SINGLE_CLICK) {
      // 步进切换万用表测量值 (+0.05V，循环 3.30V ~ 4.25V)
      targetCalibVoltage += 0.05f;
      if (targetCalibVoltage > 4.25f) targetCalibVoltage = 3.30f;
    } else if (evt == LONG_PRESS) {
      // 长按保存：自动计算新系数并存盘
      uint32_t pinmV = readRawPinMillivolts();
      if (pinmV > 0) {
        float newFactor = (targetCalibVoltage * 1000.0f) / (float)pinmV;
        saveCalibFactor(newFactor);
        smoothedVbat = 0.0f; // 重置滤波缓冲
        batteryTick(true);
      }
      inCalibUI = false;
      inMenu = false;
    }
    drawCalibDisplay();
  }
  // 4. 菜单控制
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
        inCalibUI = true; // 进入电池校准模式
        targetCalibVoltage = readBatteryVoltage(); // 以当前测得电压为基准
        if (targetCalibVoltage < 3.3f) targetCalibVoltage = 3.80f;
      } else if (menuSelection == 3) {
        powerOff();
      }
    }
    drawMenuDisplay();
  }
  // 5. 频谱扫描模式
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
  // 6. LoRa 分析模式
  else {
    if (isLocked) {
      if (evt == LONG_PRESS || evt == SINGLE_CLICK) {
        isLocked = false;
        decodedPayload = "";
      }
    } else {
      if (evt == SINGLE_CLICK) {
        comboIdx = (comboIdx + 1) % COMBO_COUNT;
        applyLoRaConfig();
      } else if (evt == LONG_PRESS) {
        freqIdx = (freqIdx + 1) % FREQ_COUNT;
        currentFreq = FREQ_LIST[freqIdx];
        applyLoRaConfig();
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

// 读取 ADC 引脚毫伏值：32 次采样，排序后去掉最高/最低各 6 个取平均
uint32_t readRawPinMillivolts() {
  digitalWrite(VEXT_CTRL_PIN, LOW);
  delayMicroseconds(3000);
  analogRead(VBAT_ADC_PIN); // 丢弃首帧

  uint16_t s[32];
  for (int i = 0; i < 32; i++) {
    s[i] = analogRead(VBAT_ADC_PIN);
    delayMicroseconds(100);
  }
  for (int i = 1; i < 32; i++) {          // 插入排序
    uint16_t k = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
    s[j + 1] = k;
  }
  uint32_t sum = 0;
  for (int i = 6; i < 26; i++) sum += s[i];
  return esp_adc_cal_raw_to_voltage(sum / 20, &adc_chars);
}

// 定时采样 + 慢速滤波
void batteryTick(bool force) {
  unsigned long now = millis();
  if (!force && now - lastBatSampleMs < 500) return;
  lastBatSampleMs = now;

  uint32_t pinmV = readRawPinMillivolts();
  float instant = ((float)pinmV * vbatCalFactor) / 1000.0f;

  if (smoothedVbat <= 0.1f) {
    smoothedVbat = instant;
  } else if (fabsf(instant - smoothedVbat) < 0.30f) {      // 剔除突变
    smoothedVbat = smoothedVbat * 0.95f + instant * 0.05f;
  } else {
    smoothedVbat = smoothedVbat * 0.99f + instant * 0.01f; // 大偏差时极慢跟随
  }
}

// 只返回已滤波的电压，不再每帧采样
float readBatteryVoltage() {
  if (smoothedVbat <= 0.1f) batteryTick(true);
  return smoothedVbat;
}

// 电池电量百分比：每 4 秒最多变化 1%
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

// 绘制电池校准界面
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

  // 反显可调的目标测量值
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

// 频谱扫描逻辑
void runSpectrumScan() {
  float step = (specEndFreq - specStartFreq) / SPEC_CHANNELS;
  float raw[SPEC_CHANNELS];

  for (int i = 0; i < SPEC_CHANNELS; i++) {
    float freq = specStartFreq + (i + 0.5f) * step;   // 取通道中心
    LoRa.setFrequency(freq * 1E6);
    LoRa.receive();
    delayMicroseconds(2500);                          // 留足 RSSI 稳定时间
    raw[i] = LoRa.rssi();
  }

  // 1) 每通道时间平滑，抑制毛刺
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    specSmooth[i] = specInit ? (specSmooth[i] * 0.6f + raw[i] * 0.4f) : raw[i];
    specRssi[i] = specSmooth[i];
  }

  // 2) 中位数作为底噪（不受信号影响）
  float tmp[SPEC_CHANNELS];
  memcpy(tmp, specRssi, sizeof(tmp));
  for (int i = 1; i < SPEC_CHANNELS; i++) {
    float k = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > k) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = k;
  }
  float median = tmp[SPEC_CHANNELS / 2];
  minFoundRssi = tmp[0];
  smoothedNoiseFloor = specInit ? (smoothedNoiseFloor * 0.85f + median * 0.15f) : median;
  if (!specInit) {
    displayedNoiseFloor = smoothedNoiseFloor;
    specInit = true;
  }

  if (millis() - lastNoiseUpdateMs > NOISE_HOLD_TIME_MS) {
    displayedNoiseFloor = smoothedNoiseFloor;
    lastNoiseUpdateMs = millis();
  }

  // 3) 峰值：必须高出底噪 PEAK_MIN_SNR 才有效
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
    displayedPeakValid = false;   // 信号消失后 1.5s 清除
  }

  // 4) 瀑布图：以底噪为基准，而不是以最低点为基准
  int wfLines = WATERFALL_H - 2;
  for (int y = wfLines - 1; y > 0; y--)
    for (int x = 0; x < SPEC_CHANNELS; x++)
      waterfallBuf[y][x] = waterfallBuf[y - 1][x];

  for (int x = 0; x < SPEC_CHANNELS; x++) {
    float delta = specRssi[x] - smoothedNoiseFloor;
    if (delta > WF_THRESH_HIGH)     waterfallBuf[0][x] = 2;
    else if (delta > WF_THRESH_LOW) waterfallBuf[0][x] = 1;
    else                            waterfallBuf[0][x] = 0;
  }
}

void drawSpectrumDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // 顶部峰值：频率 + 强度
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

  // 柱状图下限以底噪为基准
  float lowRef = displayedNoiseFloor - 6.0f;
  if (lowRef > -45.0f) lowRef = -45.0f;   // 防止范围过小

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

  display.setCursor(0, 0);
  display.printf("%.3fM", currentFreq);

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

  int bottomY = 52;
  display.setCursor(0, bottomY);
  display.printf("BW:%.0fK", signalBandwidth / 1000.0);

  char sfCrBuf[16];
  snprintf(sfCrBuf, sizeof(sfCrBuf), "SF%d/CR%d", COMBO_LIST[comboIdx].sf, COMBO_LIST[comboIdx].cr);
  int xPos = SCREEN_WIDTH - (strlen(sfCrBuf) * 6);
  display.setCursor(xPos, bottomY);
  display.print(sfCrBuf);

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

// 按键轮询任务（5ms 周期）
void buttonTask(void* param) {
  for (;;) {
    BtnEvent e = checkButton();
    if (e != NONE) xQueueSend(btnQueue, &e, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// 只允许在 buttonTask 中调用
BtnEvent checkButton() {
  bool currentState = digitalRead(PRG_BUTTON_PIN);
  unsigned long now = millis();
  BtnEvent event = NONE;

  // 电平变化需稳定 BTN_DEBOUNCE_MS 以上才接受
  if (currentState != lastBtnState && (now - lastBtnChangeMs) < BTN_DEBOUNCE_MS) {
    currentState = lastBtnState;
  }

  if (lastBtnState == HIGH && currentState == LOW) {
    btnPressTime = now;
    lastBtnChangeMs = now;
  } else if (lastBtnState == LOW && currentState == HIGH) {
    lastBtnChangeMs = now;
    unsigned long pressDuration = now - btnPressTime;

    if (pressDuration >= 1200) {
      event = LONG_PRESS;
      isWaitingForClick = false;
    } else if (pressDuration > 30) {
      if (isWaitingForClick && (now - lastReleaseTime < DOUBLE_CLICK_GAP_MS)) {
        event = DOUBLE_CLICK;
        isWaitingForClick = false;
      } else {
        isWaitingForClick = true;
      }
      lastReleaseTime = now;
    }
  }

  if (isWaitingForClick && (now - lastReleaseTime >= DOUBLE_CLICK_GAP_MS)) {
    event = SINGLE_CLICK;
    isWaitingForClick = false;
  }

  lastBtnState = currentState;
  return event;
}
