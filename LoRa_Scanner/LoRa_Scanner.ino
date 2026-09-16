#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_sleep.h>

// ================= 硬件引脚配置 =================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

#define VEXT_CTRL_PIN 21   // 控制屏幕及电池分压电路供电 (LOW 为激活)
#define OLED_SDA      4
#define OLED_SCL      15
#define OLED_RST      16 
#define VBAT_ADC_PIN  37   // Heltec V2 板载电池采样引脚 (ADC1_CH1)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

#define SCK_PIN   5
#define MISO_PIN  19
#define MOSI_PIN  27
#define SS_PIN    18
#define RST_PIN   14
#define DIO0_PIN  26

#define PRG_BUTTON_PIN 0 

// ================= 定时与休眠配置 =================
const unsigned long AUTO_POWER_OFF_MS = 10 * 60 * 1000UL; // 10分钟无操作自动关机
unsigned long lastActivityMs = 0;

// ================= 模式与状态 =================
enum SystemMode { MODE_SPECTRUM, MODE_LORA_ANALYZER };
SystemMode currentMode = MODE_SPECTRUM; 

// 优化：增加分段 Span/步进 结构
struct SpectrumSpanOption {
  float spanMHz;
  float stepkHz;
};
SpectrumSpanOption spanOptions[] = {
  {10.0, 250.0}, // 全景扫描
  {5.0,  125.0}, // 中等细化
  {2.0,  50.0}   // 高精窄带细化 (可以清晰看清窄带LoRa/手台信号)
};
const int SPAN_COUNT = sizeof(spanOptions) / sizeof(spanOptions[0]);
int spanIdx = 0;

float specStartFreq = 430.0;
float specEndFreq   = 440.0;
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

// 峰值防抖相关变量
float rawPeakRssi = -160.0;
float rawPeakFreq = 430.0;
float displayedPeakFreq = 430.0;
float displayedPeakRssi = -160.0;
unsigned long lastPeakUpdateMs = 0;

// 新增：底噪平滑与保持变量
float smoothedNoiseFloor = -100.0;
float displayedNoiseFloor = -100.0;
unsigned long lastNoiseUpdateMs = 0;
const unsigned long NOISE_HOLD_TIME_MS = 1500; // 1.5秒刷新一次底噪数字

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

// 按键与菜单
enum BtnEvent { NONE, SINGLE_CLICK, DOUBLE_CLICK, LONG_PRESS };
unsigned long btnPressTime = 0;
unsigned long lastReleaseTime = 0;
bool lastBtnState = HIGH;
bool isWaitingForClick = false;

bool inMenu = false;
int menuSelection = 0; 
const int MENU_ITEMS = 3; 

// 函数声明
void applyLoRaConfig();
void runSpectrumScan();
void sampleRSSI();
void drawMainDisplay();
void drawSpectrumDisplay();
void drawMenuDisplay();
BtnEvent checkButton();
float readBatteryVoltage();
int getBatteryPercent(float vbat);
void powerOff();
void updateSpanFreqs();

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n--- Heltec WiFi LoRa 32 V2 Spectrum Fixed ---");

  pinMode(PRG_BUTTON_PIN, INPUT_PULLUP);

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

  // 配置 ESP32 ADC 采样
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

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
  
  LoRa.receive();
  for (int i = 0; i < RSSI_HIST_LEN; i++) rssiHistory[i] = -120.0;
  memset(waterfallBuf, 0, sizeof(waterfallBuf));

  lastActivityMs = millis();
}

void updateSpanFreqs() {
  float currentCenter = 435.0; // 中心频点设定在 435 MHz
  float halfSpan = spanOptions[spanIdx].spanMHz / 2.0;
  specStartFreq = currentCenter - halfSpan;
  specEndFreq   = currentCenter + halfSpan;
}

void loop() {
  // 1. 自动关机检测
  if (millis() - lastActivityMs > AUTO_POWER_OFF_MS) {
    powerOff();
  }

  // 2. 检测按键事件
  BtnEvent evt = checkButton();
  if (evt != NONE) {
    lastActivityMs = millis();
  }

  if (evt == DOUBLE_CLICK) {
    inMenu = !inMenu;
    if (inMenu) menuSelection = (currentMode == MODE_SPECTRUM) ? 0 : 1;
  }

  // 3. 菜单控制
  if (inMenu) {
    if (evt == SINGLE_CLICK) {
      menuSelection = (menuSelection + 1) % MENU_ITEMS;
    } else if (evt == LONG_PRESS) {
      if (menuSelection == 0) {
        currentMode = MODE_SPECTRUM;
      } else if (menuSelection == 1) {
        currentMode = MODE_LORA_ANALYZER;
        applyLoRaConfig();
      } else if (menuSelection == 2) {
        powerOff();
      }
      inMenu = false;
    }
    drawMenuDisplay();
  } 
  // 4. 频谱扫描模式 (支持按键切换 Span)
  else if (currentMode == MODE_SPECTRUM) {
    if (evt == SINGLE_CLICK) {
      // 单击：切换 Span (10M -> 5M -> 2M 细化扫描)
      spanIdx = (spanIdx + 1) % SPAN_COUNT;
      updateSpanFreqs();
    } else if (evt == LONG_PRESS) {
      // 长按：复位 Peak 清屏
      displayedPeakRssi = -160.0;
    }

    runSpectrumScan();
    drawSpectrumDisplay();
  } 
  // 5. LoRa 分析模式
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

// 频谱扫描与底噪/峰值算法
void runSpectrumScan() {
  float step = (specEndFreq - specStartFreq) / SPEC_CHANNELS;
  maxFoundRssi = -160.0;
  minFoundRssi = 0.0;
  float rssiSum = 0.0;

  for (int i = 0; i < SPEC_CHANNELS; i++) {
    float freq = specStartFreq + i * step;
    LoRa.setFrequency(freq * 1E6);
    LoRa.receive(); 
    delayMicroseconds(1800); 
    
    float val = LoRa.rssi();
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

  // 1. 底噪平滑与保持算法
  float instantAvgNoise = rssiSum / SPEC_CHANNELS;
  smoothedNoiseFloor = (smoothedNoiseFloor * 0.85) + (instantAvgNoise * 0.15); // 一阶低通滤波
  
  if (millis() - lastNoiseUpdateMs > NOISE_HOLD_TIME_MS) {
    displayedNoiseFloor = smoothedNoiseFloor;
    lastNoiseUpdateMs = millis();
  }

  // 2. 峰值防抖算法
  if ((maxFoundRssi > displayedPeakRssi + 3.0) || (millis() - lastPeakUpdateMs > 1500)) {
    displayedPeakFreq = rawPeakFreq;
    displayedPeakRssi = maxFoundRssi;
    lastPeakUpdateMs = millis();
  }

  // 3. 瀑布图更新
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

  // 顶栏：显示频率与峰值
  char topBuf[16];
  snprintf(topBuf, sizeof(topBuf), "P:%.2f", displayedPeakFreq);
  display.setCursor(0, 0);
  display.print(topBuf);

  // 顶栏右侧：平滑后防抖的电池百分比
  float vbat = readBatteryVoltage();
  int pct = getBatteryPercent(vbat);
  char batStr[8];
  snprintf(batStr, sizeof(batStr), "%d%%", pct);
  int batX = SCREEN_WIDTH - (strlen(batStr) * 6);
  display.setCursor(batX, 0);
  display.print(batStr);

  // 1. 绘制频谱柱状图边框
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

  // 嵌入：在频谱框内部右上角绘制背景遮罩层与保持后的底噪平均值 (NF: -XXdBm)
  char noiseBuf[12];
  snprintf(noiseBuf, sizeof(noiseBuf), "NF:%.0f", displayedNoiseFloor);
  int noiseTextW = strlen(noiseBuf) * 6;
  int noiseBoxX = (SPEC_BOX_X + SPEC_BOX_W) - noiseTextW - 3;
  
  // 黑色反空遮罩，防止背景频谱线与文字重叠
  display.fillRect(noiseBoxX - 1, SPEC_BOX_Y + 2, noiseTextW + 2, 9, SSD1306_BLACK);
  display.setCursor(noiseBoxX, SPEC_BOX_Y + 3);
  display.print(noiseBuf);

  // 2. 瀑布图绘制
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

  // 3. 底栏：动态显示 Span 和计算后的 Step 细化步进
  display.setCursor(0, 56);
  display.printf("SPAN:%.0fM", spanOptions[spanIdx].spanMHz);
  
  display.setCursor(70, 56); 
  display.printf("STEP:%.0fK", spanOptions[spanIdx].stepkHz);

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

void drawMenuDisplay() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(20, 2);
  display.println("= SELECT MODE =");
  display.drawFastHLine(0, 12, 128, SSD1306_WHITE);

  const char* items[] = {"1. Spectrum Scan", "2. LoRa Receiver", "3. Power Off"};

  for (int i = 0; i < 3; i++) {
    int yPos = 16 + i * 16;
    if (menuSelection == i) {
      display.fillRect(10, yPos, 108, 14, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    display.setCursor(15, yPos + 3);
    display.println(items[i]);
  }

  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.display();
}

// 修正： Heltec WiFi LoRa 32 V2 板载 100K/390K 分压，精准分压系数约为 4.9
float readBatteryVoltage() {
  pinMode(VEXT_CTRL_PIN, OUTPUT);
  digitalWrite(VEXT_CTRL_PIN, LOW);
  delayMicroseconds(500);

  int rawSum = 0;
  for (int i = 0; i < 10; i++) {
    rawSum += analogRead(VBAT_ADC_PIN);
  }
  float raw = rawSum / 10.0;
  
  // 关键校准系数：使用 4.9 替换原 3.2
  float instantVbat = (raw / 4095.0) * 3.3 * 4.9; 

  // 电压一阶平滑滤波
  if (smoothedVbat == 0.0) smoothedVbat = instantVbat;
  smoothedVbat = (smoothedVbat * 0.9) + (instantVbat * 0.1);

  return smoothedVbat;
}

// 修正：加迟滞防抖的电池百分比算法
int getBatteryPercent(float vbat) {
  int calcPct = 0;
  if (vbat >= 4.15)      calcPct = 100;
  else if (vbat <= 3.30) calcPct = 0;
  else calcPct = (int)((vbat - 3.30) / (4.15 - 3.30) * 100.0);
  
  calcPct = constrain(calcPct, 0, 100);

  // 滞后比较 (Hysteresis)：防止 39% 与 40% 频繁交替跳动
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

  digitalWrite(VEXT_CTRL_PIN, HIGH); // 切断 VEXT 供电
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
