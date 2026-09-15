#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= 硬件引脚与电源配置 =================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

#define VEXT_CTRL_PIN 21 // Heltec OLED 供电控制 (LOW = 开启)
#define OLED_SDA      4
#define OLED_SCL      15
#define OLED_RST      16 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// LoRa 引脚 (Heltec WiFi LoRa 32 V2)
#define SCK_PIN   5
#define MISO_PIN  19
#define MOSI_PIN  27
#define SS_PIN    18
#define RST_PIN   14
#define DIO0_PIN  26

#define PRG_BUTTON_PIN 0 

// ================= 运行模式定义 =================
enum SystemMode { MODE_SPECTRUM, MODE_LORA_ANALYZER };
SystemMode currentMode = MODE_SPECTRUM; // 默认启动为频谱分析模式

// ================= 频谱扫描配置 (430MHz - 440MHz) =================
#define SPEC_START_FREQ  430.0
#define SPEC_END_FREQ    440.0
#define SPEC_CHANNELS    40     // 40 个通道 (步长 250kHz)
#define SPEC_BOX_X       4
#define SPEC_BOX_Y       12
#define SPEC_BOX_W       120
#define SPEC_BOX_H       20
#define WATERFALL_Y      34
#define WATERFALL_H      16

float specRssi[SPEC_CHANNELS];
uint8_t waterfallBuf[WATERFALL_H][SPEC_CHANNELS]; // 瀑布图历史数据缓存

float peakRssiFreq = 430.0;
float maxFoundRssi = -120.0;

// ================= LoRa 模式配置与缓冲区 =================
const float FREQ_LIST[] = { 438.150, 438.125, 438.000, 438.500 }; 
const int FREQ_COUNT = sizeof(FREQ_LIST) / sizeof(FREQ_LIST[0]);
int freqIdx = 0;
float currentFreq = FREQ_LIST[0];
long signalBandwidth = 125E3; // 125kHz

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

// ================= 按键状态机 =================
enum BtnEvent { NONE, SINGLE_CLICK, DOUBLE_CLICK, LONG_PRESS };
unsigned long btnPressTime = 0;
unsigned long lastReleaseTime = 0;
bool lastBtnState = HIGH;
bool isWaitingForClick = false;

// ================= 菜单系统 =================
bool inMenu = false;
int menuSelection = 0; 
const int MENU_ITEMS = 2; // 0: 频谱分析, 1: LoRa 模式

// ================= 函数声明 =================
void applyLoRaConfig();
void runSpectrumScan();
void sampleRSSI();
void drawMainDisplay();
void drawSpectrumDisplay();
void drawMenuDisplay();
BtnEvent checkButton();
void enterDeepSleep();

void setup() {
  Serial.begin(115200);
  pinMode(PRG_BUTTON_PIN, INPUT_PULLUP);

  // 1. 开启外设电源 VEXT
  pinMode(VEXT_CTRL_PIN, OUTPUT);
  digitalWrite(VEXT_CTRL_PIN, LOW); 
  delay(50);

  // 2. 复位 OLED 硬件
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  // 3. 初始化 I2C 与 OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(20, 25);
  display.println(F("Initializing..."));
  display.display();

  // 4. 初始化 SPI & LoRa
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(currentFreq * 1E6)) {
    Serial.println("LoRa Init Failed!");
    display.clearDisplay();
    display.setCursor(20, 25);
    display.println(F("LoRa Init Failed!"));
    display.display();
    while (1);
  }

  for (int i = 0; i < RSSI_HIST_LEN; i++) rssiHistory[i] = -120.0;
  memset(waterfallBuf, 0, sizeof(waterfallBuf));

  if (currentMode == MODE_LORA_ANALYZER) {
    applyLoRaConfig();
  }
}

void loop() {
  BtnEvent evt = checkButton();

  // 双击：唤出模式切换菜单
  if (evt == DOUBLE_CLICK) {
    inMenu = !inMenu;
    if (inMenu) menuSelection = (currentMode == MODE_SPECTRUM) ? 0 : 1;
  }

  if (inMenu) {
    // ---- 模式切换菜单模式 ----
    if (evt == SINGLE_CLICK) {
      menuSelection = (menuSelection + 1) % MENU_ITEMS;
    } else if (evt == LONG_PRESS) {
      if (menuSelection == 0) {
        currentMode = MODE_SPECTRUM;
      } else if (menuSelection == 1) {
        currentMode = MODE_LORA_ANALYZER;
        applyLoRaConfig();
      }
      inMenu = false; // 切换并退出菜单
    }
    drawMenuDisplay();
  } 
  else if (currentMode == MODE_SPECTRUM) {
    // ---- 频谱分析模式 ----
    runSpectrumScan();
    drawSpectrumDisplay();
  } 
  else {
    // ---- LoRa 接收分析模式 ----
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

// 快速全频段 RSSI 扫描
void runSpectrumScan() {
  float step = (SPEC_END_FREQ - SPEC_START_FREQ) / SPEC_CHANNELS;
  maxFoundRssi = -120.0;

  for (int i = 0; i < SPEC_CHANNELS; i++) {
    float freq = SPEC_START_FREQ + i * step;
    LoRa.setFrequency(freq * 1E6);
    delayMicroseconds(2500); // 频点切换建立延时
    
    float val = LoRa.packetRssi();
    if (val == 0) val = -120.0;
    specRssi[i] = val;

    if (val > maxFoundRssi) {
      maxFoundRssi = val;
      peakRssiFreq = freq;
    }
  }

  // 瀑布图向下平移更新
  for (int y = WATERFALL_H - 1; y > 0; y--) {
    for (int x = 0; x < SPEC_CHANNELS; x++) {
      waterfallBuf[y][x] = waterfallBuf[y - 1][x];
    }
  }

  // 编码顶行瀑布图数值 (0: 弱, 1: 中, 2: 强)
  for (int x = 0; x < SPEC_CHANNELS; x++) {
    if (specRssi[x] > -75)       waterfallBuf[0][x] = 2;
    else if (specRssi[x] > -95)  waterfallBuf[0][x] = 1;
    else                         waterfallBuf[0][x] = 0;
  }
}

// 绘制频谱图 + 瀑布图界面
void drawSpectrumDisplay() {
  display.clearDisplay();

  // 1. 顶部状态栏: 显示范围与峰值频点
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("430-440M");
  
  char peakBuf[16];
  snprintf(peakBuf, sizeof(peakBuf), "PK:%.2f", peakRssiFreq);
  int peakX = SCREEN_WIDTH - (strlen(peakBuf) * 6);
  display.setCursor(peakX, 0);
  display.print(peakBuf);

  // 2. 频谱柱状图 (SPEC_BOX_X 到 SPEC_BOX_X + 120)
  int colWidth = SPEC_BOX_W / SPEC_CHANNELS; // 3px 每通道
  for (int i = 0; i < SPEC_CHANNELS; i++) {
    int lineH = map((int)constrain(specRssi[i], -115, -30), -115, -30, 0, SPEC_BOX_H);
    int xPos = SPEC_BOX_X + i * colWidth;
    if (lineH > 0) {
      display.fillRect(xPos, SPEC_BOX_Y + SPEC_BOX_H - lineH, colWidth - 1, lineH, SSD1306_WHITE);
    }
  }
  display.drawFastHLine(SPEC_BOX_X - 2, SPEC_BOX_Y + SPEC_BOX_H, SPEC_BOX_W + 4, SSD1306_WHITE);

  // 3. 瀑布图绘制
  for (int y = 0; y < WATERFALL_H; y++) {
    for (int i = 0; i < SPEC_CHANNELS; i++) {
      int xPos = SPEC_BOX_X + i * colWidth;
      uint8_t val = waterfallBuf[y][i];

      if (val == 2) {
        // 强信号：完全点亮像素块
        display.fillRect(xPos, WATERFALL_Y + y, colWidth - 1, 1, SSD1306_WHITE);
      } else if (val == 1) {
        // 中等信号：抖动点阵 (间隔点亮)
        if ((i + y) % 2 == 0) {
          display.drawPixel(xPos, WATERFALL_Y + y, SSD1306_WHITE);
        }
      }
    }
  }

  // 4. 底部扫描信息栏
  display.setCursor(0, 56);
  display.printf("SPAN:10M STEP:250K");

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
  float rawRssi = LoRa.packetRssi(); 
  if (rawRssi == 0) rawRssi = -120;
  rssiHistory[rssiWrIdx] = rawRssi;
  rssiWrIdx = (rssiWrIdx + 1) % RSSI_HIST_LEN;
}

void drawMainDisplay() {
  display.clearDisplay();

  // 1. 顶部状态栏
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("%.3fMHz", currentFreq);
  
  char rightTopBuf[16];
  if (isLocked) {
    snprintf(rightTopBuf, sizeof(rightTopBuf), "[LOCKED]");
  } else {
    int latestIdx = (rssiWrIdx - 1 + RSSI_HIST_LEN) % RSSI_HIST_LEN;
    snprintf(rightTopBuf, sizeof(rightTopBuf), "%ddBm", (int)rssiHistory[latestIdx]);
  }
  int rightTopX = SCREEN_WIDTH - (strlen(rightTopBuf) * 6);
  display.setCursor(rightTopX, 0);
  display.print(rightTopBuf);

  // 2. 边框外壳
  display.drawRect(BOX_X - 1, BOX_Y - 1, BOX_W + 2, BOX_H + 2, SSD1306_WHITE);

  // 3. 方框区内容
  if (isLocked) {
    display.setTextSize(1);
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

  // 4. 底部状态栏
  int bottomY = 52;
  display.setTextSize(1);
  display.setCursor(0, bottomY);
  display.printf("BW:%.0fK", signalBandwidth / 1000.0);

  char sfCrBuf[16];
  snprintf(sfCrBuf, sizeof(sfCrBuf), "SF%d/CR%d", COMBO_LIST[comboIdx].sf, COMBO_LIST[comboIdx].cr);
  int xPos = SCREEN_WIDTH - (strlen(sfCrBuf) * 6);
  display.setCursor(xPos, bottomY);
  display.print(sfCrBuf);

  display.display();
}

// 绘制模式选择菜单
void drawMenuDisplay() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(20, 4);
  display.println("= SELECT MODE =");
  display.drawFastHLine(0, 16, 128, SSD1306_WHITE);

  if (menuSelection == 0) {
    display.fillRect(10, 24, 108, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }
  display.setCursor(15, 27);
  display.println("1. Spectrum Scanner");

  if (menuSelection == 1) {
    display.fillRect(10, 42, 108, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }
  display.setCursor(15, 45);
  display.println("2. LoRa Receiver");

  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.display();
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
