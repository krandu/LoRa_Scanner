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

// ================= 频率与参数列表 =================
const float FREQ_LIST[] = { 438.150, 438.125, 438.000, 438.500 }; 
const int FREQ_COUNT = sizeof(FREQ_LIST) / sizeof(FREQ_LIST[0]);
int freqIdx = 0;
float currentFreq = FREQ_LIST[0];

long signalBandwidth = 125E3; // 125kHz

struct LoRaCombo {
  uint8_t sf;
  uint8_t cr;
};

const LoRaCombo COMBO_LIST[] = {
  {12, 5}, // 默认首选 SF12 / CR5
  {7,  5},
  {8,  5},
  {9,  5},
  {10, 5},
  {11, 5},
  {7,  8},
  {12, 8}
};
const int COMBO_COUNT = sizeof(COMBO_LIST) / sizeof(COMBO_LIST[0]);
int comboIdx = 0; 

// ================= 信号与解码数据缓冲区 =================
#define BOX_X 8
#define BOX_Y 12
#define BOX_W 112
#define BOX_H 32
#define RSSI_HIST_LEN 112

float rssiHistory[RSSI_HIST_LEN];
int rssiWrIdx = 0;
unsigned long lastSampleMs = 0;

// 信号锁定与解码结果存储
bool isLocked = false;             // 锁定状态标志
String decodedPayload = "";        // 清洗解码后的文本数据
int lastPacketRssi = 0;            // 解码数据包的 RSSI
float lastPacketSnr = 0.0;         // 解码数据包的 SNR

// ================= 按键检测状态机 =================
enum BtnEvent { NONE, SINGLE_CLICK, DOUBLE_CLICK, LONG_PRESS };
unsigned long btnPressTime = 0;
unsigned long lastReleaseTime = 0;
bool lastBtnState = HIGH;
bool isWaitingForClick = false;

// ================= 菜单系统 =================
bool inMenu = false;
int menuSelection = 0; 
const int MENU_ITEMS = 2;

// ================= 函数声明 =================
void applyLoRaConfig();
void sampleRSSI();
void drawMainDisplay();
void drawMenuDisplay();
BtnEvent checkButton();
void enterDeepSleep();

void setup() {
  Serial.begin(115200);
  pinMode(PRG_BUTTON_PIN, INPUT_PULLUP);

  // 1. 开启外设电源 VEXT (GPIO 21 拉低)
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

  applyLoRaConfig();
}

void loop() {
  BtnEvent evt = checkButton();

  // 双击：切换系统菜单 / 主界面
  if (evt == DOUBLE_CLICK) {
    inMenu = !inMenu;
  }

  if (inMenu) {
    // ---- 菜单模式 ----
    if (evt == SINGLE_CLICK) {
      menuSelection = (menuSelection + 1) % MENU_ITEMS;
    } else if (evt == LONG_PRESS) {
      if (menuSelection == 0) {
        ESP.restart();
      } else if (menuSelection == 1) {
        enterDeepSleep();
      }
    }
    drawMenuDisplay();
  } else {
    // ---- 主显示运行模式 ----
    
    // 1. 锁定状态下的按键控制
    if (isLocked) {
      if (evt == LONG_PRESS || evt == SINGLE_CLICK) {
        // 解除锁定：按键直接退出锁定，恢复常规扫描
        isLocked = false;
        decodedPayload = "";
        Serial.println("[System] Unlocked by user.");
      }
    } 
    // 2. 未锁定状态下的按键控制
    else {
      if (evt == SINGLE_CLICK) {
        // 短按：循环切换 SF / CR 参数组合
        comboIdx = (comboIdx + 1) % COMBO_COUNT;
        applyLoRaConfig();
      } else if (evt == LONG_PRESS) {
        // 长按：手动循环切换工作频率
        freqIdx = (freqIdx + 1) % FREQ_COUNT;
        currentFreq = FREQ_LIST[freqIdx];
        applyLoRaConfig();
      }
    }

    // 定时采样 RSSI 记录走势
    if (millis() - lastSampleMs >= 30) {
      lastSampleMs = millis();
      sampleRSSI();
    }

    // 3. 尝试接收解析 LoRa 数据包
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String incoming = "";
      while (LoRa.available()) {
        incoming += (char)LoRa.read();
      }
      
      isLocked = true;
      
      // APRS 报头过滤清洗
      int aprsMsgIdx = incoming.indexOf("::");
      if (aprsMsgIdx != -1) {
        decodedPayload = incoming.substring(aprsMsgIdx + 2);
      } else {
        decodedPayload = incoming;
      }

      lastPacketRssi = LoRa.packetRssi();
      lastPacketSnr = LoRa.packetSnr();

      Serial.printf("[DEC] Raw: %s\n", incoming.c_str());
      Serial.printf("[DEC] Parsed: %s | RSSI: %d | SNR: %.2f\n", 
                    decodedPayload.c_str(), lastPacketRssi, lastPacketSnr);
    }

    drawMainDisplay();
  }
}

void applyLoRaConfig() {
  LoRa.setFrequency(currentFreq * 1E6);
  LoRa.setSignalBandwidth(signalBandwidth);
  LoRa.setSpreadingFactor(COMBO_LIST[comboIdx].sf);
  LoRa.setCodingRate4(COMBO_LIST[comboIdx].cr);
  LoRa.receive();
  
  Serial.printf("[LoRa Config] Freq: %.3f MHz | SF: %d | CR: 4/%d\n", 
                currentFreq, COMBO_LIST[comboIdx].sf, COMBO_LIST[comboIdx].cr);
}

void sampleRSSI() {
  float rawRssi = LoRa.packetRssi(); 
  if (rawRssi == 0) rawRssi = -120;
  rssiHistory[rssiWrIdx] = rawRssi;
  rssiWrIdx = (rssiWrIdx + 1) % RSSI_HIST_LEN;
}

void drawMainDisplay() {
  display.clearDisplay();

  // 1. 顶部状态栏：左侧频率，右侧显示 [LOCKED] 或实时 RSSI
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("%.3fMHz", currentFreq);
  
  display.setCursor(75, 0);
  if (isLocked) {
    display.print("[LOCKED]"); 
  } else {
    int latestIdx = (rssiWrIdx - 1 + RSSI_HIST_LEN) % RSSI_HIST_LEN;
    display.printf("%ddBm", (int)rssiHistory[latestIdx]);
  }

  // 2. 边框外壳
  display.drawRect(BOX_X - 1, BOX_Y - 1, BOX_W + 2, BOX_H + 2, SSD1306_WHITE);

  // 3. 方框区内容
  if (isLocked) {
    // === 锁定状态：显示解码数据与包信号状态 ===
    display.setTextSize(1);
    display.setTextWrap(true);
    
    // 超长文本截断并用 ... 结尾
    String dispText = decodedPayload;
    if (dispText.length() > 33) {
      dispText = dispText.substring(0, 30) + "...";
    }

    display.setCursor(BOX_X + 2, BOX_Y + 2);
    if (dispText.length() > 0) {
      display.print(dispText);
    } else {
      display.print("<EMPTY>");
    }

    // 保持原本位置与格式显示 RSSI 和 SNR
    display.setCursor(BOX_X + 2, BOX_Y + 22);
    display.printf("R:%ddBm S:%.1fdB", lastPacketRssi, lastPacketSnr);

  } else {
    // === 未锁定状态：绘制 RSSI 历史时序波形 ===
    display.setTextWrap(false);
    for (int col = 0; col < BOX_W; col++) {
      int idx = (rssiWrIdx + col) % RSSI_HIST_LEN;
      float val = rssiHistory[idx];
      
      int lineH = map((int)constrain(val, -120, -30), -120, -30, 0, BOX_H);
      if (lineH > 0) {
        display.drawFastVLine(BOX_X + col, BOX_Y + BOX_H - lineH, lineH, SSD1306_WHITE);
      }
    }
  }

  // 4. 底部状态栏：CR 简化显示（如 CR5）
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

void drawMenuDisplay() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(32, 4);
  display.println("= SYSTEM MENU =");
  display.drawFastHLine(0, 16, 128, SSD1306_WHITE);

  if (menuSelection == 0) {
    display.fillRect(10, 24, 108, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }
  display.setCursor(15, 27);
  display.println("1. Reboot System");

  if (menuSelection == 1) {
    display.fillRect(10, 42, 108, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }
  display.setCursor(15, 45);
  display.println("2. Deep Sleep (OFF)");

  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.display();
}

void enterDeepSleep() {
  display.clearDisplay();
  display.setCursor(30, 28);
  display.println("Powering Off...");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();

  // 关机时关闭外设电源
  digitalWrite(VEXT_CTRL_PIN, HIGH);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)PRG_BUTTON_PIN, 0);
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
