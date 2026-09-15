#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= 引脚与硬件配置 =================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// LoRa 引脚
#define SCK_PIN   5
#define MISO_PIN  19
#define MOSI_PIN  27
#define SS_PIN    18
#define RST_PIN   14
#define DIO0_PIN  26

#define PRG_BUTTON_PIN 0  // PRG 按键 GPIO 0

// ================= 频率配置与切换列表 =================
const float FREQ_LIST[] = { 438.150, 438.125, 438.000, 438.500 }; // 常用频率切换列表 (MHz)
const int FREQ_COUNT = sizeof(FREQ_LIST) / sizeof(FREQ_LIST[0]);
int freqIdx = 0; // 当前选中的频率索引
float currentFreq = FREQ_LIST[0];

long signalBandwidth = 125E3; // 125kHz

struct LoRaCombo {
  uint8_t sf;
  uint8_t cr;
};

// 扫描参数列表 (包含 SF12 CR5)
const LoRaCombo COMBO_LIST[] = {
  {7,  5},
  {8,  5},
  {9,  5},
  {10, 5},
  {11, 5},
  {12, 5}, // SF12 / CR5
  {7,  8},
  {12, 8}
};
const int COMBO_COUNT = sizeof(COMBO_LIST) / sizeof(COMBO_LIST[0]);

int comboIdx = 0;
bool locked = false;

// 自动扫描控制
unsigned long lastScanMs = 0;
const unsigned long SCAN_INTERVAL_MS = 800;

// 频谱历史缓冲区 (112 像素宽)
#define BOX_X 8
#define BOX_Y 12
#define BOX_W 112
#define BOX_H 32
#define RSSI_HIST_LEN 112

float rssiHistory[RSSI_HIST_LEN];
int rssiWrIdx = 0;
unsigned long lastSampleMs = 0;

// ================= 按键状态机 (双击/长按) =================
enum BtnEvent { NONE, SINGLE_CLICK, DOUBLE_CLICK, LONG_PRESS };
unsigned long btnPressTime = 0;
unsigned long lastReleaseTime = 0;
bool lastBtnState = HIGH;
bool isWaitingForClick = false;

// ================= 菜单系统 =================
bool inMenu = false;
int menuSelection = 0; // 0: Reboot, 1: Deep Sleep
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

  // 初始化 OLED
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

  // 初始化 SPI & LoRa
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

  // 初始化 RSSI 数组
  for (int i = 0; i < RSSI_HIST_LEN; i++) rssiHistory[i] = -120.0;

  applyLoRaConfig();
}

void loop() {
  BtnEvent evt = checkButton();

  // 双击进入/退出系统菜单
  if (evt == DOUBLE_CLICK) {
    inMenu = !inMenu;
  }

  if (inMenu) {
    // ---- 菜单模式控制 ----
    if (evt == SINGLE_CLICK) {
      menuSelection = (menuSelection + 1) % MENU_ITEMS; // 切换菜单选项
    } else if (evt == LONG_PRESS) {
      if (menuSelection == 0) {
        ESP.restart(); // 重启
      } else if (menuSelection == 1) {
        enterDeepSleep(); // 关机休眠
      }
    }
    drawMenuDisplay();
  } else {
    // ---- 正常工作模式 ----
    if (evt == SINGLE_CLICK) {
      // 单击：手动切换下一个频率点
      freqIdx = (freqIdx + 1) % FREQ_COUNT;
      currentFreq = FREQ_LIST[freqIdx];
      locked = false; // 切换频率后解除锁定，重新开启扫描
      applyLoRaConfig();
    }

    // 自动扫描 SF / CR 参数
    if (!locked && (millis() - lastScanMs >= SCAN_INTERVAL_MS)) {
      lastScanMs = millis();
      comboIdx = (comboIdx + 1) % COMBO_COUNT;
      applyLoRaConfig();
    }

    // 定时采样 RSSI 刷新频谱
    if (millis() - lastSampleMs >= 30) {
      lastSampleMs = millis();
      sampleRSSI();
    }

    // LoRa 数据包接收检测
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      locked = true; // 抓到数据包，锁定当前 SF/CR
      Serial.printf("Packet received on %.3f MHz SF%d CR4/%d, RSSI: %d\n", 
                    currentFreq, COMBO_LIST[comboIdx].sf, COMBO_LIST[comboIdx].cr, LoRa.packetRssi());
    }

    drawMainDisplay();
  }
}

// 应用当前的 LoRa 参数 (包括频率、带宽、SF、CR)
void applyLoRaConfig() {
  LoRa.setFrequency(currentFreq * 1E6);
  LoRa.setSignalBandwidth(signalBandwidth);
  LoRa.setSpreadingFactor(COMBO_LIST[comboIdx].sf);
  LoRa.setCodingRate4(COMBO_LIST[comboIdx].cr);
  LoRa.receive();
  
  Serial.printf("Freq: %.3f MHz | SF: %d | CR: 4/%d\n", 
                currentFreq, COMBO_LIST[comboIdx].sf, COMBO_LIST[comboIdx].cr);
}

// 采样 RSSI
void sampleRSSI() {
  float rawRssi = LoRa.packetRssi(); 
  if (rawRssi == 0) rawRssi = -120; // 防止未触发时的空值
  rssiHistory[rssiWrIdx] = rawRssi;
  rssiWrIdx = (rssiWrIdx + 1) % RSSI_HIST_LEN;
}

// 绘制主界面
void drawMainDisplay() {
  display.clearDisplay();

  // 1. 顶部状态栏: 频率与锁定状态
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("%.3fMHz", currentFreq);
  
  display.setCursor(85, 0);
  if (locked) {
    display.print("[LOCK]");
  } else {
    display.print("[SCAN]");
  }

  // 2. 绘制频谱框外壳
  display.drawRect(BOX_X - 1, BOX_Y - 1, BOX_W + 2, BOX_H + 2, SSD1306_WHITE);

  // 3. 绘制 RSSI 波形 (由右向左移动)
  for (int col = 0; col < BOX_W; col++) {
    int idx = (rssiWrIdx + col) % RSSI_HIST_LEN;
    float val = rssiHistory[idx];
    
    // 将 RSSI (-120dBm 到 -30dBm) 映射到 0 ~ BOX_H 像素
    int lineH = map((int)constrain(val, -120, -30), -120, -30, 0, BOX_H);
    if (lineH > 0) {
      display.drawFastVLine(BOX_X + col, BOX_Y + BOX_H - lineH, lineH, SSD1306_WHITE);
    }
  }

  // 4. 底部显示 (左下角：带宽，右下角：SF/CR)
  int bottomY = 52;

  // 左下角：带宽显示
  display.setCursor(0, bottomY);
  display.printf("BW:%.0fK", signalBandwidth / 1000.0);

  // 右下角：SF/CR 参数显示
  char sfCrBuf[16];
  snprintf(sfCrBuf, sizeof(sfCrBuf), "SF%d/CR4/%d", COMBO_LIST[comboIdx].sf, COMBO_LIST[comboIdx].cr);
  int xPos = SCREEN_WIDTH - (strlen(sfCrBuf) * 6);
  display.setCursor(xPos, bottomY);
  display.print(sfCrBuf);

  display.display();
}

// 绘制系统菜单
void drawMenuDisplay() {
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setCursor(32, 4);
  display.println("= SYSTEM MENU =");
  display.drawFastHLine(0, 16, 128, SSD1306_WHITE);

  // 菜单项 1: Reboot
  if (menuSelection == 0) {
    display.fillRect(10, 24, 108, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }
  display.setCursor(15, 27);
  display.println("1. Reboot System");

  // 菜单项 2: Deep Sleep
  if (menuSelection == 1) {
    display.fillRect(10, 42, 108, 14, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  }
  display.setCursor(15, 45);
  display.println("2. Deep Sleep (OFF)");

  // 还原前景色
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
  display.display();
}

// 进入低功耗关机休眠
void enterDeepSleep() {
  display.clearDisplay();
  display.setCursor(30, 28);
  display.println("Powering Off...");
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();

  // 配置 GPIO 0 低电平唤醒（按 PRG 键重新开机）
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PRG_BUTTON_PIN, 0);
  esp_deep_sleep_start();
}

// 按键检测状态机 (处理单击、双击、长按)
BtnEvent checkButton() {
  bool currentState = digitalRead(PRG_BUTTON_PIN);
  unsigned long now = millis();
  BtnEvent event = NONE;

  if (lastBtnState == HIGH && currentState == LOW) {
    btnPressTime = now;
  } else if (lastBtnState == LOW && currentState == HIGH) {
    unsigned long pressDuration = now - btnPressTime;
    
    if (pressDuration >= 1500) {
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
