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

#define FW_VERSION "v4-presets"   // 开机画面显示，用于确认刷入的是新固件

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
// 预设频率表（可在"Freq Presets"菜单中增删改，保存在 Flash）
#define MAX_PRESETS 8
const uint32_t DEFAULT_PRESETS_KHZ[] = { 438150, 438125, 438000, 438500 };
const int DEFAULT_PRESET_COUNT = sizeof(DEFAULT_PRESETS_KHZ) / sizeof(DEFAULT_PRESETS_KHZ[0]);
uint32_t presetKHz[MAX_PRESETS] = { 438150, 438125, 438000, 438500 };
int presetCount = DEFAULT_PRESET_COUNT;
int freqIdx = 0;
float currentFreq = 438.150;
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
const int MENU_ITEMS = 5; // 1.Spectrum 2.LoRa 3.Freq Presets 4.Calib Bat 5.Power Off

// ===== 手动设置频率界面 =====
bool inFreqUI = false;
int  freqEditPos = 0;                       // 0~5 = 六位数字，6 = 保存
int  freqDigits[6] = {4, 3, 8, 1, 5, 0};    // 438.150 MHz
const uint32_t FREQ_MIN_KHZ = 410000;       // SX1278 433 频段下限
const uint32_t FREQ_MAX_KHZ = 525000;       // SX1278 433 频段上限
int  freqEditTarget = -2;                   // -2=手动设置并直接应用, -1=新建预设, >=0=编辑该预设

// ===== 预设频率管理界面 =====
bool inPresetUI = false;
int  presetPage = 0;      // 0=预设列表, 1=操作菜单
int  presetSel = 0;       // 列表选中项：0..presetCount-1 为预设，presetCount 为"新建"，presetCount+1 为"手动频率"
int  presetActSel = 0;    // 操作菜单：0 使用 1 编辑 2 删除 3 返回

// ===== 设置自动保存 =====
bool settingsDirty = false;
unsigned long settingsDirtyMs = 0;
const unsigned long SETTINGS_SAVE_DELAY_MS = 3000;  // 参数变化后 3 秒无操作再写入 Flash

// ===== LoRa 模式实时信号强度 =====
float liveRssi = -120.0;
bool  liveRssiInit = false;

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
void loadSettings();
void saveSettings();
void markSettingsDirty();
void savePresets();
void openPresetManager();
void openFreqEditor(int target, uint32_t startKHz);
void applyFreqAndEnterLoRa(uint32_t khz);
void commitFreq();
void drawFreqEditDisplay();
void drawPresetDisplay();
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
  Serial.println("FW " FW_VERSION);

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
  display.setCursor(32, 40);
  display.println(F(FW_VERSION));
  display.display();

  // 配置 ESP32 ADC1 & 工厂 eFuse 校准
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  pinMode(VBAT_ADC_PIN, INPUT);

  // 读取 Flash 保存的校准参数
  loadCalibFactor();
  batteryTick(true);   // 初始化电池读数
  loadSettings();      // 读取上次保存的模式和参数

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

  if (currentMode == MODE_SPECTRUM) {
    applySpectrumBandwidth();   // 频谱模式按步进选择接收带宽
  } else {
    applyLoRaConfig();          // 恢复上次的 LoRa 接收参数
  }
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

  // 设置自动保存：参数变化后延时写入，减少 Flash 擦写
  if (settingsDirty && millis() - settingsDirtyMs > SETTINGS_SAVE_DELAY_MS) {
    saveSettings();
  }

  // 1. 自动关机检测
  if (millis() - lastActivityMs > AUTO_POWER_OFF_MS) {
    powerOff();
  }

  // 2. 取按键事件（由按键任务产生）
  BtnEvent evt = NONE;
  xQueueReceive(btnQueue, &evt, 0);
  if (evt != NONE) {
    lastActivityMs = millis();
  }

  // 双击：返回上一级 / 进入或退出主菜单（频率编辑界面里双击用于数字 -1，不在此处理）
  if (evt == DOUBLE_CLICK && !inFreqUI) {
    if (inPresetUI) {
      if (presetPage == 1) {
        presetPage = 0;              // 操作菜单 -> 列表
      } else {
        inPresetUI = false;          // 列表 -> 主菜单
        inMenu = true;
        menuSelection = 2;
      }
    } else if (inCalibUI) {
      inCalibUI = false;
      inMenu = true;
    } else {
      inMenu = !inMenu;
      if (inMenu) menuSelection = (currentMode == MODE_SPECTRUM) ? 0 : 1;
    }
  }

  // 3. 频率编辑界面
  if (inFreqUI) {
    if (evt == SINGLE_CLICK) {
      if (freqEditPos < 6) {
        freqDigits[freqEditPos] = (freqDigits[freqEditPos] + 1) % 10;
      } else {
        commitFreq();                 // 保存
      }
    } else if (evt == DOUBLE_CLICK) {
      if (freqEditPos < 6) {
        freqDigits[freqEditPos] = (freqDigits[freqEditPos] + 9) % 10;
      } else {
        inFreqUI = false;             // 放弃修改
        if (!inPresetUI) inMenu = true;
      }
    } else if (evt == LONG_PRESS) {
      freqEditPos = (freqEditPos + 1) % 7;   // 下一位，最后一位之后是"保存"
    }
    if (inFreqUI) drawFreqEditDisplay();
    else if (inPresetUI) drawPresetDisplay();
    else if (inMenu) drawMenuDisplay();
  }
  // 4. 预设频率管理界面
  else if (inPresetUI) {
    int total = presetCount + 2;      // 预设 + 新建 + 手动频率
    if (presetPage == 0) {
      if (evt == SINGLE_CLICK) {
        presetSel = (presetSel + 1) % total;
      } else if (evt == LONG_PRESS) {
        uint32_t curKHz = (uint32_t)lroundf(currentFreq * 1000.0f);
        if (presetSel < presetCount) {
          presetPage = 1;             // 打开该预设的操作菜单
          presetActSel = 0;
        } else if (presetSel == presetCount) {
          if (presetCount < MAX_PRESETS) openFreqEditor(-1, curKHz);   // 新建预设
        } else {
          openFreqEditor(-2, curKHz);                                  // 手动频率，直接应用
        }
      }
    } else {
      if (evt == SINGLE_CLICK) {
        presetActSel = (presetActSel + 1) % 4;
      } else if (evt == LONG_PRESS) {
        if (presetActSel == 0) {                       // 使用
          freqIdx = presetSel;
          applyFreqAndEnterLoRa(presetKHz[presetSel]);
        } else if (presetActSel == 1) {                // 编辑
          openFreqEditor(presetSel, presetKHz[presetSel]);
        } else if (presetActSel == 2) {                // 删除（至少保留 1 个）
          if (presetCount > 1) {
            for (int i = presetSel; i < presetCount - 1; i++) presetKHz[i] = presetKHz[i + 1];
            presetCount--;
            if (freqIdx > presetSel) freqIdx--;
            else if (freqIdx == presetSel) freqIdx = 0;
            if (freqIdx >= presetCount) freqIdx = 0;
            if (presetSel >= presetCount) presetSel = presetCount - 1;
            savePresets();
            markSettingsDirty();
          }
          presetPage = 0;
        } else {                                       // 返回
          presetPage = 0;
        }
      }
    }
    if (inPresetUI) drawPresetDisplay();
    else if (inMenu) drawMenuDisplay();
  }
  // 5. 校准界面交互
  else if (inCalibUI) {
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
  // 6. 菜单控制
  else if (inMenu) {
    if (evt == SINGLE_CLICK) {
      menuSelection = (menuSelection + 1) % MENU_ITEMS;
    } else if (evt == LONG_PRESS) {
      if (menuSelection == 0) {
        currentMode = MODE_SPECTRUM;
        applySpectrumBandwidth();
        inMenu = false;
        markSettingsDirty();
      } else if (menuSelection == 1) {
        currentMode = MODE_LORA_ANALYZER;
        applyLoRaConfig();
        inMenu = false;
        markSettingsDirty();
      } else if (menuSelection == 2) {
        openPresetManager();         // 预设频率管理
      } else if (menuSelection == 3) {
        inCalibUI = true; // 进入电池校准模式
        targetCalibVoltage = readBatteryVoltage(); // 以当前测得电压为基准
        if (targetCalibVoltage < 3.3f) targetCalibVoltage = 3.80f;
      } else if (menuSelection == 4) {
        powerOff();
      }
    }
    if (inPresetUI) drawPresetDisplay();
    else if (inCalibUI) drawCalibDisplay();
    else if (inMenu) drawMenuDisplay();
  }
  // 7. 频谱扫描模式
  else if (currentMode == MODE_SPECTRUM) {
    if (evt == SINGLE_CLICK) {
      specCenterFreq += 1.0;
      if (specCenterFreq > 439.0) specCenterFreq = 431.0;
      updateSpanFreqs();
      specInit = false;
      markSettingsDirty();
    } else if (evt == LONG_PRESS) {
      spanIdx = (spanIdx + 1) % SPAN_COUNT;
      updateSpanFreqs();
      applySpectrumBandwidth();
      markSettingsDirty();
    }

    runSpectrumScan();
    drawSpectrumDisplay();
  }
  // 8. LoRa 分析模式
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
        markSettingsDirty();
      } else if (evt == LONG_PRESS) {
        freqIdx = (freqIdx + 1) % presetCount;          // 循环切换预设频率
        currentFreq = presetKHz[freqIdx] / 1000.0f;
        applyLoRaConfig();
        markSettingsDirty();
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

  const char* items[] = {"1. Spectrum Scan", "2. LoRa Receiver", "3. Freq Presets", "4. Calib Battery", "5. Power Off"};

  for (int i = 0; i < MENU_ITEMS; i++) {
    int yPos = 12 + i * 10;
    if (menuSelection == i) {
      display.fillRect(5, yPos, 118, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    display.setCursor(8, yPos + 1);
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
  liveRssiInit = false;   // 切换频率/参数后重新计算实时信号强度
  LoRa.setFrequency(currentFreq * 1E6);
  LoRa.setSignalBandwidth(signalBandwidth);
  LoRa.setSpreadingFactor(COMBO_LIST[comboIdx].sf);
  LoRa.setCodingRate4(COMBO_LIST[comboIdx].cr);
  LoRa.receive();
}

void sampleRSSI() {
  float rawRssi = LoRa.rssi();
  if (!liveRssiInit) {
    liveRssi = rawRssi;
    liveRssiInit = true;
  } else {
    liveRssi = liveRssi * 0.7f + rawRssi * 0.3f;   // 轻度平滑
  }
  rssiHistory[rssiWrIdx] = rawRssi;
  rssiWrIdx = (rssiWrIdx + 1) % RSSI_HIST_LEN;
}

void drawMainDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("%.3fM", currentFreq);

  // 实时信号强度（顶部中间）
  display.setCursor(54, 0);
  display.printf("%.0fdBm", liveRssi);

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
  if (settingsDirty) saveSettings();
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

// ================= 设置保存 / 读取 =================
void markSettingsDirty() {
  settingsDirty = true;
  settingsDirtyMs = millis();
}

void saveSettings() {
  prefs.begin("settings", false);
  prefs.putUChar("mode", (uint8_t)currentMode);
  prefs.putUChar("span", (uint8_t)spanIdx);
  prefs.putFloat("cen", specCenterFreq);
  prefs.putUInt("fkhz", (uint32_t)lroundf(currentFreq * 1000.0f));
  prefs.putUChar("combo", (uint8_t)comboIdx);
  prefs.putUChar("fidx", (uint8_t)freqIdx);
  prefs.end();
  settingsDirty = false;
  Serial.printf("[NVS] Settings saved: mode=%d span=%d cen=%.1f freq=%.3f combo=%d\n",
                (int)currentMode, spanIdx, specCenterFreq, currentFreq, comboIdx);
}

void loadSettings() {
  prefs.begin("settings", false);   // 读写模式打开，首次运行会自动创建命名空间
  uint8_t mode  = prefs.getUChar("mode", (uint8_t)MODE_SPECTRUM);
  uint8_t span  = prefs.getUChar("span", 0);
  float   cen   = prefs.getFloat("cen", 435.0f);
  uint32_t fkhz = prefs.getUInt("fkhz", DEFAULT_PRESETS_KHZ[0]);
  uint8_t combo = prefs.getUChar("combo", 0);
  uint8_t fidx  = prefs.getUChar("fidx", 0);

  // 预设频率表
  uint8_t pc = prefs.getUChar("pcount", 0);
  bool presetOk = false;
  if (pc >= 1 && pc <= MAX_PRESETS && prefs.getBytesLength("presets") == sizeof(presetKHz)) {
    uint32_t tmp[MAX_PRESETS];
    prefs.getBytes("presets", tmp, sizeof(tmp));
    presetOk = true;
    for (int i = 0; i < pc; i++) {
      if (tmp[i] < FREQ_MIN_KHZ || tmp[i] > FREQ_MAX_KHZ) { presetOk = false; break; }
    }
    if (presetOk) {
      memcpy(presetKHz, tmp, sizeof(presetKHz));
      presetCount = pc;
    }
  }
  prefs.end();

  if (!presetOk) {   // 首次运行或数据异常：使用默认预设
    memset(presetKHz, 0, sizeof(presetKHz));
    for (int i = 0; i < DEFAULT_PRESET_COUNT; i++) presetKHz[i] = DEFAULT_PRESETS_KHZ[i];
    presetCount = DEFAULT_PRESET_COUNT;
  }

  // 合法性检查，避免 Flash 中的异常值导致越界
  currentMode  = (mode == (uint8_t)MODE_LORA_ANALYZER) ? MODE_LORA_ANALYZER : MODE_SPECTRUM;
  spanIdx      = (span < SPAN_COUNT) ? span : 0;
  specCenterFreq = (cen >= 431.0f && cen <= 439.0f) ? cen : 435.0f;
  if (fkhz < FREQ_MIN_KHZ || fkhz > FREQ_MAX_KHZ) fkhz = presetKHz[0];
  currentFreq  = fkhz / 1000.0f;
  comboIdx     = (combo < COMBO_COUNT) ? combo : 0;
  freqIdx      = (fidx < presetCount) ? fidx : 0;

  Serial.printf("[NVS] Settings loaded: mode=%d span=%d cen=%.1f freq=%.3f combo=%d presets=%d\n",
                (int)currentMode, spanIdx, specCenterFreq, currentFreq, comboIdx, presetCount);
}

void savePresets() {
  prefs.begin("settings", false);
  prefs.putUChar("pcount", (uint8_t)presetCount);
  prefs.putBytes("presets", presetKHz, sizeof(presetKHz));
  prefs.end();
  Serial.printf("[NVS] Presets saved, count=%d\n", presetCount);
}

// ================= 手动设置频率 =================
void openPresetManager() {
  uint32_t cur = (uint32_t)lroundf(currentFreq * 1000.0f);
  presetSel = 0;
  for (int i = 0; i < presetCount; i++) {
    if (presetKHz[i] == cur) { presetSel = i; break; }
  }
  presetPage = 0;
  presetActSel = 0;
  inPresetUI = true;
  inMenu = false;
}

void openFreqEditor(int target, uint32_t startKHz) {
  freqEditTarget = target;
  freqDigits[0] = (startKHz / 100000) % 10;
  freqDigits[1] = (startKHz / 10000) % 10;
  freqDigits[2] = (startKHz / 1000) % 10;
  freqDigits[3] = (startKHz / 100) % 10;
  freqDigits[4] = (startKHz / 10) % 10;
  freqDigits[5] = startKHz % 10;
  freqEditPos = 0;
  inFreqUI = true;
  inMenu = false;
}

static uint32_t freqDigitsToKHz() {
  return (uint32_t)freqDigits[0] * 100000UL + (uint32_t)freqDigits[1] * 10000UL +
         (uint32_t)freqDigits[2] * 1000UL   + (uint32_t)freqDigits[3] * 100UL +
         (uint32_t)freqDigits[4] * 10UL     + (uint32_t)freqDigits[5];
}

// 把频率应用到接收机并进入 LoRa 接收界面
void applyFreqAndEnterLoRa(uint32_t khz) {
  if (khz < FREQ_MIN_KHZ) khz = FREQ_MIN_KHZ;
  if (khz > FREQ_MAX_KHZ) khz = FREQ_MAX_KHZ;

  currentFreq = khz / 1000.0f;
  currentMode = MODE_LORA_ANALYZER;
  isLocked = false;
  decodedPayload = "";
  for (int i = 0; i < RSSI_HIST_LEN; i++) rssiHistory[i] = -120.0;
  applyLoRaConfig();
  markSettingsDirty();

  inFreqUI = false;
  inPresetUI = false;
  inMenu = false;
}

void commitFreq() {
  uint32_t khz = freqDigitsToKHz();
  if (khz < FREQ_MIN_KHZ) khz = FREQ_MIN_KHZ;
  if (khz > FREQ_MAX_KHZ) khz = FREQ_MAX_KHZ;

  if (freqEditTarget == -1) {                       // 新建预设
    if (presetCount < MAX_PRESETS) {
      presetKHz[presetCount] = khz;
      presetSel = presetCount;
      presetCount++;
      savePresets();
    }
    presetPage = 0;
    inFreqUI = false;
  } else if (freqEditTarget >= 0 && freqEditTarget < presetCount) {   // 修改预设
    presetKHz[freqEditTarget] = khz;
    savePresets();
    presetPage = 0;
    inFreqUI = false;
  } else {                                          // 手动频率：直接应用
    applyFreqAndEnterLoRa(khz);
  }
}

void drawFreqEditDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  display.setCursor(10, 0);
  display.print(freqEditTarget >= 0 ? "= EDIT PRESET =" : (freqEditTarget == -1 ? "= NEW PRESET =" : "= SET LORA FREQ ="));
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  // 大号频率显示  XXX.XXX
  char buf[10];
  snprintf(buf, sizeof(buf), "%d%d%d.%d%d%d",
           freqDigits[0], freqDigits[1], freqDigits[2],
           freqDigits[3], freqDigits[4], freqDigits[5]);
  display.setTextSize(2);
  display.setCursor(8, 15);
  display.print(buf);
  display.setTextSize(1);
  display.setCursor(96, 22);
  display.print("MHz");

  // 当前编辑位的下划线（字符位置 0,1,2,4,5,6，每字符宽 12px）
  if (freqEditPos < 6) {
    static const uint8_t charPos[6] = {0, 1, 2, 4, 5, 6};
    display.fillRect(8 + charPos[freqEditPos] * 12, 32, 11, 2, SSD1306_WHITE);
  }

  // 范围提示
  uint32_t khz = freqDigitsToKHz();
  display.setCursor(0, 37);
  if (khz < FREQ_MIN_KHZ || khz > FREQ_MAX_KHZ) display.print("OUT OF RANGE>clamp");
  else display.print("Range 410-525MHz");

  // 保存行
  if (freqEditPos == 6) {
    display.fillRect(0, 46, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(2, 47);
    display.print(freqEditTarget >= -1 ? "> SAVE PRESET <" : "> SAVE & APPLY <");
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, 56);
    display.print("Click:Save 2xClk:Esc");
  } else {
    display.setCursor(0, 47);
    display.print("Hold: next digit");
    display.setCursor(0, 56);
    display.print("Click:+1  2xClk:-1");
  }

  display.display();
}

// ================= 预设频率管理界面 =================
void drawPresetDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  if (presetPage == 0) {
    display.setCursor(16, 0);
    display.print("= FREQ PRESETS =");
    display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

    int total = presetCount + 2;          // 预设 + 新建 + 手动频率
    int first = presetSel - 2;            // 选中项尽量居中，最多显示 5 行
    if (first > total - 5) first = total - 5;
    if (first < 0) first = 0;
    uint32_t curKHz = (uint32_t)lroundf(currentFreq * 1000.0f);

    for (int r = 0; r < 5; r++) {
      int idx = first + r;
      if (idx >= total) break;
      int y = 12 + r * 10;
      if (idx == presetSel) {
        display.fillRect(0, y, 122, 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      } else {
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      }
      display.setCursor(4, y + 1);
      if (idx < presetCount) {
        display.printf("%d. %.3f MHz", idx + 1, presetKHz[idx] / 1000.0f);
        if (presetKHz[idx] == curKHz) {     // 当前正在使用的频率
          display.setCursor(110, y + 1);
          display.print("*");
        }
      } else if (idx == presetCount) {
        display.print(presetCount < MAX_PRESETS ? "+ New preset" : "+ New (full)");
      } else {
        display.print("~ Manual freq");
      }
    }
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

    // 右侧滚动条
    if (total > 5) {
      int thumbH = max(4, 50 * 5 / total);
      int thumbY = 12 + (50 - thumbH) * first / (total - 5);
      display.drawFastVLine(126, 12, 50, SSD1306_WHITE);
      display.fillRect(125, thumbY, 3, thumbH, SSD1306_WHITE);
    }
  } else {
    char title[24];
    snprintf(title, sizeof(title), "= %.3f MHz =", presetKHz[presetSel] / 1000.0f);
    display.setCursor(10, 0);
    display.print(title);
    display.drawFastHLine(0, 10, 128, SSD1306_WHITE);

    const char* acts[] = {"1. Use this freq", "2. Edit freq", "3. Delete", "4. Back"};
    for (int i = 0; i < 4; i++) {
      int y = 12 + i * 10;
      if (i == presetActSel) {
        display.fillRect(5, y, 118, 10, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      } else {
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
      }
      display.setCursor(8, y + 1);
      if (i == 2 && presetCount <= 1) display.print("3. Delete (last)");
      else display.print(acts[i]);
    }
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.setCursor(0, 56);
    display.print("Click:Next Hold:OK");
  }

  display.display();
}
