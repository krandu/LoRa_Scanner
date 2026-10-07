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
float currentRealtimeRssi = -120.0; // 实时 RSSI 采样值

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
  Serial.println("\n--- Heltec WiFi LoRa 32 V2 (Full Functional) ---");

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

  floatToFreqDigits(); // 初始化数位数组
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

void floatToFreqDigits() {
  long khz = round(currentFreq * 1000.0f);
  freqDigits[0] = (khz / 100000) % 10;
  freqDigits[1] = (khz / 10000) % 10;
  freqDigits[2] = (khz / 1000) % 10;
  freqDigits[3] = (khz / 100) % 10;
  freqDigits[4] = (khz / 10) % 10;
  freqDigits[5] = khz % 10;
}

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
  floatToFreqDigits();
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

  // 1. 最高优先级：非数字编辑模式下双击触发主菜单 (含瀑布图/LoRa接收模式)
  if (evt == DOUBLE_CLICK && !isFreqEditing) {
    if (inCalibUI) {
      inCalibUI = false;
      inMenu = true;
    } else {
      inMenu = !inMenu;
      if (inMenu) menuSelection = (currentMode == MODE_SPECTRUM) ? 0 : 1;
    }
  }

  // 2. 电池校准 UI
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
  // 3. 主菜单
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
  // 4. 频谱扫描/瀑布图模式
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
  // 5. LoRa 分析解码接收模式
  else {
    if