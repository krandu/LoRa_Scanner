# LoRa Scanner / Decoder — Heltec WiFi LoRa 32 V2

基于 OpenWebRX 瀑布图实际观测结果，针对 **Meshtastic** 信号优化的扫描解码工具。

---

## 背景：瀑布图分析结论

| 截图 | 频率 | 特征 | 结论 |
|---|---|---|---|
| 图1 | 435~436 MHz | 窄带弱亮斑 | 弱 LoRa 包 |
| 图2 | 479 MHz | 双平行矩形块（强） | 标准 Meshtastic 参考信号 |
| 图3 | 438.125 MHz | 单实心矩形块 125 kHz | **确认 LoRa / Meshtastic** ✅ |

LoRa 在瀑布图上的标志：**固定带宽的实心水平矩形块**（chirp 扫频速度远快于瀑布图刷新率，积分后呈矩形）。

---

## 硬件

| 组件 | 说明 |
|---|---|
| Heltec WiFi LoRa 32 **V2** | ESP32 + SX1276 + SSD1306 OLED 128×64 |
| 天线 | 433/470 MHz 弹簧天线或 SMA 外接 |
| 按键 | 板载 PRG（GPIO0），无需额外接线 |

---

## 依赖

### Board Manager
```
https://resource.heltec.cn/download/package_heltec_esp32_index.json
```
选择：**Heltec WiFi LoRa 32(V2)**

### Library Manager
```
Heltec ESP32 Dev-Boards   ≥ 1.1.0
LoRa (by Sandeep Mistry)  ≥ 0.8.0
```

---

## 按键操作

| 操作 | 判定时长 | 效果 |
|---|---|---|
| 短按 | 50 ms ~ 1.5 s | 解锁 → 手动切换下一个 SF/CR 组合 |
| 长按 | ≥ 1.5 s | 循环切换频率，重置扫描 |

---

## 频率表（长按循环切换）

| 序号 | 频率 | 备注 |
|---|---|---|
| 0 | 435.000 MHz | 弱信号区起点 |
| 1 | 435.500 MHz | 弱信号中心 |
| 2 | 436.000 MHz | 弱信号右侧 |
| **3** | **438.125 MHz** | **★ 默认，强 Meshtastic 信号** |
| 4 | 433.000 MHz | ISM 常用 |
| 5 | 434.000 MHz | ISM 常用 |

---

## 自动扫描 SF/CR 组合（共 16 种，Meshtastic 优先）

| 优先级 | SF | CR | Meshtastic 名称 |
|---|---|---|---|
| ★1 | SF11 | 4/8 | LongSlow |
| ★2 | SF12 | 4/8 | VeryLongSlow |
| ★3 | SF11 | 4/5 | LongFast |
| ★4 | SF10 | 4/5 | MediumSlow |
| ★5 | SF10 | 4/8 | — |
| ★6 | SF9  | 4/5 | MediumFast |
| ★7 | SF9  | 4/8 | — |
| 8  | SF7  | 4/5 | ShortFast |
| … | … | … | 其余补充组合 |

每个组合自动停留 **800 ms**，解码成功后**自动锁定**。

---

## OLED 显示布局

```
┌────────────────────────────────┐
│ 438.125M          SF11 CR4/8  │  ← 频率 / 当前 SF·CR
│ SCAN 1/16 BW:125k             │  ← 扫描中
│ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  │
│ ▓▓▓░░▓▓▓░░░▓▓▓▓░░▓▓▓░░░░▓▓▓ │
│ ▓▓░░░░░▓▓░░░░▓▓▓░░░▓▓▓░░░▓▓  │  ← RSSI 瀑布热力图
│ ░░░░░░░░░░░░░░░░░░░▓▓▓▓▓░░░  │
│ ░░░░░░░░░░░░░░░░░░░░░▓▓▓▓▓▓  │
│ >48 65 00 FF AB 12 DE AD...   │  ← 锁定时显示 Payload
└────────────────────────────────┘

锁定后：
│ *LOCK* R:-87 S:6              │
```

---

## 串口输出示例（115200 baud）

```
========================================
 LoRa Scanner  —  Heltec WiFi LoRa 32 V2
 长按 PRG : 切换频率
 短按 PRG : 切换 SF/CR 或解锁
========================================
[CFG] 438.125 MHz | SF11 | CR4/8 | BW:125kHz | SYNC:0x2B
[CFG] 438.125 MHz | SF12 | CR4/8 | BW:125kHz | SYNC:0x2B
...
[RX] 438.125 MHz | SF11 CR4/8 | RSSI:-87 SNR:6 | len:24
     !LH 00 00 00 00 48 65 6C 6C 6F ...
[BTN] 长按 → 435.000 MHz
[BTN] 短按 → SF12 CR4/8
```

---

## 关键参数说明

| 参数 | 值 | 说明 |
|---|---|---|
| 同步字 | `0x2B` | Meshtastic 私有；LoRaWAN 用 `0x34`；通用私有用 `0x12` |
| 带宽 | 125 kHz | 与瀑布图实测吻合 |
| CRC | 关闭 | 提高捕获率；确认目标后可改为 `LoRa.enableCrc()` |
| Payload 格式 | 可打印字符直接显示，其余转 HEX | 兼容 Meshtastic protobuf 二进制帧 |

---

## 常见问题

**Q: 收不到包？**
- 确认天线已连接（SX1276 无天线发射可能损坏芯片）
- 尝试长按切换到 438.125 MHz + 等待 SF11/SF12 组合
- 用 SDR 先确认目标频率和带宽

**Q: 如何解码 Meshtastic 内容？**
- 本工具捕获原始 LoRa 帧，Meshtastic 使用 **Protobuf** 编码
- 可将 HEX 输出导入 [Meshtastic Packet Decoder](https://meshtastic.org/) 进一步解析

**Q: 如何添加更多频率？**
- 在 `FREQ_LIST[]` 数组中添加，`FREQ_COUNT` 会自动计算

---

## 文件结构

```
LoRa_Scanner/
├── LoRa_Scanner.ino   主程序
└── README.md          本文档
```
