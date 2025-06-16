ecognition + Robot Drawing System (Non-ROS Version)

This project combines a **TFLite-based hand gesture recognition model** with **TurtleBot3 robot movement control logic**, enabling the robot to "draw" corresponding patterns or letters based on real-time recognized hand gestures from a camera.

## 📁 Project Structure

| File         | Description                                                                 |
|--------------|-----------------------------------------------------------------------------|
| `gesture.py` | Recognizes hand gestures in real-time using a TFLite model and prints results. |
| `draw.py`    | Executes corresponding drawing routines based on gesture strings via movement simulation. |

## 🔧 Requirements

- Python 3.8+
- TFLite model (quantized to int8 format, YOLOv5-style)
- Python packages:
  - `tensorflow`
  - `opencv-python`
  - `numpy`

## 🚀 How to Run

### 1️⃣ Run Gesture Recognition

```bash
python3 gesture.py
```

> 📌 Make sure to update the model path inside the script. It prints recognized labels such as `"W"`, `"E"`, `"LOVE"`...

### 2️⃣ Run Drawing Script (for simulation or real robot)

```bash
python3 draw.py
```

> 📌 The current logic assumes ROS-style topics. You can adapt it to non-ROS simulation or direct function calls.

## ✋ Supported Gestures

| Label | Action Description               |
|-------|----------------------------------|
| `W`   | Draws the capital letter W       |
| `E`   | Draws a lowercase e (line + arc) |
| `LOVE`| Draws a heart shape              |
| `U`   | Draws the "UJesik" combined path |

## 🧠 Model Notes

- The model must be **int8 quantized TFLite format**.
- Update the model path in `gesture.py`:

```python
MODEL_PATH = '/your/path/to/best-int8.tflite'
```

## 🔄 Future Extensions

- Integrate gesture recognition as a callable API
- Add streaming or network interface to control remote robots
- Extend `draw.py` to support GUI simulation or real robot path planning



# Edge-Fog-Computing-final-project
# 🤖 手勢辨識 + 機器人寫字系統（非 ROS 節點版）

本專案結合 **TFLite 手勢辨識模型** 與 **TurtleBot3 機器人運動控制邏輯**，實現從攝影機即時辨識手勢後，讓機器人根據不同手勢畫出對應的圖案或字母（如 W、E、LOVE、UJesik 等）。

## 📁 專案結構

| 檔案            | 說明                                           |
|-----------------|------------------------------------------------|
| `gesture.py`    | 使用 TFLite 模型辨識即時攝影機影像中的手勢，並印出辨識結果（可透過變數傳遞給 draw 模組）。 |
| `draw.py`       | 根據手勢字串執行對應的繪圖函式，透過程式模擬機器人走出圖案。 |

## 🔧 系統需求

- Python 3.8+
- 已轉為 int8 的 TFLite 模型（YOLO v5 類型）
- 相容套件：
  - `tensorflow`
  - `opencv-python`
  - `numpy`

## 🚀 執行方式

### 1️⃣ 執行手勢辨識（從攝影機辨識手語）

```bash
python3 gesture.py
```

> 📌 模型會載入 `best-int8.tflite`，記得確認模型路徑正確，會印出辨識結果（如 "W", "E", "LOVE"...）

### 2️⃣ 執行畫圖動作模擬（模擬或實體機器人控制）

```bash
python3 draw.py
```

> 📌 預設為 ROS 控制格式，如你不使用 ROS，可改寫為純 Python 動作模擬或使用模擬變數呼叫繪圖函數。

## 🖐 支援手勢類型與對應動作

| 手勢代碼 | 動作說明                      |
|----------|-------------------------------|
| `W`      | 畫出大寫字母 W                |
| `E`      | 畫出小寫 e（橫線與圓弧）       |
| `LOVE`   | 畫出愛心圖案                   |
| `U`      | 畫出「UJesik」字樣組合動作     |

## 🧠 模型注意事項

- 模型必須為 **int8 量化格式 (`.tflite`)**。
- 修改 `gesture.py` 中的模型路徑以符合你電腦上的位置：

```python
MODEL_PATH = '/your/path/to/best-int8.tflite'
```

## 🔄 延伸用途

- 可將 `gesture.py` 模組作為辨識 API，與其他動作程式整合。
- 可加入串流架構，實現跨平台影像辨識 → 指令傳送 → 遠端寫字。
- 可將 `draw.py` 擴展為支援虛擬模擬器（如 Pygame）或實體機器人底層。
