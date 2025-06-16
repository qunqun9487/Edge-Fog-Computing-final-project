# 🤖 Hand Gesture Recognition + Robot Drawing System (Non-ROS Version)

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
