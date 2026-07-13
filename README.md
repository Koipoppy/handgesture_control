# Hand Gesture Control — 手势控制舵机

使用电脑 USB 摄像头实时识别手势，通过串口控制 Arduino Mega 2560 驱动 DS3225 舵机。提供两个版本：**双位控制**（开/关二值）和**连续控制**（无级调节）。

## 系统架构

```
USB Camera → Python(OpenCV + MediaPipe) → Serial(COM) → Arduino Mega2560 → DS3225 Servo
```

| 版本 | 手势 | 串口命令 | 舵机动作 |
|------|------|----------|----------|
| 双位控制 | 张开手掌 | `O` | 2200us（全开） |
| 双位控制 | 握拳 | `C` | 1100us（全关） |
| 连续控制 | 手的开合程度 | `1500\n`（脉宽值） | 1100~2200us（无级调节） |

## 硬件清单

| 硬件 | 说明 |
|------|------|
| Arduino Mega 2560 | 主控板，通过 USB 连接电脑 |
| DS3225 数字舵机 | 270° 大角度舵机，驱动夹爪 |
| USB 摄像头 | 电脑自带或外接均可 |

## 接线说明

| 舵机线 | 连接到 |
|--------|--------|
| 信号线（橙/白） | Arduino D9 |
| 电源线（红） | 外部供电 5V~7.4V（根据舵机规格） |
| 地线（棕/黑） | Arduino GND（共地） |

> **注意**：DS3225 舵机电流较大，建议使用外部电源供电，不要直接从 Arduino 5V 引脚取电。

## 项目结构

```
handgesture_control/
├── 双位控制/
│   ├── main/
│   │   └── main.ino              # Arduino 固件（双位）
│   ├── main.py                   # Python 上位机（双位）
│   └── requirements.txt
├── 连续控制/
│   ├── main/
│   │   └── main.ino              # Arduino 固件（连续）
│   ├── main.py                   # Python 上位机（连续）
│   └── requirements.txt
├── .gitignore
└── README.md
```

## 安装

### 1. Python 依赖

两个版本的依赖相同，任选一个目录安装：

```bash
pip install -r 双位控制/requirements.txt
```

依赖包：

| 包名 | 版本要求 | 说明 |
|------|----------|------|
| `opencv-python` | ==4.10.0.84 | 摄像头画面采集与显示 |
| `mediapipe` | 0.10.35 | 手部 21 关键点检测（Tasks API） |
| `pyserial` | 3.5 | 串口通信 |
| `numpy` | <2 | mediapipe 兼容性要求 |

> **重要**：mediapipe 0.10.35 需要 `numpy<2`，如果已安装 numpy 2.x，请降级：`pip install "numpy<2"`

### 2. 下载 MediaPipe 模型文件

由于模型文件较大（约 7.5MB），不包含在仓库中，需手动下载：

```bash
# 下载到纯英文路径（MediaPipe C++ 底层不支持路径中的中文字符）
curl.exe -L -o C:\mediapipe_models\hand_landmarker.task "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
```

或手动下载：[hand_landmarker.task](https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task)

下载后放置到 `C:\mediapipe_models\` 目录下（或其他纯英文路径），并在 `main.py` 中修改 `MODEL_PATH`。

### 3. Arduino 固件

根据使用的版本选择对应的固件：

| 版本 | 固件文件 |
|------|----------|
| 双位控制 | `双位控制/main/main.ino` |
| 连续控制 | `连续控制/main/main.ino` |

1. 用 Arduino IDE 打开对应固件
2. 选择开发板：**工具 → 开发板 → Arduino Mega or Mega 2560**
3. 选择处理器：**ATmega2560 (Mega 2560)**
4. 选择正确的 COM 端口
5. 点击上传

## 配置

编辑对应版本 `main.py` 顶部的配置区：

```python
# --- 串口配置 ---
SERIAL_PORT = "COM9"        # ← 改成你的实际串口号
BAUD_RATE = 115200           # 波特率

# --- 摄像头配置 ---
CAMERA_INDEX = 0             # 摄像头索引

# --- 模型文件路径 ---
MODEL_PATH = r"C:\mediapipe_models\hand_landmarker.task"  # ← 模型文件路径
```

连续控制版本还有额外参数：

```python
# --- 舵机脉宽范围 ---
PULSE_MIN = 1100            # 最小脉宽（握拳时）
PULSE_MAX = 2200            # 最大脉宽（张开时）

# --- 开合度归一化参数 ---
RATIO_MIN = 0.8             # 伸展比下限（对应握拳）
RATIO_MAX = 1.3             # 伸展比上限（对应张开）

# --- 平滑参数 ---
SMOOTH_WINDOW = 5           # 滑动平均窗口大小

# --- 串口发送间隔 ---
SEND_INTERVAL = 0.05        # 最小发送间隔（秒）
```

查看串口号方法：
- Windows：设备管理器 → 端口（COM 和 LPT）
- Linux：`ls /dev/ttyUSB*` 或 `ls /dev/ttyACM*`

## 运行

### 双位控制版本

```bash
cd 双位控制
python main.py
```

- 张开手掌 → 夹爪全开（2200us）
- 握拳 → 夹爪全关（1100us）
- 连续 5 帧确认才切换，防误触

### 连续控制版本

```bash
cd 连续控制
python main.py
```

- 手的开合程度直接映射到舵机位置
- 画面显示开合度百分比、脉宽值和进度条
- 滑动平均平滑，无级调节

按 **Q** 键退出。

## 工作原理

### 双位控制

使用 MediaPipe 检测 21 个关键点，距离法判断手指伸直/弯曲，5帧投票防抖，仅在状态变化时发送命令。

### 连续控制

计算 5 根手指的"伸展比"（指尖到手腕距离 / 关节到手腕距离），取平均后归一化到 [0, 1]，映射到脉宽 [1100, 2200]us。滑动平均平滑，50ms 发送间隔限流。

## 命令协议

### 双位控制

| 串口命令 | 含义 | 舵机动作 |
|----------|------|----------|
| `O` | Open（张开手掌） | `servo.writeMicroseconds(2200)` |
| `C` | Close（握拳） | `servo.writeMicroseconds(1100)` |

### 连续控制

| 串口格式 | 含义 | 示例 |
|----------|------|------|
| `脉宽值\n` | ASCII 数字 + 换行符 | `1500\n` → 舵机到 1500us |

有效范围：1100~2200，超出自动限幅。

## 常见问题

### Q: 运行时报 `module 'mediapipe' has no attribute 'solutions'`

A: mediapipe 0.10.35 已移除旧版 `solutions` API，本项目已改用新版 Tasks API，请确保使用最新代码。

### Q: 运行时报 `Unable to open file` 或 numpy 相关错误

A:
1. numpy 版本需 <2：`pip install "numpy<2"`
2. opencv-python 需降级到 4.10：`pip install opencv-python==4.10.0.84`
3. 模型文件路径不能包含中文字符

### Q: 串口打开失败

A: 检查串口号是否正确、Arduino IDE 串口监视器是否已关闭（会占用端口）。

### Q: 连续控制时舵机抖动

A: 增大 `SMOOTH_WINDOW` 值（如改为 8 或 10），增加平滑度。

### Q: 连续控制范围不匹配

A: 调整 `RATIO_MIN` 和 `RATIO_MAX` 参数。每个人手型不同，可在运行时观察画面中的开合度百分比是否与实际手势匹配。

### Q: 舵机不动

A: 确认外部供电已接通，信号线接在 D9，Arduino 与舵机需共地。确保 Arduino 和 Python 使用的是同一版本的协议（双位 vs 连续）。

## 技术栈

- Python 3.11
- OpenCV 4.10.0.84
- MediaPipe 0.10.35（Tasks API）
- pyserial 3.5
- Arduino C++（Servo 库）
- 硬件：Arduino Mega 2560 + DS3225 舵机
