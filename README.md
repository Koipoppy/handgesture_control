# Hand Gesture Control — 手势控制舵机

使用电脑 USB 摄像头实时识别手势（张开手掌 / 握拳），通过串口控制 Arduino Mega 2560 驱动 DS3225 舵机切换夹爪开合状态。

## 系统架构

```
USB Camera → Python(OpenCV + MediaPipe) → Serial(COM) → Arduino Mega2560 → DS3225 Servo
```

- 张开手（Open Palm）→ 发送 `O` → 舵机运动到 2200us（夹爪张开）
- 握拳（Closed Fist）→ 发送 `C` → 舵机运动到 1100us（夹爪关闭）

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
├── arduino/
│   └── main.ino              # Arduino Mega 2560 固件
├── python/
│   ├── main.py               # Python 上位机主程序
│   ├── requirements.txt       # Python 依赖清单
│   └── hand_landmarker.task  # MediaPipe 手部检测模型（需自行下载）
├── .gitignore
└── README.md
```

## 安装

### 1. Python 依赖

```bash
pip install -r python/requirements.txt
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

下载后放置到 `C:\mediapipe_models\` 目录下（或其他纯英文路径），并在 `python/main.py` 中修改 `MODEL_PATH`。

### 3. Arduino 固件

1. 用 Arduino IDE 打开 `arduino/main.ino`
2. 选择开发板：**工具 → 开发板 → Arduino Mega or Mega 2560**
3. 选择处理器：**ATmega2560 (Mega 2560)**
4. 选择正确的 COM 端口
5. 点击上传

## 配置

编辑 `python/main.py` 顶部的配置区：

```python
# --- 串口配置 ---
SERIAL_PORT = "COM9"        # ← 改成你的实际串口号
BAUD_RATE = 115200           # 波特率

# --- 防抖配置 ---
DEBOUNCE_FRAMES = 5          # 防抖帧数阈值

# --- 摄像头配置 ---
CAMERA_INDEX = 0             # 摄像头索引

# --- 模型文件路径 ---
MODEL_PATH = r"C:\mediapipe_models\hand_landmarker.task"  # ← 模型文件路径
```

查看串口号方法：
- Windows：设备管理器 → 端口（COM 和 LPT）
- Linux：`ls /dev/ttyUSB*` 或 `ls /dev/ttyACM*`

## 运行

```bash
cd python
python main.py
```

## 使用方法

1. 确保 Arduino 已上传固件并连接电脑
2. 运行 Python 程序，摄像头画面会弹出
3. 对着摄像头做手势：
   - **张开手掌**（五指伸直）→ 夹爪张开（舵机 2200us）
   - **握拳**（五指弯曲）→ 夹爪关闭（舵机 1100us）
4. 画面左上角显示当前状态和识别结果
5. 按 **Q** 键退出

## 工作原理

### 手势检测

使用 **MediaPipe Tasks API** 检测手部 21 个关键点，通过几何规则判断手势：

- **距离法**：比较指尖(TIP)到手腕的距离与近端关节(PIP)到手腕的距离
  - 伸直时 TIP 离手腕更远
  - 弯曲时 TIP 被拉回，距离反而比 PIP 近
- 此方法对手部旋转不敏感，可以在任意角度挥手

### 防抖机制

使用 `collections.deque` 滑动窗口（5帧）+ `Counter` 投票取众数：
- 连续 5 帧识别为同一手势才确认状态改变
- 避免单帧误识别导致舵机不停运动

### 状态管理

- 维护 `current_state` 变量（"OPEN" / "CLOSE"）
- 仅在状态变化时发送串口命令，不重复发送
- 未检测到手时不发送任何命令，保持当前状态

## 命令协议

| 串口命令 | 含义 | 舵机动作 |
|----------|------|----------|
| `O` | Open（张开手掌） | `servo.writeMicroseconds(2200)` |
| `C` | Close（握拳） | `servo.writeMicroseconds(1100)` |

## 代码结构

Python 上位机采用函数封装，结构清晰：

| 函数/类 | 说明 |
|---------|------|
| `calculate_distance()` | 计算两个 landmark 之间的欧氏距离 |
| `is_finger_extended()` | 判断非拇指手指是否伸直（距离法） |
| `is_thumb_extended()` | 判断拇指是否伸直（横向距离法） |
| `detect_hand_state()` | 手势识别核心：5指全伸=OPEN，全弯=CLOSE |
| `send_command()` | 串口发送封装 |
| `GestureDebouncer` | 防抖器类：滑动窗口投票 |
| `draw_hand_landmarks()` | 在画面上绘制手部关键点和骨架 |
| `draw_ui()` | 绘制 UI 信息（状态、手势、FPS） |
| `main()` | 主程序入口 |

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

### Q: 识别不灵敏

A: 确保光线充足，手部完整出现在画面中，保持 5 帧以上才能触发动作（防抖设计）。

### Q: 舵机不动

A: 确认外部供电已接通，信号线接在 D9，Arduino 与舵机需共地。

## 技术栈

- Python 3.11
- OpenCV 4.10.0.84
- MediaPipe 0.10.35（Tasks API）
- pyserial 3.5
- Arduino C++（Servo 库）
- 硬件：Arduino Mega 2560 + DS3225 舵机
