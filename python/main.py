"""
======================================================================
项目名称：手势控制 DS3225 舵机 — Python 上位机
功能说明：
    使用电脑 USB 摄像头实时识别手势（张开手掌 / 握拳），
    通过串口发送命令给 Arduino Mega 2560，控制 DS3225 舵机。

    张开手（Open Palm）→ 发送 'O' → 舵机运动到 2200us（夹爪张开）
    握拳（Closed Fist） → 发送 'C' → 舵机运动到 1100us（夹爪关闭）

技术栈：
    - OpenCV：摄像头画面采集与显示
    - MediaPipe Tasks API：手部 21 关键点检测（新版 API）
    - pyserial：串口通信

运行方式：
    python main.py
    按 Q 键退出
======================================================================
"""

import cv2
import math
import serial
import mediapipe as mp
import numpy as np
from collections import deque, Counter
import time
import sys
import os

# ======================================================================
# 配置区（可修改参数集中在此处）
# ======================================================================

# --- 串口配置 ---
SERIAL_PORT = "COM9"        # 串口号，根据实际连接修改（如 COM4, /dev/ttyUSB0 等）
BAUD_RATE = 115200           # 波特率，需与 Arduino 端保持一致

# --- 防抖配置 ---
DEBOUNCE_FRAMES = 5          # 防抖帧数阈值：连续识别同一手势 N 帧才确认状态改变

# --- 摄像头配置 ---
CAMERA_INDEX = 0             # 摄像头索引（0=默认摄像头，1=第二个摄像头）

# --- MediaPipe 配置 ---
MAX_NUM_HANDS = 1            # 最多检测的手数
MIN_DETECTION_CONFIDENCE = 0.6   # 手掌检测置信度阈值
MIN_TRACKING_CONFIDENCE = 0.6    # 关键点跟踪置信度阈值

# --- 模型文件路径 ---
# 注意：MediaPipe C++ 底层不支持路径中包含中文字符，
# 因此将模型文件放在纯英文路径下
MODEL_PATH = r"C:\mediapipe_models\hand_landmarker.task"

# --- 手部关键点连接关系 ---
# MediaPipe 21 个关键点的骨架连线定义，用于在画面上绘制手部骨架
HAND_CONNECTIONS = [
    # 拇指
    (0, 1), (1, 2), (2, 3), (3, 4),
    # 食指
    (0, 5), (5, 6), (6, 7), (7, 8),
    # 中指
    (5, 9), (9, 10), (10, 11), (11, 12),
    # 无名指
    (9, 13), (13, 14), (14, 15), (15, 16),
    # 小指
    (13, 17), (17, 18), (18, 19), (19, 20),
    # 手掌底部
    (0, 17),
]


# ======================================================================
# 工具函数
# ======================================================================

def calculate_distance(p1, p2):
    """
    计算两个 MediaPipe landmark 之间的欧氏距离。

    参数：
        p1, p2: MediaPipe NormalizedLandmark 对象，包含 .x 和 .y 归一化坐标
    返回：
        float: 两点之间的欧氏距离
    """
    return math.hypot(p1.x - p2.x, p1.y - p2.y)


def is_finger_extended(landmarks, tip_idx, pip_idx, wrist_idx=0):
    """
    判断非拇指手指是否伸直。

    原理：手指伸直时，指尖(TIP)到手腕的距离 > 近端关节(PIP)到手腕的距离；
          手指弯曲时，指尖(TIP)会被拉回，距离反而比近端关节近。

    参数：
        landmarks: 关键点列表（NormalizedLandmark 的列表）
        tip_idx:   指尖索引（如 8=食指尖, 12=中指尖, 16=无名指尖, 20=小指尖）
        pip_idx:   近端指间关节索引（如 6=食指PIP, 10=中指PIP, 14=无名指PIP, 18=小指PIP）
        wrist_idx: 手腕索引（默认 0）
    返回：
        bool: True=手指伸直，False=手指弯曲
    """
    tip = landmarks[tip_idx]
    pip = landmarks[pip_idx]
    wrist = landmarks[wrist_idx]

    # 指尖到手腕的距离
    dist_tip_to_wrist = calculate_distance(tip, wrist)
    # 近端关节到手腕的距离
    dist_pip_to_wrist = calculate_distance(pip, wrist)

    # 指尖比近端关节离手腕更远 → 手指伸直
    return dist_tip_to_wrist > dist_pip_to_wrist


def is_thumb_extended(landmarks):
    """
    判断拇指是否伸直。

    原理：拇指运动方向与其他手指不同（横向伸展），
          通过比较拇指尖(4)与拇指指间关节(3)到食指掌指关节(5)的距离来判断。
          拇指伸直时，拇指尖离食指掌根更远。

    参数：
        landmarks: 关键点列表（NormalizedLandmark 的列表）
    返回：
        bool: True=拇指伸直，False=拇指弯曲
    """
    thumb_tip = landmarks[4]       # 拇指尖
    thumb_ip = landmarks[3]        # 拇指指间关节
    index_mcp = landmarks[5]       # 食指掌指关节

    # 拇指尖到食指掌根的距离
    dist_tip_to_index = calculate_distance(thumb_tip, index_mcp)
    # 拇指关节到食指掌根的距离
    dist_ip_to_index = calculate_distance(thumb_ip, index_mcp)

    # 拇指尖比拇指关节离食指掌根更远 → 拇指伸直
    # 乘以 1.1 增加判断余量，减少误判
    return dist_tip_to_index > dist_ip_to_index * 1.1


# ======================================================================
# 核心功能函数
# ======================================================================

def detect_hand_state(landmarks):
    """
    手势识别核心函数。
    根据 MediaPipe 返回的 21 个关键点，用几何规则判断手势。

    判断规则：
        - 5 根手指全部伸直 → "OPEN"（张开手掌）
        - 5 根手指全部弯曲 → "CLOSE"（握拳）
        - 其他情况 → "PARTIAL"（部分手指弯曲，不触发任何动作）

    参数：
        landmarks: MediaPipe 检测到的手部关键点列表（21 个 NormalizedLandmark）
    返回：
        str: "OPEN" | "CLOSE" | "PARTIAL"
    """
    # 四根非拇指手指的 (TIP索引, PIP索引)
    # 食指(8,6) / 中指(12,10) / 无名指(16,14) / 小指(20,18)
    finger_indices = [
        (8, 6),    # 食指
        (12, 10),  # 中指
        (16, 14),  # 无名指
        (20, 18),  # 小指
    ]

    # 逐根判断是否伸直
    fingers_extended = []
    for tip_idx, pip_idx in finger_indices:
        fingers_extended.append(
            is_finger_extended(landmarks, tip_idx, pip_idx)
        )

    # 拇指单独判断（运动方向不同）
    fingers_extended.append(is_thumb_extended(landmarks))

    # 统计伸直手指数量
    extended_count = sum(fingers_extended)

    # 判断手势
    if extended_count == 5:
        return "OPEN"
    elif extended_count == 0:
        return "CLOSE"
    else:
        return "PARTIAL"


def send_command(ser, command_byte):
    """
    串口发送封装函数。
    向 Arduino 发送单字节命令。

    参数：
        ser: pyserial 的 Serial 对象
        command_byte: 要发送的字节，b'O' 或 b'C'
    """
    try:
        ser.write(command_byte)
        # 打印发送日志
        print(f"[串口发送] 命令: {command_byte.decode('ascii')}")
    except Exception as e:
        print(f"[串口错误] 发送失败: {e}")


# ======================================================================
# 防抖器类
# ======================================================================

class GestureDebouncer:
    """
    手势防抖器。
    使用滑动窗口投票机制：维护最近 N 帧的识别结果，
    当窗口内众数（出现次数最多的结果）与当前确认状态不同时，
    且该众数出现的次数达到阈值，才确认状态改变。

    这样可以避免单帧误识别导致舵机不停运动。
    """

    def __init__(self, window_size=5):
        """
        初始化防抖器。

        参数：
            window_size: 滑动窗口大小（帧数）
        """
        self.window_size = window_size
        self.history = deque(maxlen=window_size)
        self.current_confirmed_state = "CLOSE"  # 初始状态为关闭

    def update(self, gesture):
        """
        更新防抖队列，返回是否应触发状态改变。

        参数：
            gesture: 当前帧识别到的手势（"OPEN" / "CLOSE" / "PARTIAL"）
        返回：
            str or None:
                - 如果防抖后状态发生变化，返回新状态（"OPEN" 或 "CLOSE"）
                - 如果状态未变化或手势为 PARTIAL，返回 None
        """
        # PARTIAL 状态不进入防抖队列，直接忽略
        if gesture == "PARTIAL":
            return None

        # 将当前帧手势加入滑动窗口
        self.history.append(gesture)

        # 窗口未填满时，不确认状态改变
        if len(self.history) < self.window_size:
            return None

        # 投票：取窗口内出现次数最多的手势
        most_common = Counter(self.history).most_common(1)[0]
        voted_gesture = most_common[0]
        vote_count = most_common[1]

        # 只有当投票结果与当前确认状态不同，且票数达到阈值时才确认改变
        if voted_gesture != self.current_confirmed_state and vote_count >= self.window_size:
            # 确认状态改变
            old_state = self.current_confirmed_state
            self.current_confirmed_state = voted_gesture
            print(f"[状态改变] {old_state} → {self.current_confirmed_state}")
            return self.current_confirmed_state

        return None


# ======================================================================
# 界面绘制函数
# ======================================================================

def draw_hand_landmarks(image, landmarks):
    """
    在画面上手动绘制手部关键点和骨架连线。
    由于新版 MediaPipe Tasks API 不再内置 drawing_utils，需要用 OpenCV 手动绘制。

    参数：
        image: OpenCV 图像帧（numpy 数组）
        landmarks: 关键点列表（NormalizedLandmark 的列表，坐标归一化到 [0,1]）
    返回：
        numpy 数组: 绘制了关键点和连线的图像
    """
    h, w = image.shape[:2]

    # --- 绘制骨架连线 ---
    for connection in HAND_CONNECTIONS:
        start_idx, end_idx = connection
        start_pt = landmarks[start_idx]
        end_pt = landmarks[end_idx]
        # 归一化坐标 → 像素坐标
        x1, y1 = int(start_pt.x * w), int(start_pt.y * h)
        x2, y2 = int(end_pt.x * w), int(end_pt.y * h)
        # 绘制白色连线
        cv2.line(image, (x1, y1), (x2, y2), (255, 255, 255), 2)

    # --- 绘制关键点 ---
    for i, lm in enumerate(landmarks):
        x, y = int(lm.x * w), int(lm.y * h)
        # 指尖用红色，其他关节用蓝色
        if i in [4, 8, 12, 16, 20]:
            cv2.circle(image, (x, y), 6, (0, 0, 255), -1)
        else:
            cv2.circle(image, (x, y), 4, (255, 0, 0), -1)

    return image


def draw_ui(image, detected_gesture, current_state, fps):
    """
    在视频画面上绘制 UI 信息。

    参数：
        image: OpenCV 图像帧（numpy 数组）
        detected_gesture: 当前帧检测到的手势（"OPEN" / "CLOSE" / "PARTIAL" / "NO HAND"）
        current_state: 当前确认的状态（"OPEN" / "CLOSE"）
        fps: 当前帧率
    返回：
        numpy 数组: 绘制了 UI 的图像
    """
    # --- 左上角：当前状态 ---
    # 根据状态选择颜色（绿色=张开，红色=关闭）
    state_color = (0, 255, 0) if current_state == "OPEN" else (0, 0, 255)
    state_text = f"Current State: {current_state}"
    cv2.putText(image, state_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)

    # --- 左上角下方：当前识别的手势 ---
    # 手势显示文本映射
    gesture_display = {
        "OPEN": "OPEN PALM",
        "CLOSE": "CLOSED FIST",
        "PARTIAL": "PARTIAL",
        "NO HAND": "NO HAND",
    }
    gesture_text = f"Detected: {gesture_display.get(detected_gesture, detected_gesture)}"
    cv2.putText(image, gesture_text, (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    # --- 左下角：FPS ---
    fps_text = f"FPS: {fps:.1f}"
    cv2.putText(image, fps_text, (10, image.shape[0] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    # --- 右下角：退出提示 ---
    exit_text = "Press Q to quit"
    text_size = cv2.getTextSize(exit_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    cv2.putText(image, exit_text,
                (image.shape[1] - text_size[0] - 10, image.shape[0] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    return image


# ======================================================================
# 主程序入口
# ======================================================================

def main():
    """
    主程序入口。

    流程：
        1. 打开串口（失败则打印错误并退出）
        2. 初始化 MediaPipe HandLandmarker（新版 Tasks API）
        3. 打开摄像头
        4. 主循环：
           a. 读取一帧画面
           b. BGR → RGB 转换，送入 HandLandmarker 检测
           c. 如果检测到手 → detect_hand_state() 识别手势
           d. 防抖器过滤
           e. 状态变化时 → send_command() 发送串口命令
           f. 绘制 UI → 显示画面
           g. 按 Q 退出
        5. 释放资源（摄像头、串口、窗口）
    """
    # ==================== 1. 打开串口 ====================
    print(f"[初始化] 正在打开串口 {SERIAL_PORT} @ {BAUD_RATE} baud ...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        # 等待串口初始化稳定
        time.sleep(1)
        print(f"[初始化] 串口 {SERIAL_PORT} 打开成功")
    except Exception as e:
        print(f"[错误] 串口打开失败: {e}")
        print("[错误] 请检查串口号是否正确，以及设备是否已连接。")
        sys.exit(1)

    # ==================== 2. 初始化 MediaPipe HandLandmarker ====================
    # 检查模型文件是否存在
    if not os.path.exists(MODEL_PATH):
        print(f"[错误] 模型文件不存在: {MODEL_PATH}")
        print("[错误] 请下载 hand_landmarker.task 文件放到 python/ 目录下。")
        ser.close()
        sys.exit(1)

    # 使用新版 Tasks API 初始化 HandLandmarker
    BaseOptions = mp.tasks.BaseOptions
    HandLandmarker = mp.tasks.vision.HandLandmarker
    HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode

    # 配置 HandLandmarker 选项
    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=VisionRunningMode.VIDEO,    # 视频流模式
        num_hands=MAX_NUM_HANDS,                  # 最多检测手数
        min_hand_detection_confidence=MIN_DETECTION_CONFIDENCE,
        min_hand_presence_confidence=MIN_TRACKING_CONFIDENCE,
        min_tracking_confidence=MIN_TRACKING_CONFIDENCE,
    )

    # 创建 HandLandmarker 实例
    hand_landmarker = HandLandmarker.create_from_options(options)
    print(f"[初始化] MediaPipe HandLandmarker 初始化成功（模型: {MODEL_PATH}）")

    # ==================== 3. 打开摄像头 ====================
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("[错误] 摄像头打开失败，请检查设备。")
        ser.close()
        sys.exit(1)
    print("[初始化] 摄像头打开成功")

    # ==================== 初始化防抖器 ====================
    debouncer = GestureDebouncer(window_size=DEBOUNCE_FRAMES)

    # 当前确认的状态（初始为 CLOSE）
    current_state = "CLOSE"

    # FPS 计算相关变量
    prev_frame_time = time.time()
    fps = 0.0

    # 视频时间戳（毫秒），用于 MediaPipe VIDEO 模式
    frame_timestamp = 0

    print("[系统] 系统就绪，开始手势识别...")
    print("[系统] 按 Q 键退出")

    # ==================== 4. 主循环 ====================
    try:
        while cap.isOpened():
            # --- a. 读取一帧画面 ---
            success, image = cap.read()
            if not success:
                print("[警告] 读取摄像头画面失败，跳过该帧")
                continue

            # 镜像翻转（前置摄像头体验：左右镜像）
            image = cv2.flip(image, 1)

            # --- b. BGR → RGB 转换，送入 MediaPipe 处理 ---
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # 创建 MediaPipe Image 对象（新版 API 要求）
            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=rgb_image
            )

            # 更新时间戳（毫秒，必须单调递增）
            frame_timestamp += 1

            # 使用 VIDEO 模式检测
            result = hand_landmarker.detect_for_video(mp_image, frame_timestamp)

            # --- c/d/e. 手势检测 + 防抖 + 串口发送 ---
            if result.hand_landmarks:
                # 取第一只手的关键点（新版 API 返回的是列表的列表）
                hand_landmarks = result.hand_landmarks[0]

                # 在画面上绘制手部关键点和连线（手动绘制）
                image = draw_hand_landmarks(image, hand_landmarks)

                # 识别手势
                detected_gesture = detect_hand_state(hand_landmarks)

                # 防抖处理，检查是否需要改变状态
                new_state = debouncer.update(detected_gesture)

                # 状态变化时发送串口命令
                if new_state is not None:
                    if new_state == "OPEN":
                        send_command(ser, b'O')
                    elif new_state == "CLOSE":
                        send_command(ser, b'C')
                    current_state = new_state

            else:
                # 未检测到手
                detected_gesture = "NO HAND"
                # 不发送任何命令，保持当前状态

            # --- f. FPS 计算 ---
            curr_frame_time = time.time()
            fps = 1.0 / (curr_frame_time - prev_frame_time + 1e-6)
            prev_frame_time = curr_frame_time

            # --- 绘制 UI ---
            image = draw_ui(image, detected_gesture, current_state, fps)

            # --- 显示画面 ---
            cv2.imshow('Hand Gesture Control', image)

            # --- g. 按 Q 退出 ---
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[系统] 收到退出指令，正在关闭...")
                break

    except KeyboardInterrupt:
        # Ctrl+C 中断
        print("\n[系统] 用户中断，正在关闭...")

    finally:
        # ==================== 5. 清理资源 ====================
        print("[清理] 释放摄像头...")
        cap.release()

        print("[清理] 关闭 HandLandmarker...")
        hand_landmarker.close()

        print("[清理] 关闭串口...")
        if ser.is_open:
            ser.close()

        print("[清理] 关闭所有 OpenCV 窗口...")
        cv2.destroyAllWindows()

        print("[系统] 资源已全部释放，程序退出。")


# ==================== 程序入口 ====================
if __name__ == "__main__":
    main()
