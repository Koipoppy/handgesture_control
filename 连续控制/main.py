"""
======================================================================
项目名称：手势连续控制 DS3225 舵机 — Python 上位机
功能说明：
    使用电脑 USB 摄像头实时识别手的开合程度，
    将开合度连续映射到舵机脉宽（1100us~2200us），
    实现无级控制夹爪开合。

    手张开程度越大 → 脉宽越大 → 夹爪越张开
    手握拳程度越大 → 脉宽越小 → 夹爪越关闭

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
from collections import deque
import time
import sys
import os

# ======================================================================
# 配置区（可修改参数集中在此处）
# ======================================================================

# --- 串口配置 ---
SERIAL_PORT = "COM9"        # 串口号，根据实际连接修改
BAUD_RATE = 115200           # 波特率，需与 Arduino 端保持一致

# --- 摄像头配置 ---
CAMERA_INDEX = 0             # 摄像头索引（0=默认摄像头）

# --- 舵机脉宽范围 ---
PULSE_MIN = 1100            # 最小脉宽（握拳时，夹爪关闭）
PULSE_MAX = 2200            # 最大脉宽（张开时，夹爪全开）

# --- 开合度归一化参数 ---
# 伸展比 = 指尖到手腕距离 / 近端关节到手腕距离
# 握拳时约 0.7~0.9，张开时约 1.2~1.5
# 以下范围用于将伸展比归一化到 [0.0, 1.0]
RATIO_MIN = 0.8             # 伸展比下限（对应 openness=0，握拳）
RATIO_MAX = 1.3             # 伸展比上限（对应 openness=1，张开）

# --- 非线性灵敏度参数 ---
# 用幂函数 openness = t^GAMMA 替代线性映射：
#   GAMMA > 1 → 闭合侧低灵敏度（舵机平缓），打开侧高灵敏度（舵机跟手）
#   GAMMA = 1.0 → 线性（均匀灵敏度）
#   GAMMA < 1 → 打开侧低灵敏度，闭合侧高灵敏度（与需求相反）
GAMMA = 2.0

# --- 平滑参数 ---
SMOOTH_WINDOW = 5           # 滑动平均窗口大小（帧数），越大越平滑但延迟越高

# --- 串口发送间隔 ---
SEND_INTERVAL = 0.05        # 最小发送间隔（秒），每秒最多发送 20 次

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


# ======================================================================
# 核心功能函数
# ======================================================================

def calculate_hand_openness(landmarks):
    """
    计算手的开合度（连续值）。

    原理：对每根手指计算"伸展比"——
        指尖到手腕的距离 / 近端关节到手腕的距离。
    手指伸直时比值 > 1（指尖比关节远），
    手指弯曲时比值 < 1（指尖被拉回，比关节近）。

    仅使用 4 根非拇指手指（食指/中指/无名指/小指）的伸展比取平均，
    不依赖拇指（拇指被遮挡时 MediaPipe 估计的姿态会跳变）。

    然后用幂函数非线性映射到 [0.0, 1.0]：
        - 0.0 = 完全握拳
        - 1.0 = 完全张开
        GAMMA > 1 使得闭合侧灵敏度低（平缓），打开侧灵敏度高（跟手）

    参数：
        landmarks: MediaPipe 检测到的手部关键点列表（21 个 NormalizedLandmark）
    返回：
        float: 开合度，范围 [0.0, 1.0]
    """
    wrist = landmarks[0]  # 手腕

    # --- 四根非拇指手指的伸展比 ---
    # 不使用拇指：拇指经常被遮挡，MediaPipe 估计的关键点不稳定
    # 每项: (TIP索引, PIP索引)
    finger_indices = [
        (8, 6),    # 食指
        (12, 10),  # 中指
        (16, 14),  # 无名指
        (20, 18),  # 小指
    ]

    ratios = []
    for tip_idx, pip_idx in finger_indices:
        tip = landmarks[tip_idx]
        pip = landmarks[pip_idx]
        # 指尖到手腕距离 / 近端关节到手腕距离
        dist_tip = calculate_distance(tip, wrist)
        dist_pip = calculate_distance(pip, wrist)
        # 避免除零
        if dist_pip > 1e-6:
            ratios.append(dist_tip / dist_pip)

    # 计算平均伸展比（4 指平均）
    avg_ratio = sum(ratios) / len(ratios) if ratios else 1.0

    # 线性归一化到 [0.0, 1.0]
    t = (avg_ratio - RATIO_MIN) / (RATIO_MAX - RATIO_MIN)
    t = max(0.0, min(1.0, t))

    # 非线性映射：闭合侧低灵敏度，打开侧高灵敏度
    openness = t ** GAMMA

    return openness


def map_to_pulse(openness):
    """
    将开合度 [0.0, 1.0] 线性映射到舵机脉宽 [PULSE_MIN, PULSE_MAX]。

    参数：
        openness: 开合度，范围 [0.0, 1.0]
    返回：
        int: 舵机脉宽值（微秒），范围 [1100, 2200]
    """
    pulse = PULSE_MIN + openness * (PULSE_MAX - PULSE_MIN)
    return int(pulse)


def send_pulse(ser, pulse):
    """
    通过串口发送脉宽值给 Arduino。
    发送格式: "脉宽值\n"（ASCII 字符串 + 换行符）

    参数：
        ser: pyserial 的 Serial 对象
        pulse: 舵机脉宽值（微秒），如 1500
    """
    try:
        # 发送格式: "1500\n"
        ser.write(f"{pulse}\n".encode('ascii'))
    except Exception as e:
        print(f"[串口错误] 发送失败: {e}")


# ======================================================================
# 平滑器类
# ======================================================================

class ValueSmoother:
    """
    滑动平均滤波器。
    维护最近 N 个值的滑动窗口，返回平均值。
    用于平滑开合度，消除单帧抖动。
    """

    def __init__(self, window_size=5):
        """
        初始化平滑器。

        参数：
            window_size: 滑动窗口大小（帧数）
        """
        self.window_size = window_size
        self.history = deque(maxlen=window_size)

    def update(self, value):
        """
        加入新值并返回滑动平均。

        参数：
            value: 当前帧的原始值
        返回：
            float: 滑动平均后的值
        """
        self.history.append(value)
        return sum(self.history) / len(self.history)


# ======================================================================
# 界面绘制函数
# ======================================================================

def draw_hand_landmarks(image, landmarks):
    """
    在画面上手动绘制手部关键点和骨架连线。

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


def draw_ui(image, openness, pulse, fps, hand_detected):
    """
    在视频画面上绘制 UI 信息。

    参数：
        image: OpenCV 图像帧（numpy 数组）
        openness: 开合度 [0.0, 1.0]
        pulse: 当前脉宽值（微秒）
        fps: 当前帧率
        hand_detected: 是否检测到手
    返回：
        numpy 数组: 绘制了 UI 的图像
    """
    # --- 左上角：开合度百分比 ---
    openness_pct = int(openness * 100)

    # 根据开合度选择颜色：0%红色 → 50%黄色 → 100%绿色
    if openness > 0.5:
        # 黄色到绿色过渡
        green = 255
        red = int(255 * (1 - (openness - 0.5) * 2))
        color = (0, green, max(0, red))
    else:
        # 红色到黄色过渡
        red = 255
        green = int(255 * openness * 2)
        color = (0, max(0, green), red)

    openness_text = f"Openness: {openness_pct}%"
    cv2.putText(image, openness_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    # --- 进度条 ---
    # 在开合度文字下方画一个进度条
    bar_x, bar_y = 10, 45
    bar_w, bar_h = 200, 15
    # 背景框（灰色）
    cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                  (50, 50, 50), -1)
    # 填充（根据开合度）
    fill_w = int(bar_w * openness)
    cv2.rectangle(image, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h),
                  color, -1)
    # 边框
    cv2.rectangle(image, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h),
                  (200, 200, 200), 1)

    # --- 左上角下方：脉宽值 ---
    pulse_text = f"Pulse: {pulse}us"
    cv2.putText(image, pulse_text, (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

    # --- 左上角更下方：手检测状态 ---
    if hand_detected:
        hand_text = "Hand: DETECTED"
        hand_color = (0, 255, 0)
    else:
        hand_text = "Hand: NO HAND"
        hand_color = (128, 128, 128)
    cv2.putText(image, hand_text, (10, 115),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, hand_color, 1)

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
           c. 如果检测到手 → calculate_hand_openness() 计算开合度
           d. 滑动平均平滑
           e. 映射到脉宽 → send_pulse() 发送串口命令
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
        print("[错误] 请下载 hand_landmarker.task 文件。")
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
        running_mode=VisionRunningMode.VIDEO,
        num_hands=MAX_NUM_HANDS,
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

    # ==================== 初始化平滑器 ====================
    smoother = ValueSmoother(window_size=SMOOTH_WINDOW)

    # 当前脉宽值（初始为关闭位置）
    current_pulse = PULSE_MIN

    # 上次发送时间（用于发送间隔限流）
    last_send_time = 0.0

    # FPS 计算相关变量
    prev_frame_time = time.time()
    fps = 0.0

    # 视频时间戳（毫秒），用于 MediaPipe VIDEO 模式
    frame_timestamp = 0

    print("[系统] 系统就绪，开始连续手势控制...")
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

            # 更新时间戳（必须单调递增）
            frame_timestamp += 1

            # 使用 VIDEO 模式检测
            result = hand_landmarker.detect_for_video(mp_image, frame_timestamp)

            # --- c/d/e. 开合度计算 + 平滑 + 串口发送 ---
            if result.hand_landmarks:
                # 取第一只手的关键点
                hand_landmarks = result.hand_landmarks[0]

                # 在画面上绘制手部关键点和连线
                image = draw_hand_landmarks(image, hand_landmarks)

                # 计算原始开合度
                raw_openness = calculate_hand_openness(hand_landmarks)

                # 滑动平均平滑
                smoothed_openness = smoother.update(raw_openness)

                # 映射到脉宽
                current_pulse = map_to_pulse(smoothed_openness)

                # 串口发送（限流：每 SEND_INTERVAL 秒最多发一次）
                curr_time = time.time()
                if curr_time - last_send_time >= SEND_INTERVAL:
                    send_pulse(ser, current_pulse)
                    last_send_time = curr_time

                # 标记检测到手
                hand_detected = True
                display_openness = smoothed_openness

            else:
                # 未检测到手，不发送命令，保持当前脉宽
                hand_detected = False
                display_openness = (current_pulse - PULSE_MIN) / (PULSE_MAX - PULSE_MIN)

            # --- f. FPS 计算 ---
            curr_frame_time = time.time()
            fps = 1.0 / (curr_frame_time - prev_frame_time + 1e-6)
            prev_frame_time = curr_frame_time

            # --- 绘制 UI ---
            image = draw_ui(image, display_openness, current_pulse, fps, hand_detected)

            # --- 显示画面 ---
            cv2.imshow('Hand Gesture Continuous Control', image)

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
