/*
 * ======================================================================
 * 项目名称：手势连续控制 DS3225 舵机
 * 硬件平台：Arduino Mega 2560
 * 舵机型号：DS3225 数字舵机
 * 
 * 功能说明：
 *   通过串口接收上位机（Python）发送的脉宽值，
 *   连续控制 DS3225 舵机位置（1100us~2200us）。
 *   实现无级调节夹爪开合程度。
 * 
 * 通信协议：
 *   波特率：115200
 *   接收格式："脉宽值\n"（ASCII 数字字符串 + 换行符）
 *   例如："1500\n" → 舵机运动到 1500us
 *   有效范围：1100~2200（超出范围会被自动限幅）
 * 
 * 接线说明：
 *   舵机信号线 → D9
 *   舵机电源线 → 外部供电（5V~7.4V，根据舵机规格）
 *   舵机地线   → GND（与 Arduino 共地）
 *   Arduino通过USB连接电脑
 * ======================================================================
 */

#include <Servo.h>

// ==================== 引脚定义 ====================
// DS3225 舵机信号线连接的引脚
#define SERVO_PIN 9

// ==================== 脉宽定义 ====================
// 舵机脉宽范围（微秒）
#define PULSE_MIN 1100    // 最小脉宽（夹爪完全关闭）
#define PULSE_MAX 2200    // 最大脉宽（夹爪完全张开）

// ==================== 串口参数 ====================
// 波特率，需与 Python 端保持一致
#define BAUD_RATE 115200

// 串口读取超时（毫秒）
// 超过此时间未收到完整数据则放弃当前读取
#define SERIAL_TIMEOUT 50

// ==================== 全局变量 ====================
// 舵机控制对象
Servo myServo;

/*
 * ======================================================================
 * setup() — 初始化函数
 * 在 Arduino 上电或复位时执行一次
 * ======================================================================
 */
void setup() {
  // 初始化串口通信
  Serial.begin(BAUD_RATE);
  
  // 等待串口就绪
  while (!Serial) {
    ;
  }

  // 设置串口读取超时
  Serial.setTimeout(SERIAL_TIMEOUT);

  // 将舵机信号引脚绑定到 Servo 对象
  myServo.attach(SERVO_PIN);

  // 初始位置：设为夹爪关闭状态
  myServo.writeMicroseconds(PULSE_MIN);

  // 打印初始化完成信息
  Serial.println("Arduino Mega 2560 - DS3225 Continuous Servo Control Ready");
  Serial.println("Format: send pulse width as ASCII string + newline, e.g. \"1500\\n\"");
  Serial.print("Range: ");
  Serial.print(PULSE_MIN);
  Serial.print(" ~ ");
  Serial.println(PULSE_MAX);
}

/*
 * ======================================================================
 * loop() — 主循环
 * 持续监听串口，收到脉宽值后控制舵机
 * ======================================================================
 */
void loop() {
  // 检查串口是否有数据可读
  if (Serial.available() > 0) {
    // 读取一行数据（以换行符 '\n' 结尾）
    String input = Serial.readStringUntil('\n');
    
    // 去除可能的回车符 '\r'（Windows 换行为 \r\n）
    input.trim();
    
    // 如果读取到非空字符串
    if (input.length() > 0) {
      // 将字符串解析为整数（脉宽值）
      long pulse = input.toInt();
      
      // 安全限幅：确保脉宽在有效范围内
      pulse = constrain(pulse, PULSE_MIN, PULSE_MAX);
      
      // 控制舵机运动到指定脉宽
      myServo.writeMicroseconds((int)pulse);
    }
  }
}
