/*
 * ======================================================================
 * 项目名称：手势控制 DS3225 舵机
 * 硬件平台：Arduino Mega 2560
 * 舵机型号：DS3225 数字舵机
 * 
 * 功能说明：
 *   通过串口接收上位机（Python）发送的手势命令，
 *   控制 DS3225 舵机在最大位置（夹爪张开）和最小位置（夹爪关闭）之间切换。
 * 
 * 通信协议：
 *   波特率：115200
 *   命令 'O' → 舵机运动到最大位置（2200us，夹爪张开）
 *   命令 'C' → 舵机运动到最小位置（1100us，夹爪关闭）
 *   其他字符 → 不执行任何动作
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
// 夹爪张开：最大位置脉宽（微秒）
// DS3225 舵机在 2200us 时运动到最大位置
#define PULSE_OPEN 2200

// 夹爪关闭：最小位置脉宽（微秒）
// DS3225 舵机在 1100us 时运动到最小位置
#define PULSE_CLOSE 1100

// ==================== 串口参数 ====================
// 波特率，需与 Python 端保持一致
#define BAUD_RATE 115200

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
  
  // 等待串口就绪（仅对带有原生 USB 的板子有效，如 Leonardo）
  // Mega 2560 有硬件串口，通常不需要等待，但加上更安全
  while (!Serial) {
    ;
  }

  // 将舵机信号引脚绑定到 Servo 对象
  myServo.attach(SERVO_PIN);

  // 初始位置：设为夹爪关闭状态
  // 上电时默认关闭，与 Python 端的初始 current_state="CLOSE" 对应
  myServo.writeMicroseconds(PULSE_CLOSE);

  // 打印初始化完成信息
  Serial.println("Arduino Mega 2560 - DS3225 Servo Control Ready");
  Serial.println("Commands: 'O' = Open (2200us), 'C' = Close (1100us)");
}

/*
 * ======================================================================
 * loop() — 主循环
 * 持续监听串口，收到命令后控制舵机
 * ======================================================================
 */
void loop() {
  // 检查串口是否有数据可读
  if (Serial.available() > 0) {
    // 读取一个字节
    char command = Serial.read();

    // 根据命令控制舵机
    switch (command) {
      case 'O':
        // 收到 'O' 命令 → 夹爪张开
        // 舵机运动到最大位置
        myServo.writeMicroseconds(PULSE_OPEN);
        Serial.println("OPEN - Servo moved to 2200us");
        break;

      case 'C':
        // 收到 'C' 命令 → 夹爪关闭
        // 舵机运动到最小位置
        myServo.writeMicroseconds(PULSE_CLOSE);
        Serial.println("CLOSE - Servo moved to 1100us");
        break;

      default:
        // 收到其他字符，不执行任何动作
        break;
    }
  }
}
