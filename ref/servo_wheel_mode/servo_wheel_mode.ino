/*
 * 测试舵机轮式模式
 * 提示: 拓展板上电之后, 记得按下Arduino的RESET按键
 * --------------------------
*/
#include "FashionStar_UartServoProtocal.h"
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// 配置参数
#define BAUDRATE 115200 // 波特率
#define SERVO_ID 0 //舵机ID号

FSUS_Protocal protocal(BAUDRATE); //协议
FSUS_Servo uservo(SERVO_ID, &protocal); // 创建舵机

/* 轮子持续旋转指令与停止指令测试 */
void testWheelRunAndStop(){
    uservo.wheelRun(FSUS_CCW); // 轮子持续旋转, 方向为逆时针
    delay(2000);            // 等待2s
    uservo.wheelStop();
    delay(2000);            // 等待2s
    uservo.wheelRun(FSUS_CW); // 轮子持续旋转
    delay(2000);            // 等待2s
    uservo.wheelStop();
    delay(2000);            // 等待2s
}

/* 测试轮子旋转特定的时间 */
void testWheelRunNTime(){
    uservo.wheelRunNTime(FSUS_CW, 5000); // 轮子持续旋转5s(顺时针)
    delay(5000);                         
    uservo.wheelRunNTime(FSUS_CCW, 5000); // 轮子持续旋转5s(逆时针)
    delay(5000);
}

/* 测试轮子旋转特定的圈数 */
void testWheelRunNCircle(){
    uint16_t nCircle = 2; // 旋转圈数
    uint16_t delayMsEstimate = (uint16_t)(360.0 * nCircle / uservo.speed * 1000); // 估计旋转的时间
    uservo.wheelRunNCircle(FSUS_CW, 2); // 轮子持续旋转2圈(顺时针)
    delay(delayMsEstimate);             // 等到轮子旋转到特定的位置 

    uservo.wheelRunNCircle(FSUS_CCW, 2);// 轮子持续旋转2圈(逆时针)
    delay(delayMsEstimate);             // 等到轮子旋转到特定的位置}
}

void setup(){
    protocal.init();        // 通信协议初始化
    uservo.init();          //舵机角度初始化
    uservo.setSpeed(100);    // 设置转速为20°/s

    // 测试持续旋转与停止
    //testWheelRunAndStop();

    // 测试旋转特定的时间
   // testWheelRunNTime();
    
    // 测试旋转特定的圈数
    testWheelRunNCircle();
}

void loop(){
}
