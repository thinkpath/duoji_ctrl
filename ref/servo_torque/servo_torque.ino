/*
 * 测试舵机扭力开关
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
void setup(){
    protocal.init(); // 通信协议初始化
    uservo.init(); //舵机角度初始化

    uservo.setTorque(true); // 开启扭力
    // uservo.setTorque(false); // 开启扭力
}

void loop(){
    
}