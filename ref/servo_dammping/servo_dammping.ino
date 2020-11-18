/*
 * 设置舵机为阻尼模式
 * 调整参数`DAMMPING_POWER`感受不同的阻尼力
 * --------------------------
*/
#include <SoftwareSerial.h>
#include "FashionStar_UartServoProtocal.h"
#include "FashionStar_UartServo.h" // Fashion Star串口总线舵机的依赖

// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800

// 配置参数
#define BAUDRATE 115200 // 波特率

#define SERVO_ID 0 //舵机ID号
#define DAMMPING_POWER 800 // 阻尼模式下的功率(单位mW) 500,800,1000

SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
FSUS_Protocal protocal(BAUDRATE); //协议
FSUS_Servo uservo(SERVO_ID, &protocal); // 创建舵机

void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE);
    protocal.init(); // 通信协议初始化
    uservo.init(); // 舵机初始化

    softSerial.println("Set Servo Mode To Dammping");
    uservo.setDamping(DAMMPING_POWER);
}

void loop(){
    // TODO;
}
