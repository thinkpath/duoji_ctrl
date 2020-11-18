/* 
 * 设置舵机的角度
 * 让舵机在两个角度之间进行切换, 并动态的查询舵机的角度
 * 提示: 拓展板上电之后, 记得按下Arduino的RESET按键
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

SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
FSUS_Protocal protocal(BAUDRATE); //协议
FSUS_Servo uservo(SERVO_ID, &protocal); // 创建舵机

// 等待舵机旋转到目标角度
// 注: 在执行isStop的函数的时候, 本身就会查询当前舵机的角度
void waitUntilStop(FSUS_Servo* uservo){
    delay(1); //要等待一段时间才可以发送舵机角度查询的指令, 防止被覆盖.
    while(true){
        String message = "Servo Running, Angle = " + String(uservo->curAngle,1)+" Target Angle = "+String(uservo->targetAngle, 1);
        softSerial.println(message);
        if (uservo->isStop()){
            break;
        }
    }
    softSerial.println("Servo Stop, Angle = "+String(uservo->curAngle,1));
}

void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE); // 初始化软串口的波特率
    protocal.init(); // 通信协议初始化
    uservo.init(); //舵机角度初始化

    softSerial.println("Set Servo Angle");

}

void loop(){
    softSerial.println("Set Angle = 90°");
    uservo.setAngle(90.0); // 设置舵机的角度
    waitUntilStop(&uservo); // 等待舵机旋转到目标角度
    // 暂停2s
    delay(2000);
    
    softSerial.println("Set Angle = -90°");
    uservo.setAngle(-90);
    waitUntilStop(&uservo);
    delay(2000);
}