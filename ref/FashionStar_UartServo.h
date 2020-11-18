/* 
 * FashionStar串口总线舵机ArduinoSDK
 * --------------------------
 */
#ifndef _FS_UART_SERVO_H
#define _FS_UART_SERVO_H

#include <Arduino.h>
#include "FashionStar_UartServoProtocal.h"

#define FSUS_K_ANGLE_REAL2RAW 1
#define FSUS_B_ANGLE_REAL2RAW 0
#define FSUS_SERVO_SPEED 100.0 // 舵机角度的默认转速
#define FSUS_ANGLE_CTL_DEADBLOCK 0.2 // 舵机控制的死区
#define FSUS_WAIT_TIMEOUT_MS 100 // 等待的时间上限 ms

class FSUS_Servo{
public:
    FSUS_Protocal *protocal; // 舵机串口通信协议
    FSUS_SERVO_ID_T servoId; //舵机ID
    bool isOnline; //舵机是否在线

    float kAngleReal2Raw; // 舵机标定数据-舵机角度与位置之间的比例系数
    float bAngleReal2Raw; // 舵机标定数据-舵机角度与位置转换过程中的偏移量

    FSUS_SERVO_ANGLE_T curAngle; // 真实的当前角度
    FSUS_SERVO_ANGLE_T targetAngle; // 真实的目标角度

    // FSUS_SERVO_ANGLE_T curRawAngle; // 原始的当前角度
    // FSUS_SERVO_ANGLE_T targetRawAngle; // 原始的目标角度
    
    FSUS_SERVO_ANGLE_T angleMin; //舵机角度的最小值
    FSUS_SERVO_ANGLE_T angleMax; // 舵机角度最大值

    FSUS_SERVO_SPEED_T speed; // 舵机转速 单位dps °/s
    
    // 构造函数
    FSUS_Servo();
    FSUS_Servo(uint8_t servoId, FSUS_Protocal *protocal); 
    void init(); // 初始化
    void init(uint8_t servoId, FSUS_Protocal *protocal);
    //舵机通讯检测
    bool ping();
    // 舵机标定
    void calibration(FSUS_SERVO_ANGLE_T rawA, FSUS_SERVO_ANGLE_T realA, FSUS_SERVO_ANGLE_T rawB, FSUS_SERVO_ANGLE_T realB);
    void calibration(float kAngleReal2Raw, float bAngleReal2Raw);
    // 真实角度转化为原始角度
    FSUS_SERVO_ANGLE_T angleReal2Raw(FSUS_SERVO_ANGLE_T realAngle);    
    // 原始角度转换为真实角度
    FSUS_SERVO_ANGLE_T angleRaw2Real(FSUS_SERVO_ANGLE_T rawAngle);
    // 查询舵机的角度
    FSUS_SERVO_ANGLE_T queryAngle();
    // 查询舵机原始的角度
    FSUS_SERVO_ANGLE_T queryRawAngle();
    // 范围是否合法
    bool isAngleLegal(FSUS_SERVO_ANGLE_T candiAngle);
    // 设置舵机的角度范围
    void setAngleRange(FSUS_SERVO_ANGLE_T angleMin, FSUS_SERVO_ANGLE_T angleMax);
    // 设置舵机的平均转速
    void setSpeed(FSUS_SERVO_SPEED_T speed);
    // 设置舵机角度
    void setAngle(FSUS_SERVO_ANGLE_T angle);
    void setAngle(FSUS_SERVO_ANGLE_T angle, FSUS_INTERVAL_T interval);
    // 设置舵机原始角度
    void setRawAngle(FSUS_SERVO_ANGLE_T rawAngle, FSUS_INTERVAL_T interval);
    void setRawAngle(FSUS_SERVO_ANGLE_T rawAngle);
    // 设置阻尼模式
    void setDamping(FSUS_POWER_T power);
    void setDamping();
    // 设置轮式模式
    // 轮子停止
    void wheelStop();
    // 轮子旋转
    void wheelRun(uint8_t is_cw);
    // 轮子旋转特定的时间
    void wheelRunNTime(uint8_t is_cw, uint16_t time_ms);
    // 旋转圈数
    void wheelRunNCircle(uint8_t is_cw, uint16_t circle_num);
    // 查询轮子是否在旋转
    bool wheelIsStop();
    // 是否开启扭力
    void setTorque(bool enable);
    // 舵机是否在旋转
    bool isStop();
    // 舵机等待
    void wait();
private:

};// NOTE：类的末尾要加;
#endif