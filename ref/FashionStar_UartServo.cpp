/* 
 * FashionStar串口总线舵机ArduinoSDK
 * --------------------------
*/
#include "FashionStar_UartServo.h"

FSUS_Servo::FSUS_Servo(){

}

FSUS_Servo::FSUS_Servo(uint8_t servoId, FSUS_Protocal *protocal){
    this->servoId = servoId; // 设定舵机的ID号
    this->protocal = protocal; // 舵机的通信协议
    this->angleMin = FSUS_SERVO_ANGLE_MIN; // 设置默认角度的最小值
    this->angleMax = FSUS_SERVO_ANGLE_MAX; // 设置默认角度的最大值
    this->speed = FSUS_SERVO_SPEED; // 设置默认转速
    this->kAngleReal2Raw = FSUS_K_ANGLE_REAL2RAW;
    this->bAngleReal2Raw = FSUS_B_ANGLE_REAL2RAW;
    
}

/* 舵机初始化 */
void FSUS_Servo::init(){
    // ping一下舵机
    ping();
    if(this->isOnline){
        // 舵机如果在线就开始同步角度
        queryAngle();
        // 目标角度跟初始角度一致
        this->targetAngle = this->curAngle;
    }
}

void FSUS_Servo::init(uint8_t servoId, FSUS_Protocal *protocal){
    this->servoId = servoId; // 设定舵机的ID号
    this->protocal = protocal; // 舵机的通信协议
    this->angleMin = FSUS_SERVO_ANGLE_MIN; // 设置默认角度的最小值
    this->angleMax = FSUS_SERVO_ANGLE_MAX; // 设置默认角度的最大值
    this->speed = FSUS_SERVO_SPEED; // 设置默认转速
    this->kAngleReal2Raw = FSUS_K_ANGLE_REAL2RAW;
    this->bAngleReal2Raw = FSUS_B_ANGLE_REAL2RAW;
    init(); // 初始化舵机
}    

/*舵机通讯检测, 判断舵机是否在线*/
bool FSUS_Servo::ping(){
    this->protocal->emptyCache(); //清空串口缓冲区
    this->protocal->sendPing(servoId); //发送PING指令
    FSUS_SERVO_ID_T servoIdTmp;
    this->protocal->recvPing(&servoIdTmp, &(this->isOnline));
    return this->isOnline;
}

// 舵机标定
void FSUS_Servo::calibration(FSUS_SERVO_ANGLE_T rawA, FSUS_SERVO_ANGLE_T realA, FSUS_SERVO_ANGLE_T rawB, FSUS_SERVO_ANGLE_T realB){
    // rawAngle = kAngleReal2Raw*realAngle + bAngleReal2Raw
    this->kAngleReal2Raw = (rawA - rawB)/(realA-realB);
    this->bAngleReal2Raw = rawA - this->kAngleReal2Raw*realA;
}

// 舵机标定
void FSUS_Servo::calibration(float kAngleReal2Raw, float bAngleReal2Raw){
    this->kAngleReal2Raw = kAngleReal2Raw;
    this->bAngleReal2Raw = bAngleReal2Raw;
}

// 真实角度转化为原始角度
FSUS_SERVO_ANGLE_T FSUS_Servo::angleReal2Raw(FSUS_SERVO_ANGLE_T realAngle){
    return this->kAngleReal2Raw*realAngle + this->bAngleReal2Raw;
}

// 原始角度转换为真实角度
FSUS_SERVO_ANGLE_T FSUS_Servo::angleRaw2Real(FSUS_SERVO_ANGLE_T rawAngle){
    return (rawAngle -  this->bAngleReal2Raw) / this->kAngleReal2Raw;
}

// 设置舵机的角度
void FSUS_Servo::setAngleRange(FSUS_SERVO_ANGLE_T angleMin, FSUS_SERVO_ANGLE_T angleMax){
    this->angleMin = angleMin;
    this->angleMax = angleMax;
}

bool FSUS_Servo::isAngleLegal(FSUS_SERVO_ANGLE_T candiAngle){
    return candiAngle >= this->angleMin && candiAngle <= this->angleMax;
}


// 设置舵机的平均转速
void FSUS_Servo::setSpeed(FSUS_SERVO_SPEED_T speed){
    this->speed = speed;
}

/*设置舵机角度*/
void FSUS_Servo::setAngle(FSUS_SERVO_ANGLE_T angle){
    FSUS_SERVO_ANGLE_T dAngle; // 当前角度与目标角度之间的差值
    FSUS_INTERVAL_T interval; // 周期
    // 检舵机查角度是否合法
    angle = (angle < this->angleMin) ? this->angleMin: angle;
    angle = (angle > this->angleMax) ? this->angleMax: angle;
    // 检查舵机角度查询(更新当前的角度)
    this->queryAngle();
    dAngle = abs(angle - this->curAngle);
    // 计算角度差, 估计周期
    interval = (FSUS_INTERVAL_T)((dAngle/speed)*1000);
    setAngle(angle, interval);
}

/* 设置舵机角度,同时指定角度跟时间*/
void FSUS_Servo::setAngle(FSUS_SERVO_ANGLE_T angle, FSUS_INTERVAL_T interval){
    // 约束角度的范围
    angle = (angle < this->angleMin) ? this->angleMin: angle;
    angle = (angle > this->angleMax) ? this->angleMax: angle;
    // 发送舵机角度控制指令
    this->targetAngle = angle; // 设置目标角度
    setRawAngle(angleReal2Raw(this->targetAngle), interval);
}

/* 设置舵机的原始角度 */
void FSUS_Servo::setRawAngle(FSUS_SERVO_ANGLE_T rawAngle, FSUS_INTERVAL_T interval){
    this->protocal->sendSetAngle(this->servoId, rawAngle, interval, 0);
}
/* 设置舵机的原始角度 */
void FSUS_Servo::setRawAngle(FSUS_SERVO_ANGLE_T rawAngle){
    this->protocal->sendSetAngle(this->servoId, rawAngle, 0, 0);
}

/* 查询舵机当前的真实角度*/
FSUS_SERVO_ANGLE_T FSUS_Servo::queryAngle(){
    FSUS_SERVO_ANGLE_T rawAngle = queryRawAngle();
    this->curAngle = angleRaw2Real(rawAngle);
    return this->curAngle; 
}

/* 查询舵机当前的原始角度 */
FSUS_SERVO_ANGLE_T FSUS_Servo::queryRawAngle(){
    this->protocal->emptyCache(); //清空串口缓冲区
    this->protocal->sendQueryAngle(this->servoId);
    FSUS_SERVO_ID_T servoIdTmp;
    FSUS_SERVO_ANGLE_T rawAngle;
    this->protocal->recvQueryAngle(&servoIdTmp, &rawAngle);
    return rawAngle;
}

/* 设置舵机为阻尼模式 */
void FSUS_Servo::setDamping(FSUS_POWER_T power){
    this->protocal->sendDammping(this->servoId, power);
}

/* 设置舵机为阻尼模式, 默认功率为500mW*/
void FSUS_Servo::setDamping(){
    setDamping(500);
}

/* 舵机扭力开关 */
void FSUS_Servo::setTorque(bool enable){
    if(enable){
        queryAngle(); // 查询舵机角度
        setAngle(this->curAngle); //设置角度为当前的角度
    }else{
        setDamping(0); // 设置为阻尼模式, 功率为0
    }
}

/* 判断舵机是否停止 */
bool FSUS_Servo::isStop(){
    queryAngle(); // 查询舵机角度
    if (this->protocal->responsePack.recv_status != FSUS_STATUS_SUCCESS){
        // 舵机角度查询失败
        return false;
    }

    FSUS_SERVO_ANGLE_T curRawAngle = angleReal2Raw(curAngle);
    FSUS_SERVO_ANGLE_T targetRawAngle = angleReal2Raw(targetAngle);
    return abs(curRawAngle-targetRawAngle) <= FSUS_ANGLE_CTL_DEADBLOCK;
}

/* 舵机等待 */
void FSUS_Servo::wait(){
    long t_start = millis();
    // 角度误差
    float dAngle = abs(this->targetAngle - this->curAngle);
    
    while (!isStop()){
        // 角度误差保持不变(判断条件1°)
        if(abs(dAngle - abs(this->targetAngle - this->curAngle)) <= 1.0){
            // 判断是否发生卡死的情况
            if(millis() - t_start >= FSUS_WAIT_TIMEOUT_MS){
                // 等待超过1s
                break;
            }
        }else{
            // 更新t_start
            t_start = millis();
            // 更新角度误差
            dAngle = abs(this->targetAngle - this->curAngle);
        }
    }
}

// 轮子停止
void FSUS_Servo::wheelStop(){
    this->protocal->sendWheelStop(this->servoId);
}

// 轮子旋转
void FSUS_Servo::wheelRun(uint8_t is_cw){
    this->protocal->sendWheelKeepMove(this->servoId, is_cw, this->speed);
}

// 轮子旋转特定的时间
void FSUS_Servo::wheelRunNTime(uint8_t is_cw, uint16_t time_ms){
    this->protocal->sendWheelMoveTime(this->servoId, is_cw, this->speed, time_ms);
}

// 旋转圈数
void FSUS_Servo::wheelRunNCircle(uint8_t is_cw, uint16_t circle_num){
    this->protocal->sendWheelMoveNCircle(this->servoId, is_cw, this->speed, circle_num);
}

// 检查轮子是否停止
bool FSUS_Servo::wheelIsStop(){
    // TODO
    return false;
}

// 轮子是否在旋转状态查询
