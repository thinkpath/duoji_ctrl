/*********************************
 * 用来处理舵机的底层通信协议
 *********************************/

#include "FashionStar_UartServoProtocal.h"

typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

typedef unsigned char FSUS_SERVO_ID_T; //舵机ID的格式
typedef char* FSUS_SERVO_NAME_T; //舵机的名称
typedef unsigned int FSUS_INTERVAL_T; //时间的格式
typedef int FSUS_SERVO_POSITION_T; //舵机位置ADC采样数值格式
typedef float FSUS_SERVO_ANGLE_T; //舵机角度的格式 [-135°, 135°]
typedef float FSUS_SERVO_SPEED_T; //舵机转速的格式
typedef unsigned int FSUS_POWER_T; //功率的格式 
typedef unsigned int FSUS_HEADER_T; //帧头
typedef unsigned char FSUS_PACKAGE_SIZE_T; //Package的长度格式
typedef unsigned char FSUS_CHECKSUM_T; // 校验和的数据类型
//typedef byte FSUS_CHECKSUM_T; // 校验和的数据类型


// FSUS状态码
#define FSUS_STATUS uint8_t
#define FSUS_STATUS_SUCCESS 0 // 设置/读取成功
#define FSUS_STATUS_FAIL 1 // 设置/读取失败

#define FSUS_STATUS_WRONG_RESPONSE_HEADER 3 // 响应头不对
#define FSUS_STATUS_UNKOWN_CMD_ID 4 // 未知的控制指令
#define FSUS_STATUS_SIZE_TOO_BIG 5 // 参数的size大于FSUS_PACK_RESPONSE_MAX_SIZE里面的限制
#define FSUS_STATUS_CHECKSUM_ERROR 6 // 校验和错误
#define FSUS_STATUS_ID_NOT_MATCH 7 // 请求的舵机ID跟反馈回来的舵机ID不匹配
#define FSUS_STATUS_TIMEOUT 8 // 等待超时

// 请求数据帧的结构体
typedef struct{
    uint16_t header; // 请求头
    uint8_t cmdId; // 指令ID号
    uint8_t content_size; // 包的长度
    uint8_t content[FSUS_PACK_RESPONSE_MAX_SIZE]; // 包的内容
    FSUS_CHECKSUM_T checksum; // 校验和
    // 接收
    FSUS_STATUS recv_status; // Package的状态码
    uint8_t recv_cnt; // 接收字符的计数
    uint8_t recv_buffer[FSUS_PACK_RESPONSE_MAX_SIZE]; // 临时,测试用
} FSUS_PACKAGE_T;



// 初始化串口
#define BAUDRATE 115200   // 设置默认的波特率
#define SERVO_ID 0        //舵机ID号

#define FSUS_PACK_REQUEST_HEADER		0x4c12
#define FSUS_PACK_RESPONSE_HEADER		0x1c05

/************************************
 * 发送数据包
 * *********************************/

FSUS_PACKAGE_T requestPack;

// 加工并发送请求数据包
void sendPack() {
    // 数据加工
    requestPack.header = FSUS_PACK_REQUEST_HEADER; // 数据头
    // 计算校验码
    requestPack.checksum = calcPackChecksum(&requestPack);
    // 通过串口发送出去
    serial_write(requestPack.header & 0xFF);
    serial_write(requestPack.header >> 8);
    serial_write(requestPack.cmdId);
    serial_write(requestPack.content_size);
    for(int i=0; i<requestPack.content_size; i++) {
        serial_write(requestPack.content[i]);
    }
    serial_write(requestPack.checksum);
}

// 清空缓冲区
void emptyCache();


// 写串口数据
void serial_write(unsigned char send_data); //TODO: 需要跟实际的串口接口匹配

//计算CRC校验码
FSUS_CHECKSUM_T calcPackChecksum(const FSUS_PACKAGE_T *package){
    // uint16_t checksum = 0;
    int checksum = 0;
    checksum += (package.header & 0xFF);
    checksum += (package.header >> 8);
    checksum += package.cmdId;
    checksum += package.content_size;
    for(int i=0; i<package.content_size; i++){
        checksum += package.content[i];
    }
    
    return (FSUS_CHECKSUM_T)(checksum%256);
}

/*******************************
 * 接收数据包
 * *****************************/

FSUS_PACKAGE_T responsePack;

// 初始化响应数据包的数据结构
void initResponsePack(){
    // 初始化响应包的数据
    //发送数据的缓冲区
    
    responsePack.header = 0;
    responsePack.cmdId = 0;
    responsePack.content_size = 0;
    responsePack.checksum = 0;
    responsePack.recv_cnt = 0;
}

// 接收响应包 
FSUS_STATUS recvPack(){
    unsigned long start_time = millis();  //TODO: 开启计时器
    
    // responsePack.recv_status = 0; //重置标志位
    responsePack.recv_cnt = 0; // 数据帧接收标志位
    
    while(TRUE){ //TODO: 替换成中断更好，避免一直循环等待
        // 超时判断
        if((millis() - start_time) > FSUS_TIMEOUT_MS){ //TODO: 与系统匹配  
            return FSUS_STATUS_TIMEOUT;
        }
        // 等待有字节读入
        if (serial_available==0){ //TODO: serial_available待接口定义
            continue;
        }
        unsigned char curByte = serial_read();
        unsigned char curIdx = responsePack.recv_cnt;
        responsePack.recv_buffer[curIdx] = curByte; // 接收一个字节
        responsePack.recv_cnt+=1; // 计数自增
        
        // 数据接收是否完成
        if(curIdx==1){
            // 校验帧头
            responsePack.header = responsePack.recv_buffer[curIdx-1] | curByte << 8; //拼接
            if(responsePack.header!=FSUS_PACK_RESPONSE_HEADER){
                return FSUS_STATUS_WRONG_RESPONSE_HEADER;
            }
        }else if (curIdx==2){
            // 载入cmdId
            responsePack.cmdId = curByte;
            // 检查cmdId是否满足指令范围
            if(responsePack.cmdId > 10){
                return FSUS_STATUS_UNKOWN_CMD_ID;
            }
        }else if (curIdx==3){
            // 载入Size
            responsePack.content_size = curByte;
            // 判断size是否合法
            if(responsePack.content_size > FSUS_PACK_RESPONSE_MAX_SIZE){
                return FSUS_STATUS_SIZE_TOO_BIG;
            }
        }else if (curIdx < 4+responsePack.content_size){
            // 填充内容
            responsePack.content[curIdx-4] = curByte;
        }else{
            // 接收校验合
            responsePack.checksum = curByte;
            // 检查校验和是否匹配
            FSUS_CHECKSUM_T checksum = calcPackChecksum(&responsePack); //TODO:
            // if (responsePack.cmdId == FSUS_CMD_QUERY_ANGLE){
            //     checksum -= 0x03;// TODO Delete 不知道为什么要这样
            // }
            
            if (checksum != responsePack.checksum){
                return FSUS_STATUS_CHECKSUM_ERROR;
            }else{
                return FSUS_STATUS_SUCCESS;
            }
        }
    }
}

//TODO: 串口读命令，返回byte
unsigned char serial_read(); 

// 发送PING的请求包
void sendPing(unsigned char servoId){
    requestPack.cmdId = FSUS_CMD_PING; //0x01
    requestPack.content_size = 1;
    requestPack.content[0] = servoId;
    sendPack(); // 发送包
}

// 接收PING的响应包
FSUS_STATUS recvPing(unsigned char *servoId, bool *isOnline){
    // 接收数据帧
    FSUS_STATUS status = recvPack();
    *servoId = responsePack.content[0]; // 提取舵机ID
    *isOnline = (status == FSUS_STATUS_SUCCESS);
    responsePack.recv_status = status;
    return status;
}


// 发送旋转的请求包
void sendSetAngle(unsigned char servoId, float angle,unsigned int interval,unsigned int power){
    requestPack.cmdId = FSUS_CMD_SET_ANGLE; // 指令ID 0x08
    requestPack.content_size = 7; // 内容长度
    requestPack.content[0]=servoId; //舵机ID
    int angle_int = angle * 10; //舵机的角度
    requestPack.content[1] = angle_int & 0xFF;
    requestPack.content[2] = angle_int >> 8;
    requestPack.content[3] = interval & 0xFF; //周期
    requestPack.content[4] = interval >> 8;
    requestPack.content[5] = power & 0xFF; //功率
    requestPack.content[6] = power >> 8;
    sendPack();
}


// 发送阻尼模式
void sendDammping(unsigned char servoId, unsigned int power){
    requestPack.cmdId = FSUS_CMD_DAMPING;//指令ID 0x09
    requestPack.content_size = 3;
    requestPack.content[0] = servoId;
    requestPack.content[1] = power & 0xFF;
    requestPack.content[2] = power >> 8;
    sendPack();
}

// 发送舵机角度查询指令
void sendQueryAngle(unsigned char servoId){
    requestPack.cmdId = FSUS_CMD_QUERY_ANGLE;//指令ID 0x0a
    requestPack.content_size = 1;
    requestPack.content[0] = servoId;
    sendPack();
}

// 接收角度查询结果
FSUS_STATUS recvQueryAngle(unsigned char *servoId, float *angle){
    unsigned char status = recvPack();
    int angleVal;
    unsigned char* angleValPtr = (unsigned char*)&angleVal;
    
    // angleVal = responsePack.content[1] + responsePack.content[2] << 8;
    // if(status == FSUS_STATUS_SUCCESS){
    // 偶尔会出现校验和错误的情况, 临时允许
    if(status == FSUS_STATUS_SUCCESS || status==FSUS_STATUS_CHECKSUM_ERROR){
        (*servoId) = responsePack.content[0];
        
        angleValPtr[0] = responsePack.content[1];
        angleValPtr[1] = responsePack.content[2];
        (*angle) = 0.1*angleVal;
        // (*angle) = 0.1 * (int)(responsePack.content[1] | responsePack.content[2]<< 8);
    }
    responsePack.recv_status = status;
    return status;
}

// 轮式控制模式
void sendWheelMove(unsigned char servoId, unsigned char method, unsigned int speed, unsigned short value){
    requestPack.cmdId = FSUS_CMD_WHEEL;
    requestPack.content_size = 6;
    requestPack.content[0] = servoId;
    requestPack.content[1] = method;
    requestPack.content[2] = speed & 0xFF;
    requestPack.content[3] = speed >> 8;
    requestPack.content[4] = value & 0xFF;
    requestPack.content[5] = value >> 8;
    sendPack();
}

// 轮子停止转动
void sendWheelStop(unsigned char servoId){
    unsigned char method = 0;
    unsigned short  speed = 0;
	unsigned short  value = 0;
    sendWheelMove(servoId, method, speed, value);
}

// 轮子持续旋转
void sendWheelKeepMove(unsigned char servoId, unsigned char is_cw, unsigned short speed){
    unsigned char method = 0x01; // 持续旋转
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
	unsigned int value = 0;
    sendWheelMove(servoId, method, speed, value);
}

// 指定旋转的时间
void sendWheelMoveTime(unsigned char servoId, unsigned char is_cw, unsigned short speed, unsigned short nTime){
    unsigned char method = 0x03; // 旋转一段时间
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
    sendWheelMove(servoId, method, speed, nTime);
}

// 轮式模式 旋转特定的圈数
void sendWheelMoveNCircle(unsigned char servoId, unsigned char is_cw, unsigned short speed, unsigned short nCircle){
    unsigned char method = 0x02; // 旋转特定的圈数
	if (is_cw){
		// 顺时针旋转
		method = method | 0x80;
	}
    sendWheelMove(servoId, method, speed, nCircle);
}

