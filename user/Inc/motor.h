#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include <stdbool.h>
#include <stdlib.h>

typedef enum{
    DJ_Disable  = 0,  // 失能状态
    DJ_RPM      = 1,  // 速度模式
    DJ_Position = 2,  // 位置模式
    DJ_Zero     = 3,  // 寻零模式
    DJ_Current  = 4   // 电流/扭矩
} DJmotor_mode_t;  // 模式

typedef struct{
    uint16_t PulsePerRound;  // 电机转完一圈后为8191，再转第二圈时归零
    float Gear_ratio;        // 减速比
    float Reduction_ratio;   // 也是减速比。似乎一个是内置的，一个是外接传动装置的
    uint32_t ParamID;        // CAN通讯接收到的ID
    int16_t CurrentLimit_raw;// 结果（output?）的电流限制，raw（即，单位不是安培）。有正负号的区别！！！
} DJmotorParam;  // 参数

typedef struct{
    volatile int16_t current_raw;  // 直接设置电流
    volatile float angle_deg;      // 输出角度
    volatile int16_t speed_rpm;    // 转速（输出轴/反馈值）
    volatile float current_A;      // 反馈电流
    volatile int16_t PulseRead;    // 读到的原始脉冲数据
    volatile int16_t PulseGap;     // 变化的脉冲值
    volatile int32_t PulseTotal;   // 累积的脉冲值
    volatile int8_t temprature_C;  // 温度
} DJmotorVal;  // 值

typedef struct{
    bool IsSetZero;
    bool Overtimeflag;
    bool StuckFlag;
    bool ZeroFlag;
} DJmotorStatus;

typedef struct{
    bool RPMLimitFlag;     
    bool PosAngleLimitFlag;
    bool PosRPMFlag;
    bool CurrentLimitFlag;
    float MaxAngle_deg; 
    float MinAngle_deg;
    int16_t SpeedRPMLimit;  // 速度模式下的限制速度
    int32_t PosRPMLimit;    // 位置模式下的限制速度
    int16_t ZeroRPMLimit;   // 寻零模式下的限制速度
    int16_t ZeroCurrentLimit_raw;  // 寻零模式下的最大电流
    bool IsLooseStuck;
} DJmotorLimit;  // 前面的bool值是实行限制的开关，后面是软件上进行限制的具体的值

typedef struct{
    /*其实我也不知这些数据有多大，有无符号。先定义成这样，后面报错了再说*/
    int8_t pulselock;
    int8_t zeroCnt;
    int8_t GapCnt;
} DJmotorArgum;  // 我也不知道这些是干什么的 

typedef struct{
    /*同上一个结构体。我也不知道这些数据有多大*/
    int8_t lastRxTime;
    int8_t stuckCount;
    int16_t timeoutCount;
} DJmotorError;  // 似乎是记录错误的。但我也不知道具体干嘛的

typedef struct{
    uint8_t mode;  // 位置式or增量式
    float Kp;
    float Ki;
    float Kd;
    float err[2];  // 什么含义取决于什么模式
    float SetVal;  // 输入量，设定值
    float CurVal;  // 当前量
    float output;  // 计算结果
} PIDType;

typedef struct{
    uint8_t ID;
    volatile bool Begin;
    volatile DJmotor_mode_t MODE_Set;  // 设定状态
    volatile DJmotor_mode_t MODE_Cur;  // 实际运行状态

    DJmotorParam param;
    DJmotorVal valSet;  // 输出轴的数据rpm
    DJmotorVal valNow;  // 反馈的数据
    DJmotorVal valPre;
    DJmotorStatus statusFlag;  /*待会儿记得回来填坑*/
    DJmotorLimit limit;
    DJmotorArgum argum;
    DJmotorError error;
    PIDType posPID;
    PIDType velPID;
} DJMotor, *DJMotorPointer;  // 后者是DJMotor的指针。相当于单独给指针的类型定义了一个名称

#define PIDINC 1U  // 增量式PID
#define PIDPOS 2U  // 位置式PID
#define M2006_RATIO 36U
#define M3508_RATIO 19U
#define USE_DJNUM 8U  // 使用的电机的数量（假设一共八个2006和3508各4个）
#define M2006_NUM 4U  // 2006型号
#define M3508_NUM 4U  // 3508型号

#define GetSign(x) ((x > 0) - (x < 0))

float PID_Init(PIDType *pid, float Kp, float Ki, float Kd, uint8_t mode);
void DJmotor_Init(void);

#if USE_DJ
    extern DJMotor DJmotor[USE_DJNUM];

    void DJmotor_Init(void);
    void DJmotor_Func(void);
    void DJmotor_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data);
    void DJmotor_PID_Reload(DJMotorPointer motor, DJmotorPID pid_reload);

#endif

#endif
