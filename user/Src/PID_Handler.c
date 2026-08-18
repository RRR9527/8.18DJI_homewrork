#include "PID_Handler.h"

float PID_Init(PIDType *pid, float Kp, float Ki, float Kd, uint8_t mode){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->mode = mode;
    pid->CurVal = 0;
    pid->SetVal = 0;
}

float PID_calculate(PIDType *pid){
    pid->err[0] = pid->SetVal - pid->CurVal;  // 计算本次偏差

    switch (pid->mode){
        case PIDINC: {  // 增量式算法
            // 计算PID数值
            pid->output = pid->Kp * (pid->err[0] - pid->err[1]) +
                          pid->Ki * pid->err[0] +
                          pid->Kd * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);

            // 数据往后平移
            pid->err[2] = pid->err[1];
            pid->err[1] = pid->err[0];  // 相当于，索引越小数据越新，索引大于2的直接被淘汰了
            break;
        }

        case PIDPOS: {  // 位置式算法
            // 注意，此时err[0], err[1], err[2]的含义变化了！！！
            // err[0]和err[1]表示最近两次的偏差，err[2]表示积分值！！！
            pid->err[2] = 0.5*pid->err[0] + 0.5*pid->err[2];
            // 相当于滤波器。越古老的数据前面的系数越小，影响越小。一定程度上避免了积分饱和
            pid->output = pid->Kp * pid->err[0] +
                          pid->Ki * pid->err[2] +
                          pid->Kd * (pid->err[0] - pid->err[1]);
            // d_t被包含在Ki和Kd里面了，不用单独再乘了
            pid->err[1] = pid->err[0];
            break;
        }
    }

    return pid->output;
}