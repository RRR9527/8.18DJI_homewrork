#ifndef PID_HANDLER_H
#define PID_HANDLER_H

#include "motor.h"

float PID_Init(PIDType *pid, float Kp, float Ki, float Kd, uint8_t mode);
float PID_calculate(PIDType *pid);
void PID_Reset(PIDType *pid);

#endif