#include "motor.h"

DJMotor DJmotor[USE_DJNUM];

void DJmotor_Init(void){
    DJmotorParam dj2006_param;
    DJmotorParam dj3508_param;
    DJmotorLimit limit;
    DJmotorStatus statusFlag;
    DJmotorArgum argum;
    DJmotorError error;


    dj2006_param.ParamID = 0x1FFU;
    dj2006_param.Gear_ratio = 1.0f;
    dj2006_param.Reduction_ratio = M2006_RATIO;
    dj2006_param.PulsePerRound = 8191U;
    dj2006_param.CurrentLimit_raw = 4500;

    dj3508_param.ParamID = 0x200U;
    dj3508_param.Gear_ratio = 1.0f;
    dj3508_param.Reduction_ratio = M3508_RATIO;
    dj3508_param.PulsePerRound = 8191U;
    dj3508_param.CurrentLimit_raw = 10000;

    limit.CurrentLimitFlag = true;
    limit.IsLooseStuck = false;

    limit.MaxAngle_deg = 270.0f;
    limit.MinAngle_deg = -270.0f;
    limit.PosAngleLimitFlag = false;
    limit.PosRPMFlag = true;
    limit.PosRPMLimit = 8000;

    limit.RPMLimitFlag = false;
    limit.SpeedRPMLimit = 10000;
    limit.ZeroCurrentLimit_raw = 3000;
    limit.ZeroRPMLimit = 500;

    statusFlag.IsSetZero = true;
    statusFlag.Overtimeflag = false;
    statusFlag.StuckFlag = false;
    statusFlag.ZeroFlag = false;

    argum.pulselock = 0;
    argum.zeroCnt = 0;
    argum.GapCnt = 0;

    error.lastRxTime = 0;
    error.stuckCount = 0;
    error.timeoutCount = 0;

    for (uint32_t i = 0; i < USE_DJNUM; i++){
        DJmotor[i].Begin = false;
        DJmotor[i].MODE_Set = DJ_Disable;
        DJmotor[i].statusFlag = statusFlag;
        DJmotor[i].limit = limit;
        DJmotor[i].argum = argum;
        DJmotor[i].error = error;
        DJmotor[i].valSet.current_raw = 0;
        DJmotor[i].valSet.angle_deg = 0.0f;
        DJmotor[i].valSet.speed_rpm = 0;
        DJmotor[i].valSet.PulseTotal = 0;
        DJmotor[i].valNow.PulseTotal = 0;
        DJmotor[i].valPre.PulseRead = 0;
    }

    for (uint32_t i = 0; i < M2006_NUM; i++){
        DJmotor[i].ID = (uint8_t)(i + 1U);
        DJmotor[i].param = dj2006_param;
    }

    for (uint32_t i = 0; i < M3508_NUM; i++){
        DJmotor[i + M2006_NUM].ID = (uint8_t)(i + 1U + M2006_NUM);
        DJmotor[i + M3508_NUM].param = dj3508_param;
    }

    for (uint32_t i = 0; i < USE_DJNUM; i++){
        PID_Init(&DJmotor[i].posPID, 1.0f, 1.0f, 0.0f, PIDPOS);
        PID_Init(&DJmotor[i].velPID, 1.0f, 1.0f, 1.0f, PIDINC);
    }
}

void DJmotor_AngleCalculate(DJMotorPointer motor){  // 注意，这里传入的是一个指针！！！
    motor->valNow.PulseGap = (int16_t)(motor->valNow.PulseRead - motor->valPre.PulseRead);

    if (abs(motor->valNow.PulseGap) > 4096){
        motor->valNow.PulseGap = (int16_t)(
            motor->valNow.PulseGap - 
            GetSign(motor->valNow.PulseGap) * (int32_t)motor->param.PulsePerRound
        );
    }

    motor->valNow.PulseTotal += motor->valNow.PulseGap;
    motor->valNow.angle_deg = (float)motor->valNow.PulseTotal * 360.0f / (
        (float)motor->param.PulsePerRound * 
        motor->param.Gear_ratio * 
        motor->param.Reduction_ratio
    );

    if (motor->Begin){
        motor->argum.pulselock = motor->valNow.PulseTotal;
    }

    if (motor->statusFlag.IsSetZero){
        DJmotor_SetZero(motor);  // 记得填坑
        motor->statusFlag.IsSetZero = false;
    }

    motor->valPre = motor->valNow;
}

void DJmotor_Func(void){
    for (uint32_t i = 0; i < USE_DJNUM; i++){
        if (DJmotor[i].Begin){
            DJmotor_SwitchMode(&DJmotor[i]);

            switch (DJmotor[i].MODE_Cur){
                case DJ_Disable:{
                    DJmotor[i].valSet.current_raw = 0;
                    DJmotor_CurrentTransmit(&DJmotor[i]);
                    continue;
                    break;
                }

                case DJ_RPM:{
                    DJmotor_SpeedMode(&DJmotor[i]);
                    break;
                }

                case DJ_Position:{
                    DJmotor_PositionMode(&DJmotor[i]);
                    break;
                }

                case DJ_Zero:{
                    DJmotor_ZeroMode(&DJmotor[i]);
                    break;
                }

                case DJ_Current:{
                    ClampPeak(DJmotor[i].valSet.current_raw, DJmotor[i].param.CurrentLimit_raw);
                    break;
                }

                defualt :{
                    break;
                }
            }
        }else{
            DJmotor[i].valSet.current_raw = 0;
        }

        DJmotor_CurrentTransmit(&DJmotor[i]);
    }
}

static void DJmotor_SwitchMode(DJMotorPointer motor){
    if (motor->MODE_Set != motor->MODE_Cur){
        motor->MODE_Cur = motor->MODE_Set;
        motor->valSet.current_raw = 0;
        motor->valSet.speed_rpm = 0;
        motor->valSet.angle_deg = motor->valNow.angle_deg;
        // 清除历史残值
        PID_Reset(&motor->posPID);
        PID_Reset(&motor->velPID);
        motor->statusFlag.ZeroFlag = false;
        motor->statusFlag.Overtimeflag = false;
        motor->statusFlag.StuckFlag = false;
    }
}

void DJmotor_SpeedMode(DJMotorPointer motor){
    motor->velPID.SetVal = (float)motor->valSet.speed_rpm * 
                            motor->param.Gear_ratio * 
                            motor->param.Reduction_ratio;
    motor->velPID.CurVal = (float)motor->valNow.speed_rpm * 
                            motor->param.Gear_ratio * 
                            motor->param.Reduction_ratio;

    if (motor->limit.RPMLimitFlag){
        ClampPeak(motor->velPID.SetVal, motor->limit.SpeedRPMLimit);
    }

    motor->valSet.current_raw += PID_calculate(&motor->velPID);
    ClampPeak(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
}

void DJmotor_PositionMode(DJMotorPointer motor){
    motor->valSet.PulseTotal = (int32_t)(
        motor->valSet.angle_deg * 
        motor->param.Gear_ratio * 
        (float)motor->param.PulsePerRound / 360.0f
    );

    motor->posPID.SetVal = (float)motor->valSet.PulseTotal;
    if (motor->limit.PosAngleLimitFlag){
        const int32_t max_pulse = (int32_t)(
            motor->limit.MaxAngle_deg * 
            (float)motor->param.PulsePerRound * 
            motor->param.Gear_ratio * 
            motor->param.Reduction_ratio / 360.0f
        );

        const int32_t min_pulse = (int32_t)(
            motor->limit.MinAngle_deg * 
            (float)motor->param.PulsePerRound * 
            motor->param.Gear_ratio * 
            motor->param.Reduction_ratio / 360.0f
        );

        motor->posPID.SetVal = Clamp(motor->valSet.PulseTotal, min_pulse, max_pulse);
    }

    motor->posPID.CurVal = (float)motor->valNow.PulseTotal;

    motor->velPID.SetVal = PID_calculate(&motor->posPID);
    motor->velPID.CurVal = (float)motor->valNow.speed_rpm * 
                           motor->param.Gear_ratio * 
                           motor->param.Reduction_ratio;
    
    if (motor->limit.PosRPMFlag){
        ClampPeak(motor->velPID.SetVal, motor->limit.PosRPMLimit);
    }

    motor->valSet.current_raw += PID_calculate(&motor->velPID);
    ClampPeak(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
}

void DJmotor_ZeroMode(DJMotorPointer motor){
    motor->velPID.SetVal = (float)motor->limit.ZeroRPMLimit;
    motor->velPID.CurVal = (float)motor->valNow.speed_rpm;
    motor->valSet.current_raw += PID_calculate(&motor->velPID);
    ClampPeak(motor->valSet.current_raw, motor->limit.ZeroCurrentLimit_raw);

    if (abs(motor->valNow.PulseGap) < Zero_Distance){
        if (motor->argum.zeroCnt ++ > 100U){
            motor->argum.zeroCnt = 0;
            motor->statusFlag.ZeroFlag = true;
            motor->Begin = false;

            PID_Reset(&motor->posPID);
            PID_Reset(&motor->velPID);
            DJmotor_SetZero(motor);  // 记得填坑
        }
    }
}

void DJmotor_SetZero(DJMotorPointer motor){
    motor->valNow.PulseTotal = 0;
}

static void DJmotor_Monitor(DJMotorPointer motor){
    if(motor->valNow.PulseGap < 5 && motor->valNow.current_raw > 3000){
        if (motor->error.stuckCount++ > 500U){
            motor->error.stuckCount = 0;
            motor->statusFlag.StuckFlag = true;
            if (motor->limit.IsLooseStuck){
                motor->MODE_Set = DJ_Disable;
            }
        }
    }else{
        motor->error.stuckCount = 0;
    }

    if (motor->error.lastRxTime++ > 50U){
        if (motor->error.timeoutCount++ > 20U){
            motor->error.timeoutCount = 0;
            motor->MODE_Set = DJ_Disable;
            motor->statusFlag.Overtimeflag = true;
        }
    }
}

int32_t Clamp(int32_t x, int32_t min, int32_t max){
    if (x > max){
        return max;
    }

    if (x < min){
        return min;
    }

    return x;
}
