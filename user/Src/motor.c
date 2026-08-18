#include "motor.h"

void DJmotor_Init(void){
    DJmotorParam dj2006_param;
    DJmotorParam dj3508_param;
    DJmotorLimit limit;
    DJmotorStatus statusFlag;
    DJmotorArgum argum;
    DJmotorError error;

    DJMotor DJmotor[USE_DJNUM];

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
        DJmotor_SetZero(motor);
        motor->statusFlag.IsSetZero = false;
    }

    motor->valPre = motor->valNow;
}


