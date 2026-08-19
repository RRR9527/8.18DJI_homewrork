#ifndef CAN_IRQ_HANDLER_H
#define CAN_IRQ_HANDLER_H

#include "main.h"
#include "motor.h"
#include "can.h"

extern DJMotor DJmotor[USE_DJNUM];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void DJmotor_CurrentTransmit(DJMotorPointer motor);

#endif
