#ifndef CAN_IRQ_HANDLER_H
#define CAN_IRQ_HANDLER_H

#include "main.h"
#include "motor.h"

extern DJMotor DJmotor[USE_DJNUM];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif