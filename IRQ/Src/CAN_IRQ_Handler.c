#include "CAN_IRQ_Handler.h"

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if (RxHeader.IDE != CAN_ID_STD || RxHeader.RTR != CAN_RTR_DATA || RxHeader.StdId < 200U || RxHeader.StdId > 208U){
        return ;
    }

    uint8_t card_id = (uint8_t)(RxHeader.StdId - 200U);

    if (card_id > USE_DJNUM){
        return ;
    }

    DJMotorPointer motor = &DJmotor[card_id - 1U];
}