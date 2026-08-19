#include "CAN_IRQ_Handler.h"

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK){
        if (
            RxHeader.IDE != CAN_ID_STD || 
            RxHeader.RTR != CAN_RTR_DATA || 
            RxHeader.StdId < 200U || 
            RxHeader.StdId > 208U
        ){
            return ;
        }

        uint8_t card_id = (uint8_t)(RxHeader.StdId - 200U);

        if (card_id > USE_DJNUM){
            return ;
        }

        DJMotorPointer motor = &DJmotor[card_id - 1U];

        motor->valNow.PulseRead = (int16_t)(((uint16_t)RxData[0] << 8) | RxData[1]);
        motor->valNow.speed_rpm = (int16_t)(((uint16_t)RxData[2] << 8) | RxData[3]);
        motor->valNow.current_raw = (int16_t)(((uint16_t)RxData[4] << 8) | RxData[5]);
        // motor->ID = card_id - 1;

        if (motor->param.Reduction_ratio == M3508_RATIO){
            motor->valNow.temprature_C = (int8_t)RxData[6];
            motor->valNow.current_A = (float)motor->valNow.current_raw * 0.0012207f;
        }else{
            motor->valNow.current_A = (float)motor->valNow.current_raw / 1000.0f;
        }   


        motor->valNow.speed_rpm /= (motor->param.Gear_ratio * motor->param.Reduction_ratio);

        motor->error.lastRxTime = 0;
        DJmotor_AngleCalculate(motor);
    }
}

void DJmotor_CurrentTransmit(DJMotorPointer motor, CAN_HandleTypeDef *hcan){
    // 小困惑：motor->ID究竟是什么时候被赋值的？还是说主循环会遍历一遍所有的ID？
    static uint8_t Tx_Data[8] = {0};
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t tag = 0;
    uint32_t TxMailbox;

    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.ExtId = 0;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (motor->ID <= 4U){
        TxHeader.StdId = 0x200U;
        tag = (uint8_t)((motor->ID - 1U) * 2U);
    }else{
        TxHeader.StdId = 0x1FFU;
        tag = (uint8_t)((motor->ID - 5U) * 2U);
    }

    Tx_Data[tag] = (uint8_t)(motor->valSet.current_raw >> 8);
    Tx_Data[tag + 1] = (uint8_t)(motor->valSet.current_raw & 0xFF);

    if (motor->ID == 4U || motor->ID == 8U){
        HAL_CAN_AddTxMessage(&hcan, &TxHeader, Tx_Data, &TxMailbox);
    }
}
