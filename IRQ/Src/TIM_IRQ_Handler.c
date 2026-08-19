#include "TIM_IRQ_Handler.h"

void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM2){
        DJmotor_Func();
        for (uint8_t i = 0; i < USE_DJNUM; i++){
            DJmotor_CurrentTransmit(&DJmotor[i]);
        }
    }
}
