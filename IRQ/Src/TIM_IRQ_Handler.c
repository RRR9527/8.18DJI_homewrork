#include "TIM_IRQ_Handler.h"
#include "motor.h"

void TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM2){
        DJmotor_Func();
    }
}
