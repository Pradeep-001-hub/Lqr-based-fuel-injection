/******************************************************************************
 * timer.c
 ******************************************************************************/

#include "timer.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/*-----------------------------------------------------------
    Initialize Timers
-----------------------------------------------------------*/
void TIMER_Init(void)
{
    /* Generated using STM32CubeMX */

    HAL_TIM_Base_Start(&htim2);

    HAL_TIM_IC_Start_IT(&htim3,
                        TIM_CHANNEL_1);

    HAL_TIM_OC_Start(&htim4,
                     TIM_CHANNEL_1);
}

/*-----------------------------------------------------------
    Microseconds
-----------------------------------------------------------*/
uint32_t TIMER_GetMicros(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

/*-----------------------------------------------------------
    Milliseconds
-----------------------------------------------------------*/
uint32_t TIMER_GetMillis(void)
{
    return HAL_GetTick();
}

/*-----------------------------------------------------------
    Delay
-----------------------------------------------------------*/
void Delay_us(uint32_t us)
{
    uint32_t start = TIMER_GetMicros();

    while((TIMER_GetMicros()-start)<us);
}

void Delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/*-----------------------------------------------------------
    Scheduler
-----------------------------------------------------------*/
void SchedulerTimer_Start(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

/*-----------------------------------------------------------
    CKP Input Capture
-----------------------------------------------------------*/
void CKP_Timer_Start(void)
{
    HAL_TIM_IC_Start_IT(&htim3,
                        TIM_CHANNEL_1);
}

/*-----------------------------------------------------------
    Injector Timer
-----------------------------------------------------------*/
void Injector_Timer_Start(void)
{
    HAL_TIM_OC_Start_IT(&htim4,
                        TIM_CHANNEL_1);
}

/*-----------------------------------------------------------
    Ignition Timer
-----------------------------------------------------------*/
void Ignition_Timer_Start(void)
{
    HAL_TIM_OC_Start_IT(&htim4,
                        TIM_CHANNEL_2);
}