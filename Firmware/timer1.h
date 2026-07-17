/******************************************************************************
 * timer.h
 ******************************************************************************/

#ifndef TIMER_H
#define TIMER_H

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Initialization */
void TIMER_Init(void);

/* Microsecond timer */
uint32_t TIMER_GetMicros(void);

/* Millisecond timer */
uint32_t TIMER_GetMillis(void);

/* Delay */
void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);

/* Scheduler */
void SchedulerTimer_Start(void);

/* Input Capture */
void CKP_Timer_Start(void);

/* Output Compare */
void Injector_Timer_Start(void);
void Ignition_Timer_Start(void);

#endif