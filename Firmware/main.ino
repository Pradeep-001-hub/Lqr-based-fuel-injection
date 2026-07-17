/******************************************************************************
 * Project : LQR/LQG Fuel Injection ECU
 * File    : main.c
 * Author  : Pradeep
 * Target  : STM32F407
 ******************************************************************************/

#include "main.h"
#include "config.h"
#include "gpio.h"
#include "adc.h"
#include "timer.h"
#include "uart.h"
#include "can.h"
#include "ckp.h"
#include "cmp.h"
#include "lambda.h"
#include "map.h"
#include "tps.h"
#include "fuel.h"
#include "kalman.h"
#include "lqi.h"
#include "injector.h"
#include "diagnostics.h"
#include "scheduler.h"
#include "watchdog.h"

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    GPIO_Init();

    ADC_Init();

    TIMER_Init();

    UART_Init();

    CAN_Init();

    CKP_Init();

    CMP_Init();

    Lambda_Init();

    MAP_Init();

    TPS_Init();

    Fuel_Init();

    Kalman_Init();

    LQI_Init();

    Injector_Init();

    Diagnostics_Init();

    Scheduler_Init();

    Watchdog_Init();

    while(1)
    {
        Scheduler_Run();

        Watchdog_Refresh();
    }
}