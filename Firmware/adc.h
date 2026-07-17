/******************************************************************************
 * adc.h
 ******************************************************************************/

#ifndef ADC_H
#define ADC_H

#include "stm32f4xx_hal.h"

#define ADC_CHANNEL_COUNT    6

typedef enum
{
    ADC_LAMBDA = 0,
    ADC_MAP,
    ADC_TPS,
    ADC_COOLANT,
    ADC_IAT,
    ADC_BATTERY

}ADC_Channel_t;

void ADC_Init(void);

void ADC_Start(void);

void ADC_Stop(void);

uint16_t ADC_ReadRaw(ADC_Channel_t channel);

float ADC_ReadVoltage(ADC_Channel_t channel);

#endif