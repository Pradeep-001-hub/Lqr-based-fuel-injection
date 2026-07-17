/******************************************************************************
 * ckp.c
 ******************************************************************************/

#include "ckp.h"

static volatile uint32_t previousCapture = 0;
static volatile uint32_t toothPeriod = 0;

static volatile uint16_t toothCount = 0;

static volatile float engineRPM = 0.0f;

static volatile float crankAngle = 0.0f;

static volatile uint8_t synchronized = 0;

void CKP_Init(void)
{
    previousCapture = 0;

    toothCount = 0;

    engineRPM = 0;

    crankAngle = 0;

    synchronized = 0;
}

/*---------------------------------------------------------
 Input Capture Callback
---------------------------------------------------------*/

void CKP_InputCaptureCallback(uint32_t captureValue)
{
    if(previousCapture == 0)
    {
        previousCapture = captureValue;
        return;
    }

    toothPeriod = captureValue - previousCapture;

    previousCapture = captureValue;

    if(toothPeriod == 0)
        return;

    float frequency = 1000000.0f / toothPeriod;

    engineRPM = (frequency * 60.0f) / CKP_EFFECTIVE_TEETH;

    /* Missing tooth detection */

    static uint32_t previousPeriod = 0;

    if(previousPeriod != 0)
    {
        if(toothPeriod > (previousPeriod * 1.8f))
        {
            toothCount = 0;

            synchronized = 1;
        }
        else
        {
            toothCount++;

            if(toothCount >= CKP_EFFECTIVE_TEETH)
                toothCount = 0;
        }
    }

    previousPeriod = toothPeriod;

    crankAngle = toothCount * 6.0f;
}

/*---------------------------------------------------------*/

float CKP_GetRPM(void)
{
    return engineRPM;
}

/*---------------------------------------------------------*/

float CKP_GetCrankAngle(void)
{
    return crankAngle;
}

/*---------------------------------------------------------*/

uint16_t CKP_GetToothNumber(void)
{
    return toothCount;
}

/*---------------------------------------------------------*/

uint8_t CKP_IsSynchronized(void)
{
    return synchronized;
}