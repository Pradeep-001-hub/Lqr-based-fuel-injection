/******************************************************************************
 * Project : LQR/LQG Fuel Injection ECU
 * File    : config.h
 * Purpose : Global ECU Configuration
 ******************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

/*==========================================================
                    MCU CONFIGURATION
==========================================================*/

#define CPU_CLOCK_HZ              168000000U

/*==========================================================
                  CONTROL LOOP
==========================================================*/

#define CONTROL_FREQ_HZ           100
#define CONTROL_PERIOD_MS         10

/*==========================================================
                  ENGINE PARAMETERS
==========================================================*/

#define ENGINE_CYLINDERS          1
#define ENGINE_DISPLACEMENT_CC    150.0f
#define MAX_RPM                   10000
#define IDLE_RPM                  1500
#define REDLINE_RPM               9500

/*==========================================================
                  STOICHIOMETRY
==========================================================*/

#define STOICH_AFR                14.7f
#define TARGET_AFR_IDLE           14.7f
#define TARGET_AFR_POWER          12.8f
#define TARGET_AFR_CRUISE         15.2f

/*==========================================================
                  LAMBDA SENSOR
==========================================================*/

#define ADC_REFERENCE             3.30f
#define ADC_RESOLUTION            4095.0f

#define LAMBDA_VOLT_MIN           0.50f
#define LAMBDA_VOLT_MAX           4.50f

#define AFR_MIN                   10.0f
#define AFR_MAX                   20.0f

/*==========================================================
                  MAP SENSOR
==========================================================*/

#define MAP_MIN_KPA               20.0f
#define MAP_MAX_KPA               105.0f

/*==========================================================
                  TPS SENSOR
==========================================================*/

#define TPS_MIN_PERCENT           0.0f
#define TPS_MAX_PERCENT           100.0f

/*==========================================================
                  COOLANT
==========================================================*/

#define COOLANT_MIN               -40.0f
#define COOLANT_MAX               130.0f

/*==========================================================
                  BATTERY
==========================================================*/

#define BATTERY_MIN               10.5f
#define BATTERY_MAX               15.0f

/*==========================================================
                  INJECTOR
==========================================================*/

#define INJECTOR_FLOW_CC_MIN      220.0f

#define INJECTOR_DEADTIME_US      900

#define INJECTOR_MIN_US           800
#define INJECTOR_MAX_US           6000

/*==========================================================
                  LQI GAINS
==========================================================*/

#define LQI_K1                    6.5952f
#define LQI_K2                    0.8017f
#define LQI_KI                   -3.1623f

/*==========================================================
                  KALMAN FILTER
==========================================================*/

#define KF_L1                     0.0516f
#define KF_L2                     0.0199f

/*==========================================================
                  SAFETY
==========================================================*/

#define WATCHDOG_TIMEOUT_MS       100

#define MAX_ENGINE_TEMP           120.0f

#define MIN_OIL_PRESSURE          20.0f

/*==========================================================
                  CAN
==========================================================*/

#define CAN_ID_ECU_STATUS         0x100
#define CAN_ID_ENGINE_DATA        0x101
#define CAN_ID_DIAGNOSTICS        0x102

/*==========================================================
                  UART
==========================================================*/

#define UART_BAUDRATE             115200

/*==========================================================
                  LOGGING
==========================================================*/

#define LOG_PERIOD_MS             100

/*==========================================================
                  DIAGNOSTICS
==========================================================*/

#define DTC_SENSOR_FAILURE        0x0001
#define DTC_INJECTOR_FAILURE      0x0002
#define DTC_OVERHEAT              0x0003
#define DTC_LOW_BATTERY           0x0004

#endif