/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <stdlib.h>
#define  ARM_MATH_CM4
#include "arm_math.h"
#include "RodogTypes.h"
#include "lsm6dso32x_reg.h"
#include "kinematics.h"
#include "CurveGen.h"
#include "ov2640.h"
#include <HMC5883L.h>
#include "Madgwick_AHRS.h"
#include "MahonyAHRS.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOE
#define MUX_S0_Pin GPIO_PIN_3
#define MUX_S0_GPIO_Port GPIOE
#define MUX_S1_Pin GPIO_PIN_13
#define MUX_S1_GPIO_Port GPIOC
#define MUX_S2_Pin GPIO_PIN_14
#define MUX_S2_GPIO_Port GPIOC
#define MUX_S3_Pin GPIO_PIN_15
#define MUX_S3_GPIO_Port GPIOC
#define POT12_Pin GPIO_PIN_0
#define POT12_GPIO_Port GPIOC
#define POT3_Pin GPIO_PIN_1
#define POT3_GPIO_Port GPIOC
#define POT6_Pin GPIO_PIN_2
#define POT6_GPIO_Port GPIOC
#define POT9_Pin GPIO_PIN_3
#define POT9_GPIO_Port GPIOC
#define POT11_Pin GPIO_PIN_0
#define POT11_GPIO_Port GPIOA
#define PWM12_Pin GPIO_PIN_1
#define PWM12_GPIO_Port GPIOA
#define CURR_FB_Pin GPIO_PIN_2
#define CURR_FB_GPIO_Port GPIOA
#define POT2_Pin GPIO_PIN_3
#define POT2_GPIO_Port GPIOA
#define POT8_Pin GPIO_PIN_5
#define POT8_GPIO_Port GPIOA
#define POT5_Pin GPIO_PIN_7
#define POT5_GPIO_Port GPIOA
#define POT10_Pin GPIO_PIN_4
#define POT10_GPIO_Port GPIOC
#define POT1_Pin GPIO_PIN_5
#define POT1_GPIO_Port GPIOC
#define POT7_Pin GPIO_PIN_0
#define POT7_GPIO_Port GPIOB
#define POT4_Pin GPIO_PIN_1
#define POT4_GPIO_Port GPIOB
#define PWM10_Pin GPIO_PIN_9
#define PWM10_GPIO_Port GPIOE
#define PWM9_Pin GPIO_PIN_11
#define PWM9_GPIO_Port GPIOE
#define PWM8_Pin GPIO_PIN_13
#define PWM8_GPIO_Port GPIOE
#define PWM11_Pin GPIO_PIN_14
#define PWM11_GPIO_Port GPIOE
#define US_TRIG_Pin GPIO_PIN_12
#define US_TRIG_GPIO_Port GPIOB
#define US_ECHO3_Pin GPIO_PIN_13
#define US_ECHO3_GPIO_Port GPIOB
#define US_ECHO3_EXTI_IRQn EXTI15_10_IRQn
#define US_ECHO2_Pin GPIO_PIN_14
#define US_ECHO2_GPIO_Port GPIOB
#define US_ECHO2_EXTI_IRQn EXTI15_10_IRQn
#define US_ECHO1_Pin GPIO_PIN_15
#define US_ECHO1_GPIO_Port GPIOB
#define US_ECHO1_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_EN_Pin GPIO_PIN_10
#define MOTOR_EN_GPIO_Port GPIOD
#define BTS_DEN_Pin GPIO_PIN_11
#define BTS_DEN_GPIO_Port GPIOD
#define PWM1_Pin GPIO_PIN_12
#define PWM1_GPIO_Port GPIOD
#define PWM4_Pin GPIO_PIN_13
#define PWM4_GPIO_Port GPIOD
#define PWM7_Pin GPIO_PIN_14
#define PWM7_GPIO_Port GPIOD
#define PWM3_Pin GPIO_PIN_15
#define PWM3_GPIO_Port GPIOD
#define PWM6_Pin GPIO_PIN_6
#define PWM6_GPIO_Port GPIOC
#define PWM5_Pin GPIO_PIN_7
#define PWM5_GPIO_Port GPIOC
#define CAR_DET_Pin GPIO_PIN_8
#define CAR_DET_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_15
#define PWM2_GPIO_Port GPIOA
#define FLASH_LED_Pin GPIO_PIN_0
#define FLASH_LED_GPIO_Port GPIOD
#define VBUS_DET_Pin GPIO_PIN_4
#define VBUS_DET_GPIO_Port GPIOD
#define VBUS_DET_EXTI_IRQn EXTI4_IRQn
#define ESP_RX_Pin GPIO_PIN_5
#define ESP_RX_GPIO_Port GPIOD
#define ESP_TX_Pin GPIO_PIN_6
#define ESP_TX_GPIO_Port GPIOD
#define MAGN_INT_Pin GPIO_PIN_7
#define MAGN_INT_GPIO_Port GPIOD
#define MAGN_INT_EXTI_IRQn EXTI9_5_IRQn
#define LSM_IT2_Pin GPIO_PIN_4
#define LSM_IT2_GPIO_Port GPIOB
#define LSM_IT1_Pin GPIO_PIN_5
#define LSM_IT1_GPIO_Port GPIOB
#define LSM_IT1_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SDA_Pin GPIO_PIN_9
/*
    ADC2 channels:
	 	 channel 5  : index 0 : POT8
	 	 channel 7  : index 1 : POT5
	 	 channel 8  : index 2 : POT7
	 	 channel 9  : index 3 : POT4
	 	 channel 14 : index 4 : POT10
	 	 channel 15 : index 5 : POT1

	ADC3 channels: takes an N_POT_CHANNELS/2 + 1 offset in the adcPOS array
	   	 channel 0  : index 0 : POT11
	 	 channel 3  : index 1 : POT2
	 	 channel 10 : index 2 : POT12
	 	 channel 11 : index 3 : POT3
	 	 channel 12 : index 4 : POT6
	 	 channel 13 : index 5 : POT9

	TIMER1 channels:
		channel 1 : PWM10
		channel 2 : PWM9
		channel 3 : PWM8
		channel 4 : PWM11

	TIMER2 channels:
	  	channel 1 : PWM2
	  	channel 2 : PWM12
	  	channel 3 : n/a
	  	channel 4 : n/a

	TIMER3 channels:
	  	channel 1 : PMW6
	  	channel 2 : PWM5
	  	channel 3 : n/a
	  	channel 4 : n/a

	TIMER4 channels:
	  	channel 1 : PWM1
	  	channel 2 : PWM4
	  	channel 3 : PWM7
	  	channel 4 : PWM3	*/

#define SWD_DEBUG
#ifdef SWD_DEBUG
#define swd_print(k , c) printf(k , c)
#define swd_prints(s) printf(s)
#else
#define swd_print(x,c)
#define swd_prints(s)
#endif

#define GRAVITATIONAL_kG 0.00980665f
#define kRAD 0.000017453292519943295f

#define N_JOINTS 12
#define N_MUX_CHANNELS 14
#define ADC_POT_BUF_MULT 1
#define N_POT_CHANNELS 6

#define N_POT_BUF ADC_POT_BUF_MULT * N_POT_CHANNELS
#define ULTRA_SOUND_PULSE_MS 10
#define CURR_FB_BUF 6

//duty cycle = ccr/arr * 100
#define PWM_ARR 59999.0f
#define MIN_CCR 1500.0f
#define MAX_CCR 7535.4
#define MIN_RAD 0.0f
#define MAX_RAD PI

#define DEFAULT_MAX_POS_ADC 2730
#define DEFAULT_MIN_POS_ADC 0

//handles
#define SENSORS_I2C hi2c1
#define DCMI_I2C hi2c2

//Mux channels
#define BATT_LEVEL 12
#define CURRFB_MAIN 13
#define CURRFB_J0 4
#define CURRFB_J1 2
#define CURRFB_J2 1
#define CURRFB_J3 3
#define CURRFB_J4 8
#define CURRFB_J5 6
#define CURRFB_J6 10
#define CURRFB_J7 7
#define CURRFB_J8 5
#define CURRFB_J9 9
#define CURRFB_J10 11
#define CURRFB_J11 0

//timers
#define TIM_J0  TIM4
#define TIM_J1  TIM2
#define TIM_J2  TIM4

#define TIM_J3  TIM3
#define TIM_J4  TIM3
#define TIM_J5  TIM4

#define TIM_J6  TIM1
#define TIM_J7  TIM1
#define TIM_J8  TIM4

#define TIM_J9  TIM2
#define TIM_J10 TIM1
#define TIM_J11 TIM1

//timer channels
#define TIM_CH_J0  4
#define TIM_CH_J1  1
#define TIM_CH_J2  1

#define TIM_CH_J3  1
#define TIM_CH_J4  2
#define TIM_CH_J5  2

#define TIM_CH_J6  2
#define TIM_CH_J7  3
#define TIM_CH_J8  3

#define TIM_CH_J9  2
#define TIM_CH_J10 4
#define TIM_CH_J11 1

//ADC channels
#define ADC_BUF_RANK_J0 3
#define ADC_BUF_RANK_J1 1
#define ADC_BUF_RANK_J2 5

#define ADC_BUF_RANK_J3 4
#define ADC_BUF_RANK_J4 1
#define ADC_BUF_RANK_J5 3

#define ADC_BUF_RANK_J6 5
#define ADC_BUF_RANK_J7 0
#define ADC_BUF_RANK_J8 2

#define ADC_BUF_RANK_J9  2
#define ADC_BUF_RANK_J10 0
#define ADC_BUF_RANK_J11 4

//ADCS
#define ADC_J0 1
#define ADC_J1 1
#define ADC_J2 0

#define ADC_J3 1
#define ADC_J4 0
#define ADC_J5 0

#define ADC_J6 1
#define ADC_J7 0
#define ADC_J8 0

#define ADC_J9	1
#define ADC_J10 1
#define ADC_J11 0

//angular offset of the joint (the initial angle of the servo prior to assembly)
//should ideally be 90 degrees, or Pi/2 rad
#define J0_OFFSET    0.3f
#define J1_OFFSET   -1.4f
#define J2_OFFSET 	0.6f

#define J3_OFFSET   -0.2f
#define J4_OFFSET   -1.5f
#define J5_OFFSET   0.6f

#define J6_OFFSET    0.0f
#define J7_OFFSET   1.3f
#define J8_OFFSET   -0.2f

#define J9_OFFSET     -0.1f
#define J10_OFFSET   -1.3f
#define J11_OFFSET   -0.5f

//timer number
#define TIMNUM_J0  4
#define TIMNUM_J1  2
#define TIMNUM_J2  4
#define TIMNUM_J3  3
#define TIMNUM_J4  8
#define TIMNUM_J5  4
#define TIMNUM_J6  1
#define TIMNUM_J7  1
#define TIMNUM_J8  4
#define TIMNUM_J9  2
#define TIMNUM_J10 1
#define TIMNUM_J11 1

//minimum ADC position of each joint
#define MIN_ADC_J0  1400
#define MIN_ADC_J1  2100
#define MAX_ADC_J2  825

#define MIN_ADC_J3  1350
#define MIN_ADC_J4  2000
#define MAX_ADC_J5   800

#define MIN_ADC_J6  1400
#define MIN_ADC_J7  1800
#define MAX_ADC_J8  2860

#define MIN_ADC_J9  1400
#define MIN_ADC_J10 1900
#define MAX_ADC_J11 2930

//MAXIMUM ADC Position for each joint
#define MAX_ADC_J0  1500
#define MAX_ADC_J1  1300
#define MIN_ADC_J2  2750

#define MAX_ADC_J3  1308
#define MAX_ADC_J4  1270
#define MIN_ADC_J5  2550

#define MAX_ADC_J6  1500
#define MAX_ADC_J7  2600
#define MIN_ADC_J8  970

#define MAX_ADC_J9  1350
#define MAX_ADC_J10 1480
#define MIN_ADC_J11 810

//Radiants position for the MINIMUM ADC position
#define MIN_RAD_J0  (1.6773f)
#define MIN_RAD_J1  (1.5129f + 1.3f)
#define MIN_RAD_J2  (0.0f)

#define MIN_RAD_J3  (1.4642f )
#define MIN_RAD_J4  (1.5733f + 1.3f)
#define MIN_RAD_J5  (0.0f)

#define MIN_RAD_J6  (1.6773f)
#define MIN_RAD_J7  (1.5682f - 1.3f)
#define MIN_RAD_J8  (PI)

#define MIN_RAD_J9  (1.4642f)
#define MIN_RAD_J10 (1.3852f + 1.3f)
#define MIN_RAD_J11 (PI)

//Radiants position for the MAXIMUM ADC postion
#define MAX_RAD_J0  (1.8536f)
#define MAX_RAD_J1  (2.5688f + 1.3f)
#define MAX_RAD_J2  (PI)

#define MAX_RAD_J3  (1.2880f)
#define MAX_RAD_J4  (2.6111f + 1.3f)
#define MAX_RAD_J5  (PI)

#define MAX_RAD_J6  (1.8536f)
#define MAX_RAD_J7  (0.5304f -  1.3f)
#define MAX_RAD_J8  (0.0f)

#define MAX_RAD_J9  (1.2880f)
#define MAX_RAD_J10 (2.5688f + 1.3f)
#define MAX_RAD_J11 (0.0f)

//ultrasound
#define TIM_ULTRASOUND &htim10
#define V_SOUND 340000			//speed of sound in mm/s

//control system input clock timer
#define TIM_SYS &htim7
#define TIM_SYS_PRESC 0
#define TIM_SYS_PERIOD 65535

#define TIM_SENSORS &htim11

#define N_SAMPLES 512

#define OFFSET_XL_X -0.170f
#define OFFSET_XL_Y -0.170f
#define OFFSET_XL_Z -0.170f

#define OFFSET_GYRO_X 0.0f
#define OFFSET_GYRO_Y 0.0f
#define OFFSET_GYRO_Z 0.0f

//Madgick AHRS defs
#define sampleFreq	75.0f
#define betaDef		1.00f


//PID IMU gains
#define PID_RX_K  0.0f
#define PID_RX_KP 0.03f
#define PID_RX_KI 0.02f
#define PID_RX_KD 0.02f
#define PID_RX_DB 1e-3

#define PID_RZ_K  0.0f
#define PID_RZ_KP 0.03f
#define PID_RZ_KI 0.02f
#define PID_RZ_KD 0.02f
#define PID_RZ_DB 1e-3

#define PID_RY_K  0.0f
#define PID_RY_KP 0.0005f
#define PID_RY_KI 0.0002f
#define PID_RY_KD 0.0005f
#define PID_RY_DB 0.02f

#define MAX_PITCH_TROT PI/100

//ADC FIltering
#define FILTER_WINDOW_SIZE 123
#define IMU_FILTER_WINDOW_SIZE 63

//HOME sequence
#define N_RAMP_HOME 150
#define MAX_DEVICES 1

//if the kinematics file is not included
#ifndef INC_KINEMATICS_H_
#define CENTER_TO_HIP_X 0.00f
#define CENTER_TO_HIP_Y 0.00f
#define CENTER_TO_HIP_Z 0.00f
#endif


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
