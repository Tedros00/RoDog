/*
 * RodogTypes.h
 *
 *  Created on: Jun 13, 2022
 *      Author: Wissam
 */

#ifndef INC_RODOGTYPES_H_
#define INC_RODOGTYPES_H_
#include "main.h"

//gait number
typedef enum {
	STOP   = 0,
	GALLOP = 1,
	STALK  = 2,
	TROT   = 3,
	CRAWL  = 4,
	PACE   = 5,
}GAIT;

typedef enum {
	FALSE = 0,
	TRUE  = 1,
}bool;

typedef struct{
	 uint8_t  JointNumber;		// associated joint number
	 TIM_TypeDef * Timer;		// the timer of the PWM channel for that joint
	 uint8_t timerNum;			// number of the timer
	 uint8_t  TimerChannel;		// channel of the chosen timer
	 uint8_t  ADCChannel;		// which adc channel is the potentiometer attached to
	 uint8_t ADCNum;			// which ADC is used for this jiont
	 uint8_t  MuxChannel;		// which channel of the MUX this servo feedback current is attached to
	 uint16_t MAX_ADC_pos;		// maximum adc count for this joint
	 uint16_t MIN_ADC_pos;		// minimum adc count for this joint
	 float32_t MAX_rad_pos;		// maximum angular position in radians
	 float32_t MIN_rad_pos;		// minimum angular position in radians
	 float32_t SIT_DOWN_rad_pos;
	 float32_t HOME_rad_pos;
	 float Offset;				// angular offset required due to assembly misalignment
	 bool enabled;				// enable and disable the joint by software
	 uint8_t index;				// index in the inverse kinematic matrix
}Joint;

typedef struct{
	float32_t BodyTx;						// body Translation in X
	float32_t BodyTy;						// body Translation in Y
	float32_t BodyTz;						// body Translation in Z
	float32_t BodyPitch;					// body Rotation Z
	float32_t BodyRoll;						// body Rotation X
	float32_t BodyYaw;						// body Yotation Y
	float32_t CurveScale;					// scaling factor of the gait curve
	uint32_t CurveCount;					// number of gait curves done
	uint32_t LegCounter[4];					//index of the EE on the gait curve
	GAIT gait;								//gait number offset
	float32_t BodyVel;						//body velocity
	uint8_t MuxSelect;						//force feedback mux channel selector
	bool IsEnabled;							//motor enabled boolean
	float32_t psi_heading;					//robot body heading angle
	float32_t psi_crabbing;					//the angle about the y-axis of the gait curve
}RODOG_Control;

typedef struct{
	uint16_t xl_raw_data[3];
	uint16_t gyro_raw_data[3];
	uint16_t magn_raw_data[3];

	float32_t xl_rad[3];
	float32_t gyro_radps[3];
	float32_t magn_rad[3];
} IMU;


#endif /* INC_RODOGTYPES_H_ */
