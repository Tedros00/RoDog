/*
 * CurveGen.h
 *
 *  Created on: May 26, 2022
 *      Author: Wissam
 */

#ifndef INC_CURVEGEN_H_
#define INC_CURVEGEN_H_
#include "main.h"
#include "math.h"

//control points of the bezier curve Swing phase
extern const float32_t  YBScatter[];
extern const float32_t XBScatter[];
extern float32_t GaitCurveOffsetY[];
extern float32_t GaitCurveOffsetX[];
extern uint8_t n_swing ;
extern uint8_t n_stance;

#define N_SCATTER 12											//number of data points from which we will generate the curve
#define L_STRIDE  30.0f										    //leg stride in a gait cycle
#define GALLOP_OFFSET (int)(N_GAIT_CURVE/4)						//offset between the legs for the gallop gait
#define STALK_OFFSET  (int)(N_GAIT_CURVE/2)						//offset between the legs for the stalk gait
#define TROT_OFFSET   (int)(N_GAIT_CURVE/2)						//offset between the legs for the trot gait
#define PACE_OFFSET   (int)(N_GAIT_CURVE/2)
#define VIRT_DIS 0.00f											//virtual displacement in the vertical direction for the stance gait
#define T_STANCE	0.50f										//duration of the stance in the gait cycle
#define T_SW 0.25f												//period of the swing phase
#define N_STANCE (uint8_t)n_stance											//number of points for the stance phase
#define N_SWING	(uint8_t)n_swing												//number of points in the swing phase
#define N_GAIT_CURVE (uint8_t)(N_STANCE+N_SWING)				//number of points in the curve gait
#define YBMAX 25												//bezier Y scatter max

extern float64_t NCR[N_SCATTER];

void CurvePoints(RODOG_Control * r, arm_matrix_instance_f32 * xyz);
void Tpoints(float Phi, arm_matrix_instance_f32 *EE);
void crab(float32_t psi , arm_matrix_instance_f32 * EE);
int NcR(int N , int r);//number of combinations N chooses r
int min(int n , int k);

#endif /* INC_C1URVEGEN_H_ */
