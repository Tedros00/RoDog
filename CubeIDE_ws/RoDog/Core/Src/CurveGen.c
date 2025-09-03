/*
 * CurveGen.c
 *
 *  Created on: May 26, 2022
 *      Author: Wissam
 */


#include"CurveGen.h"

void CurvePoints(RODOG_Control * r, arm_matrix_instance_f32 *xyz){
	float T_st = L_STRIDE/(N_GAIT_CURVE*r->BodyVel);	//period of the stance period over the number of points ;
	//choose the correct offset based on the gait chosen
	switch(r->gait){
		case TROT:
			r->LegCounter[1] = r->LegCounter[0] + TROT_OFFSET;
			r->LegCounter[2] = r->LegCounter[0];
			r->LegCounter[3] = r->LegCounter[0] + TROT_OFFSET;

			if(++r->LegCounter[0]>=N_GAIT_CURVE){
				r->LegCounter[0] = 0;
			}
			break;

		case STALK:
			r->LegCounter[1] = r->LegCounter[0] + STALK_OFFSET;
			r->LegCounter[2] = r->LegCounter[1];
			r->LegCounter[3] = r->LegCounter[0];

			if(++r->LegCounter[0]>N_GAIT_CURVE){
				r->LegCounter[0] = 0;
			}
			break;

		case GALLOP:
			 r->LegCounter[1] = r->LegCounter[0] + GALLOP_OFFSET;
			 r->LegCounter[2] = r->LegCounter[1] + GALLOP_OFFSET;
			 r->LegCounter[3] = r->LegCounter[2] + GALLOP_OFFSET;

			if(++r->LegCounter[0]>N_GAIT_CURVE){
				r->LegCounter[0] = 0;
			}
			 break;

		case PACE:
			r->LegCounter[1] = r->LegCounter[0];
			r->LegCounter[2] = r->LegCounter[0] + PACE_OFFSET;
			r->LegCounter[3] = r->LegCounter[0] + PACE_OFFSET;

			if(++r->LegCounter[0]>N_GAIT_CURVE){
				r->LegCounter[0] = 0;
			}

			break;

		case CRAWL:
			//All Legs finished their Swing Phase
			if(r->CurveCount > N_SWING){
				if(r->CurveCount > 2*N_SWING){
					if(r->CurveCount > 3*N_SWING){
						if(r->CurveCount > 4*N_SWING){
							if(r->CurveCount > 4*N_SWING + N_STANCE){
								for(int i = 0 ; i < 4 ; i++){
									r->LegCounter[i] = 0;
									}
									r->CurveCount = 0;
								} else{ for(int i = 0 ; i < 4 ; i++)r->LegCounter[i]++;
							}
						} else{r->LegCounter[2]++;}
					} else{r->LegCounter[1]++;}
				} else{r->LegCounter[3]++;}
			} else {r->LegCounter[0]++;}

			r->CurveCount++;
			break;

		case STOP :
			for(int i = 0 ; i < 4 ; i++)
				r->LegCounter[i] = 0.0f;
			break;
	}

	for(int i = 0 ; i < 4 ; i++){

		if(r->LegCounter[i] > N_GAIT_CURVE){
			r->LegCounter[i] -=N_GAIT_CURVE;
			r->CurveCount++;
		}

		//swing phase(circle)
		if(r->LegCounter[i] <= N_SWING){
		    float32_t tempX[N_SCATTER], tempY[N_SCATTER];

		    for (int n = 0; n < N_SCATTER; n++) {
		        tempX[n] = XBScatter[n];
		        tempY[n] = YBScatter[n];
		    }

		    float32_t t = (float32_t)r->LegCounter[i]/N_SWING;
		    // Calculate the i-th point on the Bezier curve using De Casteljau's algorithm
		    for (int k = 1; k < N_SCATTER; k++) {
		        for (int j = 0; j < N_SCATTER - k; j++) {
		            tempX[j] = (1 - t) * tempX[j] + t * tempX[j + 1];
		            tempY[j] = (1 - t) * tempY[j] + t * tempY[j + 1];
		        }
		    }
		    // The final result is stored in tempX[0] and tempY[0]
		    xyz->pData[0+i] = tempX[0];
		    xyz->pData[4+i] = tempY[0];
		}

		//stance phase
		if(r->LegCounter[i] > N_SWING){
			xyz->pData[0+i] =  (((r->LegCounter[i]-N_SWING)*L_STRIDE)/N_STANCE) - (L_STRIDE/2.0f);
			xyz->pData[4+i] = -VIRT_DIS *cos(PI*(0.5-(r->BodyVel*(float)(r->LegCounter[i]-N_SWING)*T_st)/(L_STRIDE*N_STANCE)));
		}

		//final row
		xyz->pData[8 +i] = 0;
		xyz->pData[12+i] = 1;

		xyz->pData[i] = xyz->pData[i]* r->CurveScale;
		}

	//rotate according to the heading angle
	crab(r->psi_crabbing , xyz);


	//translate to LEG home frame
	for(int i = 0 ; i < 4 ; i++){
		xyz->pData[0+i] =  (HOME_X + GaitCurveOffsetX[i] )+ xyz->pData[0+i];
		xyz->pData[ 4+i] = HOME_Y -  xyz->pData[4+i];
		xyz->pData[ 8+i] = (HOME_Z + GaitCurveOffsetY[i]) - xyz->pData[8+i];
		xyz->pData[12+i] = 0;
	}

}

//this function scales down and rotates the obtained curve points

void crab(float32_t psi , arm_matrix_instance_f32 * EE){
	float32_t cPsi = cos(psi) , sPsi = sin(psi);
	float32_t M_crabxyz[] = {  cPsi   , 0     ,  sPsi , 0 ,
							 0      , 1     , 0     , 0 ,
							-sPsi  , 0  	  , cPsi  , 0 ,
							  0      , 0     , 0     , 1 };

	arm_matrix_instance_f32 M_crab;
	arm_mat_init_f32(&M_crab , 4 , 4 , M_crabxyz);
	arm_mat_mult_f32(&M_crab , EE , EE);

}

void Tpoints(float Phi , arm_matrix_instance_f32 *EE){

	float32_t cPhi,sPhi;
	float32_t EExyz_L[8];
	float32_t EExyz_R[8];

	arm_matrix_instance_f32 T_R , T_L , EE_L , EE_R;

	EExyz_R[0] = EE->pData[0];
	EExyz_R[1] = EE->pData[1];
	EExyz_R[2] = EE->pData[4];
	EExyz_R[3] = EE->pData[5];
	EExyz_R[4] = EE->pData[8];
	EExyz_R[5] = EE->pData[9];
	EExyz_R[6] = 1;
	EExyz_R[7] = 1;


	EExyz_L[0] = EE->pData[2];
	EExyz_L[1] = EE->pData[3];
	EExyz_L[2] = EE->pData[6];
	EExyz_L[3] = EE->pData[7];
	EExyz_L[4] = EE->pData[10];
	EExyz_L[5] = EE->pData[11];
	EExyz_L[6] = 1;
	EExyz_L[7] = 1;



	cPhi = cos(Phi);
	sPhi = sin(Phi);
	float32_t TData_R[] = {  cPhi   , 0     ,  sPhi , 0 ,
						   	   0      , 1     , 0     , 0 ,
							   -sPhi  , 0  	  , cPhi  , 0 ,
							   0      , 0     , 0     , 1 };
	//sPhi = -sPhi;
	float32_t TData_L[] = {  cPhi   , 0     ,  sPhi , 0 ,
						   	   0      , 1     , 0     , 0 ,
							   -sPhi  , 0  	  , cPhi  , 0 ,
							   0      , 0     , 0     , 1 };

	arm_mat_init_f32(&EE_L , 4 , 2 , EExyz_L);
	arm_mat_init_f32(&EE_R , 4 , 2 , EExyz_R);
	arm_mat_init_f32(&T_R , 4 , 4, TData_R);
	arm_mat_init_f32(&T_L , 4 , 4, TData_L);

	arm_mat_mult_f32(&T_L , &EE_L , &EE_L);
	arm_mat_mult_f32(&T_R , &EE_R , &EE_R);

	EE->pData[0]  = EE_R.pData[0];
	EE->pData[1]  = EE_R.pData[1];
	EE->pData[2]  = EE_L.pData[0];
	EE->pData[3]  = EE_L.pData[1];
	EE->pData[4]  = EE_R.pData[2];
	EE->pData[5]  = EE_R.pData[3];
	EE->pData[6]  = EE_L.pData[2];
	EE->pData[7]  = EE_L.pData[3];
	EE->pData[8]  = EE_R.pData[4];
	EE->pData[9]  = EE_R.pData[5];
	EE->pData[10] = EE_L.pData[4];
	EE->pData[11] = EE_L.pData[5];

}

//N chooses R, number of combinations where N>R
int NcR(int n , int k){
    // Base Cases
    if (k > n)
        return 0;
    if (k == 0 || k == n)
        return 1;

    // Recur
    return NcR(n - 1, k - 1)+ NcR(n - 1, k);
}
