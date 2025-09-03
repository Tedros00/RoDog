/*
 * kinematics.c
 *
 *  Created on: May 21, 2022
 *      Author: Wissam
 */
#include "kinematics.h"

void IK_leg(float * HLK_cmd ,  float * EExyz_cmd){


	for(int i = 0 ; i < 4 ; i++){
		//calculate the needed variables only once, this function needs to loop at a very high rate
		float32_t x = EExyz_cmd[0+i];
		float32_t y = EExyz_cmd[4+i];
		float32_t z = EExyz_cmd[8+i];

		float32_t R = sqrt(x*x + y*y + z*z);

		//check if the given data points are inside the contour of definition
		//if not, translate the end effector to the extreme, on the sphere of enclosing the volume

		if(R > MAX_L){
		   float32_t c_phi,s_phi,c_theta,s_theta,Rs_phi; //polar angles in radiant
			c_phi = z/R;
			s_phi = sqrt(1-pow(c_phi/R , 2));
			Rs_phi = R*s_phi;
			s_theta = y /Rs_phi;
			c_theta = x /Rs_phi;
			R = MAX_L;
			//update the EE position
			x  = R * s_phi * c_theta;
			y  = R * s_phi * s_theta;
			z  = R * c_phi;
		}

		// Might consider better frame offsets for the IK leg.

		//hip angle
		float32_t R0 = sqrt(pow(z,2) + pow(y,2) - pow(L_HIPS,2));
		float32_t Phi = atan2(y,z);
		float32_t Psi = asin(L_HIPS/R0);
		HLK_cmd[i] = Phi - Psi + 0.7637336417948966f;

		float32_t D = sqrt( pow(hypot(y,z),2) - pow(L_HIPS,2));
		float32_t R1 = hypot(x , D);
		float32_t Alpha = atan2(D,x);
		float32_t Beta = atan2(L_LEG*sin(PI-HLK_cmd[12+i]) , L_COXA+L_LEG*cos(PI-HLK_cmd[12+i]));
		HLK_cmd[4+i] = PI - (Alpha-Beta);

		//knee angle
		HLK_cmd[12+i] = acos( (pow(L_COXA,2) + pow(L_LEG,2) - pow(R1,2) )/(2.0f*L_COXA*L_LEG));

		//convert knee to crank angle
		HLK_cmd[8+i] = knee_to_crank(HLK_cmd[12+i]);
	}

	//convert Knee Angles to the motor crank angle
	//mirror angles for left side of the robot
	//UPPER LEG MIRRORS
	HLK_cmd[6] = PI-HLK_cmd[6];
	//HTK[7] = PI-HTK[7];

	//KNEE MIRRORS
	HLK_cmd[8] = PI-HLK_cmd[8];
	HLK_cmd[9] = PI-HLK_cmd[9];

	//HIP MIRRORS
	HLK_cmd[1]  = PI - HLK_cmd[1];
	HLK_cmd[3]  = PI - HLK_cmd[3];

	return;
}

float32_t knee_to_crank(float32_t knee){

	return (asin( (knee-AMP_CRANK_SINE)/(ANGULAR_OFFSET_CRANK_SINE) ) - TIME_OFFSET_CRANK_SINE)/(OMEGA_CRANK_SINE);

}

arm_status IK_body(RODOG_Control * r, arm_matrix_instance_f32 * EE_cmd){
	//limit the given parameters
	if(r->BodyTx>MAX_TX)r->BodyTx = MAX_TX;
	else if(r->BodyTx<MIN_TX)r->BodyTx = MIN_TX;

	if(r->BodyTy>MAX_TY)r->BodyTy = MAX_TY;
	else if(r->BodyTy<MIN_TY)r->BodyTy = MIN_TY;

	if(r->BodyTz>MAX_TZ)r->BodyTz = MAX_TZ;
	else if(r->BodyTz<MIN_TZ)r->BodyTz = MIN_TZ;

	if(r->BodyPitch>MAX_PITCH)r->BodyPitch = MAX_PITCH;
	else if(r->BodyPitch<MIN_PITCH)r->BodyPitch = MIN_PITCH;

	if(r->BodyRoll>MAX_ROLL)r->BodyRoll = MAX_ROLL;
	else if(r->BodyRoll<MIN_ROLL)r->BodyRoll = MIN_ROLL;

	if(r->BodyYaw>MAX_YAW)r->BodyYaw = MAX_YAW;
	else if(r->BodyYaw<MIN_YAW)r->BodyYaw = MIN_YAW;

	//pre calculate some the frequently used variables
	float32_t Mxyz[16],MINVxyz[16], M_CH0xyz[16];
	arm_matrix_instance_f32 M, MINV, M_CH0,CH0;

	arm_mat_init_f32(&M , 4 , 4 , Mxyz);
	arm_mat_init_f32(&MINV , 4 , 4 , MINVxyz);
	arm_mat_init_f32(&M_CH0 , 4 , 4 , M_CH0xyz);
	arm_mat_init_f32(&CH0 , 4 , 4 ,(float32_t*)CH0xyz);

	//create Transform Matrices
	TransformMatrix(r , &M  ,  FALSE);

	arm_status b = ARM_MATH_SUCCESS;
	 EE_cmd->pData[10] = -EE_cmd->pData[10];
	 EE_cmd->pData[11] = -EE_cmd->pData[11];

	 //get the EE position with respect to the COM
	 b = b + arm_mat_add_f32(EE_cmd , &CH0 ,EE_cmd);

	 //rotate the EE positions according to the robot heading angle

	 //transform the body frame with the matrix M
	 b = b + arm_mat_mult_f32(&M , &CH0 , &M_CH0);

	 EE_cmd->pData[15] = 1;

	 arm_mat_inverse_f32(&M , &MINV);
	 //get the local transformed EE position with respect to each root,
	 //by rotating the leg frame contrary to the body frame

	 b = b + arm_mat_mult_f32(&MINV , EE_cmd , EE_cmd);

	 //get the transformed EE positions with respect to the COM
	 b = b + arm_mat_sub_f32(EE_cmd , &M_CH0 , EE_cmd);

	 EE_cmd->pData[10] = -EE_cmd->pData[10];
	 EE_cmd->pData[11] = -EE_cmd->pData[11];
	 //the matrix EE is to be the input of the IK_leg() function to deduce
	 //the joint angles of the robot

	 return b;	//b = 0 => success
}

void TransformMatrix(RODOG_Control * r , arm_matrix_instance_f32 * M , bool Translation_only){
	double cr,sr,cp,sp,cy,sy,tx,ty,tz;
	arm_matrix_instance_f32 t ,rx , ry , rz;

	if(Translation_only == FALSE){
		cr = cos(r->BodyRoll/2.0f);
		sr = sin(r->BodyRoll/2.0f);

		cp = cos(r->BodyPitch/2.0f);
		sp = sin(r->BodyPitch/2.0f);

		cy = cos(r->BodyYaw/2.0f);
		sy = sin(r->BodyYaw/2.0f);

		tx = r->BodyTx/2.0f;
		ty = r->BodyTy/2.0f;
		tz = r->BodyTz/2.0f;
	}

	else{
		cr = cos(0.0f);
		sr = sin(0.0f);

		cp = cos(0.0f);
		sp = sin(0.0f);

		cy = cos(0.0f);
		sy = sin(0.0f);

		tx = r->BodyTx;
		ty = r->BodyTy;
		tz = r->BodyTz;
	}

	float32_t T[] = {1 , 0 , 0 , tx,
				 	 0 , 1 , 0 , ty,
					 0 , 0 , 1 , tz,
					 0 , 0 , 0 , 1};

	float32_t Rx[] = {1 , 0 , 0 ,    0 ,
				  	  0 , cr , -sr , 0 ,
					  0 , sr , cr ,  0 ,
					  0 , 0 , 0 ,    1};

	float32_t Ry[] = {cy , 0 , sy , 0,
				   	   0 , 1 , 0 ,  0,
					   -sy , 0 , cy , 0,
					   0 , 0 , 0 ,  1 };

	float32_t Rz[] = {cp , -sp , 0 , 0,
					  sp , cp ,  0 , 0,
					   0 ,  0 ,  1,  0,
					   0 ,  0 ,  0 , 1};
//init matrices
arm_mat_init_f32(&t  , 4 , 4 , T );
arm_mat_init_f32(&rx , 4 , 4 , Rx);
arm_mat_init_f32(&ry , 4 , 4 , Ry);
arm_mat_init_f32(&rz , 4 , 4 , Rz);

//get the transform matrix
arm_mat_mult_f32(&rx , &ry , M);
arm_mat_mult_f32(M   , &rz , M);
arm_mat_mult_f32(M   , &t  , M);

}

void FK_leg(float32_t*HLK_fb , arm_matrix_instance_f32 * EE_fb){
	//pre calculate some the frequently used variables
	float ch,sh,ct,st,ck,sk;

	float32_t T01xyz[16] = { 1 , 0 , 0 , 0,
			  	  	  	  	 0 , 1 , 0 , 0,
							 0 , 0 , 1 , 0,
							 0 , 0, 0  , 1};

	float32_t T12xyz[16] = { 1 , 0 , 0 , 0,
			  	  	  	  	 0 , 1 , 0 , 0,
							 0 , 0 , 1 , 0,
							 0 , 0, 0  , 1};

	float32_t T23xyz[16] = { 1 , 0 , 0 , 0,
			  	  	  	  	 0 , 1 , 0 , 0,
							 0 , 0 , 1 , 0,
							 0	, 0, 0 , 1};

	float32_t T03xyz[16] = { 1 , 0 , 0 , 0,
			  	  	  	  	 0 , 1 , 0 , 0,
							 0  , 0 , 1 , 0,
							 0	, 0, 0 , 1};

	arm_matrix_instance_f32 T01 , T12 , T23 , T03;
	arm_mat_init_f32(&T01 , 4 , 4 , (float32_t*)T01xyz);		//transform matrix from 0 to 1
	arm_mat_init_f32(&T12 , 4 , 4 , (float32_t*)T12xyz);		//transform matrix from 1 to 2
	arm_mat_init_f32(&T23 , 4 , 4 , (float32_t*)T23xyz);		//transform matrix from 2 to 3
	arm_mat_init_f32(&T03 , 4 , 4 , (float32_t*)T03xyz);		//transform matrix from 0 to 3

	for(int i = 0 ; i < 4 ; i++){

		ch = cos(HLK_fb[i]);
		sh = sin(HLK_fb[i]);
		ct = cos(HLK_fb[4+i]);
		st = sin(HLK_fb[4+i]);
		ck = cos(HLK_fb[8+i]);
		sk = sin(HLK_fb[8+i]);

		//construct the feedback transformation matrix T of the leg
		T01.pData[5] = ch;
		T01.pData[6] = -sh;
		T01.pData[7] = -sh * L_HIPS;
		T01.pData[9] = sh;
		T01.pData[10] = ch;
		T01.pData[11] = ch * L_HIPS;

		T12.pData[0] = ct;
		T12.pData[1] = -st;
		T12.pData[3] = -ct * -L_COXA;
		T12.pData[4] = st;
		T12.pData[5] = ct;
		T12.pData[7] = st * -L_COXA;

		T23.pData[0] = ck;
		T23.pData[1] = -sk;
		T23.pData[3] = ck * L_LEG;
		T23.pData[4] = sk;
		T23.pData[5] = ck;
		T23.pData[3] = sk * L_LEG;

		//get the transform matrix from the leg root to the EE
		arm_mat_mult_f32(&T01 , &T12,  &T03);
		arm_mat_mult_f32(&T03 , &T23,  &T03);

		//feedback EE positions of each leg
		EE_fb->pData[0+i] = T03.pData[3];
		EE_fb->pData[4+i] = T03.pData[7];
		EE_fb->pData[8+i] = T03.pData[11];
	}

	//mirroring the Leg angles on the left side of the Robot
	for(int i = 6 ; i < 12 ; i++)
		EE_fb->pData[i] = PI/2 - EE_fb->pData[i];

	//after deducting the position of each leg with respect to its root, we can compare this result with the output of the IK and deduce our error
}

arm_status FK_body(RODOG_Control *r , arm_matrix_instance_f32 * EE_fb){
	//pre calculate some the frequently used variables
	float cr,sr,cp,sp,cy,sy;
	float32_t _Mxyz[16] , _C_EExyz[16];
	arm_matrix_instance_f32 _M , _C_EE;

	arm_mat_init_f32(&_M , 4 , 4 , _Mxyz);
	arm_mat_init_f32(&_C_EE , 4 , 4 , _C_EExyz);
	//mirror coefficients for the hip joint position
	uint8_t mirx[] = {1 , -1 , -1 , 1};
	uint8_t mirz[] = {1 , 1 , -1 , -1};

	cr = cos(r->BodyRoll);
	sr = sin(r->BodyRoll);
	cp = cos(r->BodyPitch);
	sp = sin(r->BodyPitch);
	cy = cos(r->BodyYaw);
	sy = sin(r->BodyYaw);

	//uptade the transformation matrix
	_M.pData[0] = cy*cp;
	_M.pData[1] = -cy*sp;
	_M.pData[2] = sy;
	_M.pData[3] = 0;

	_M.pData[4] = sr*sy*cp+sp*cr;
	_M.pData[5] = -sp*sr*sy + cp*cr;
	_M.pData[6] = -sr*cy;
	_M.pData[7] = 0;

	_M.pData[8] = -sy*cr*cp+sr*sp;
	_M.pData[9] = sy*cr*sp+sr*cp;
	_M.pData[10] = cr*cy;
	_M.pData[11] = 0;

	_M.pData[12] = 0;
	_M.pData[13] = 0;
	_M.pData[14] = 0;
	_M.pData[15] = 1;

	//here we are adding the offset translation directly into the feedback EE matrix, because each leg is offset to a certain position
	for(int i = 0 ; i < 4 ; i++){
		EE_fb->pData[i] += mirx[i]*(CENTER_TO_HIP_X + r->BodyTx);
		EE_fb->pData[4+i] += CENTER_TO_HIP_Y + r->BodyTy;
		EE_fb->pData[8+i] += mirz[i]*(CENTER_TO_HIP_Z + r->BodyTz);
	}

	//keep track if all the arm_math operation were successful
	arm_status b = ARM_MATH_SUCCESS;
	//get the position of each EE with respect to the COM
	b = b + arm_mat_mult_f32(&_M , EE_fb , &_C_EE);

	//we can compare the matrix _C_EE with the matrix C_EE to deduce our error in the IK model
	return b;
}

//util
float rad_to_degrees(float rad){
	return rad*180.0f/PI ;
}
