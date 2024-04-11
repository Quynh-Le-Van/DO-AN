//+====================================================================
//
//  Confidential Information - Limited Distribution
//   to Authorized Persons Only
//
//  This software is protected as an unpublished work under the
//  U. S. copyright Act. Created 2016/Modified 2016 Copyright.
//  All rights reserved. Biotricity Inc.
//
//  Filename: <name of the file along with its extension>
//
//  Author(s): <first and last name of the author(s)>
//
//  Description: <A brief description of the content of the file>
//
//+====================================================================
/* Includes ----------------------------------------------------------- */
#include "motor.h"

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static Motor_Mobile_Config_T motorMobile[Motor_Mobile_Unknowm];

#define INFO(_ea, _eb, _pwm, _dirl, _dirr)\
        {.encoder_A = _ea, .encoder_B = _eb, .pwm = _pwm, .dir_l = _dirl, .dir_r = _dirr}

static Motor_Mobile_Config_Pin_T MOTOR_MOBILR_LIST[Motor_Mobile_Unknowm] = {
  INFO(MOTOR_ENCODER_A_1, MOTOR_ENCODER_B_1, MOTOR_EN_1, MOTOR_L_1, MOTOR_R_1),
  INFO(MOTOR_ENCODER_A_2, MOTOR_ENCODER_B_2, MOTOR_EN_2, MOTOR_L_2, MOTOR_R_2),
  INFO(MOTOR_ENCODER_A_3, MOTOR_ENCODER_B_3, MOTOR_EN_3, MOTOR_L_3, MOTOR_R_3),
  INFO(MOTOR_ENCODER_A_4, MOTOR_ENCODER_B_4, MOTOR_EN_4, MOTOR_L_4, MOTOR_R_4)
};

#undef INFO

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void Motor_SetVel(Motor_Mobile_T motor, int velcCmd)
{
  
}
int  Motor_ReadCurVel(Motor_Mobile_T motor)
{

}

void Motor_ReadEncoderCallback(void)
{
  for (int index = 0; index < Motor_Mobile_Unknowm; index++)
  {
    int state;
    state = (state<<1) | digitalRead(MOTOR_MOBILR_LIST[index].encoder_A);
    state = (state<<1) | digitalRead(MOTOR_MOBILR_LIST[index].encoder_B);
    state = state&0x03;

    switch (state) 
    {
		case 0:
			if(motorMobile[index].prestate == 1) motorMobile[index].encoder++;
			else motorMobile[index].encoder--;
		break;

		case 1:
			if(motorMobile[index].prestate==3) motorMobile[index].encoder++;
			else motorMobile[index].encoder--;
		break;

		case 2:
			if(motorMobile[index].prestate==0) motorMobile[index].encoder++;
			else motorMobile[index].encoder--;
		break;

		case 3:
			if(motorMobile[index].prestate==2) motorMobile[index].encoder++;
			else motorMobile[index].encoder--;
		break;
		}

    motorMobile[index].prestate = state;
  }
}



/* End of file -------------------------------------------------------- */