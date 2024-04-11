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
#ifndef _MOTOR_H
#define _MOTOR_H 
/* Includes ----------------------------------------------------------- */
#include "platform.h"

/* Public defines ---------------------------------------------------- */
#define MOTOR_EN_1              (1)             
#define MOTOR_L_1             (1)              
#define MOTOR_R_1             (1)
#define MOTOR_ENCODER_A_1             (1)
#define MOTOR_ENCODER_B_1             (1)

#define MOTOR_EN_2              (1)
#define MOTOR_L_2             (1)
#define MOTOR_R_2             (1)
#define MOTOR_ENCODER_A_2             (1)
#define MOTOR_ENCODER_B_2             (1)

#define MOTOR_EN_3              (1)
#define MOTOR_L_3             (1)
#define MOTOR_R_3             (1)
#define MOTOR_ENCODER_A_3             (1)
#define MOTOR_ENCODER_B_3             (1)

#define MOTOR_EN_4              (1)
#define MOTOR_L_4             (1)
#define MOTOR_R_4             (1)
#define MOTOR_ENCODER_A_4             (1)
#define MOTOR_ENCODER_B_4             (1)

/* Public enumerate/structure ---------------------------------------- */

//  //M_1-----M_2\\
//   |          |
//   |          |
//  \\M_3-----M_4//

typedef enum
{
  Motor_Mobile_1,
  Motor_Mobile_2,
  Motor_Mobile_3,
  Motor_Mobile_4,
  Motor_Mobile_Unknowm
} Motor_Mobile_T;

typedef struct 
{
  int encoder_A;
  int encoder_B;
  int pwm;
  int dir_l;
  int dir_r;
} Motor_Mobile_Config_Pin_T;

typedef struct 
{
  int pwm;
  int encoder;
  int dir;
  int prestate;
} Motor_Mobile_Config_T;

/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ---------------------------------------- */
void Motor_SetVel(Motor_Mobile_T motor, int velcCmd);
int  Motor_ReadCurVel(Motor_Mobile_T motor);
void Motor_ReadEncoderCallback(void);

/* End of file -------------------------------------------------------- */
#endif  // 