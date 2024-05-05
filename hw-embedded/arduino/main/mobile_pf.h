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
#include "kinematic.h"
#include "platform.h"
#include <PID_v1.h>

/* Public defines ---------------------------------------------------- */
#define MOTOR_EN_1        (4)
#define MOTOR_L_1         (5)
#define MOTOR_R_1         (6)
#define MOTOR_ENCODER_A_1 (8)
#define MOTOR_ENCODER_B_1 (7)

#define MOTOR_EN_2        (1)
#define MOTOR_L_2         (1)
#define MOTOR_R_2         (1)
#define MOTOR_ENCODER_A_2 (1)
#define MOTOR_ENCODER_B_2 (1)

#define MOTOR_EN_3        (1)
#define MOTOR_L_3         (1)
#define MOTOR_R_3         (1)
#define MOTOR_ENCODER_A_3 (1)
#define MOTOR_ENCODER_B_3 (1)

#define MOTOR_EN_4        (1)
#define MOTOR_L_4         (1)
#define MOTOR_R_4         (1)
#define MOTOR_ENCODER_A_4 (1)
#define MOTOR_ENCODER_B_4 (1)

#define ENCODER_MAX_VALUE_PPR (998)
#define MOTOR_VOLREF          (24)

/* Public enumerate/structure ---------------------------------------- */

//  //M_1-----M_2\\
//   |          |
//   |          |
//  \\M_3-----M_4//

typedef enum
{
  MOTOR_MOBILE_1,
  MOTOR_MOBILE_2,
  MOTOR_MOBILE_3,
  MOTOR_MOBILE_4,
  MOTOR_MOBILE_UNKNOW
} Motor_Mobile_T;

typedef struct
{
  int encoder_A;
  int encoder_B;
  uint8_t enable;
  int dir_l;
  int dir_r;
} Motor_Config_Pin_T;

typedef struct
{
  double pwmOut;
  long encoderCount;
  int dir;
  int posCount;
  double velCurrent;
  uint8_t prestate;
} Motor_Config_T;

/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern Motor_Config_T g_MotorMobile[MOTOR_MOBILE_UNKNOW];
extern Wheel_Vel_Config_T g_MotorSpeedCommand;
extern Mobile_Vel_Config_T g_MobileSpeedCommand;
extern Mobile_Vel_Config_T g_MobileSpeedCurent;
extern Mobile_Pos_Config_T g_MobilePositionCurent;  
extern uint32_t preTimeCommand;


/* Public function prototypes ---------------------------------------- */
void Mobile_PIDInit(void);
void Mobile_SetSpeed(Mobile_Vel_Config_T speedCommand);
Mobile_Vel_Config_T Mobile_ReadCurrentSpeed(void);
Mobile_Pos_Config_T Mobile_ReadCurrentPosition(void);
void Test_SetPin(Motor_Mobile_T motor);

/* End of file -------------------------------------------------------- */
#endif //