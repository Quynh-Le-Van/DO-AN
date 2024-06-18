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
#define MOTOR_EN_1        (3)
#define MOTOR_L_1         (34)
#define MOTOR_R_1         (35)
#define MOTOR_ENCODER_A_1 (37)
#define MOTOR_ENCODER_B_1 (36)

#define MOTOR_EN_2        (2)
#define MOTOR_L_2         (41)
#define MOTOR_R_2         (40)
#define MOTOR_ENCODER_A_2 (42)
#define MOTOR_ENCODER_B_2 (43)

#define MOTOR_EN_3        (4)
#define MOTOR_L_3         (29)
#define MOTOR_R_3         (28)
#define MOTOR_ENCODER_A_3 (30)
#define MOTOR_ENCODER_B_3 (31)

#define MOTOR_EN_4        (5)
#define MOTOR_L_4         (22)
#define MOTOR_R_4         (23)
#define MOTOR_ENCODER_A_4 (24)
#define MOTOR_ENCODER_B_4 (25)

#define ENCODER_MAX_VALUE_PPR (998)
#define MOTOR_VOLREF          (24.0)

#define MOBILE_MAX_LINEAR_VEL (0.5)
#define MOBILE_MAX_ANGULAR_VEL (1.5)


/* Public enumerate/structure ---------------------------------------- */

//  //M_1-----M_2\\
//   |          |
//   |          |
//  \\M_4-----M_3//

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
  double preVel;
  double velCurrentFilter;
} Motor_Config_T;

/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern Motor_Config_T g_MotorMobile[MOTOR_MOBILE_UNKNOW];
extern Wheel_Vel_Config_T g_MotorSpeedCommand;
extern Mobile_Vel_Config_T g_MobileSpeedCommand;
extern Mobile_Vel_Config_T g_MobileSpeedCurent;
extern Mobile_Pos_Config_T g_MobilePositionCurent;
extern uint32_t preTimeCommand;
extern Motor_Config_Pin_T MOTOR_PIN_LIST[MOTOR_MOBILE_UNKNOW];

/* Public function prototypes ---------------------------------------- */
void Mobile_PIDInit(void);
void Mobile_SetSpeed(Mobile_Vel_Config_T speedCommand);
Mobile_Vel_Config_T Mobile_ReadCurrentSpeed(void);
Mobile_Pos_Config_T Mobile_ReadCurrentPosition(void);
void Test_SetPin(double vel);
void Mobile_TrackingTrajectory();

/* End of file -------------------------------------------------------- */
#endif //