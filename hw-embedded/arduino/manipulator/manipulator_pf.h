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
#ifndef _MANIPULATOR_H
#define _MANIPULATOR_H
/* Includes ----------------------------------------------------------- */
#include "kinematic.h"
#include <string.h>

/* Public defines ---------------------------------------------------- */
#define SERVO_BASE     (11)
#define SERVO_SHOULDER (10)
#define SERVO_ELLOW    (9)
#define SERVO_ARM      (6)
#define SERVO_GRIPPER  (5)

#define SERVO_SPEED_DEFAULT (60)       // degree / second unit
#define SERVO_INIT_ANGLE    (45)

/* Public enumerate/structure ---------------------------------------- */
typedef enum
{
  MANI_DATA_POS,
  MANI_DATA_VEL,
  MANI_DATA_PICK,
  MANI_DATA_ERROR
} MANI_DATA_TYPE_T;

/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern Manipulator_Angle_Vel_T g_JointSpeedCommand;
extern Manipulator_Angle_Config_T g_JointAngleCommand;
extern Manipulator_Pos_Config_T g_ManipulatorPos;

/* Public function prototypes ---------------------------------------- */
void Mani_Init(void);
void Mani_SetSpeedEndEffector(Manipulator_Vel_Config_T vel);
void Mani_SetSpeedJoint(Manipulator_Angle_Vel_T vel);
void Mani_SetPosition(Manipulator_Pos_Config_T pos);
void Mani_Gripper(bool isGrip);
MANI_DATA_TYPE_T Mani_ReceiveData(String data);

/* End of file -------------------------------------------------------- */
#endif //