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
#include "platform.h"
#include <Servo.h>

/* Public defines ---------------------------------------------------- */
#define SERVO_BASE     (1)
#define SERVO_SHOULDER (1)
#define SERVO_ELLOW    (1)
#define SERVO_ARM      (1)
#define SERVO_GRIPPER  (1)

/* Public enumerate/structure ---------------------------------------- */
/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ---------------------------------------- */
void Mani_Init(void);
void Mani_SetSpeedEndEffector(Manipulator_Vel_Config_T vel);
void Mani_SetSpeedJoint(Manipulator_Angle_Vel_T vel);
void Mani_SetPosition(Manipulator_Pos_Config_T pos);

/* End of file -------------------------------------------------------- */
#endif //