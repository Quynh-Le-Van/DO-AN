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
#include "manipulator_pf.h"

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Public variables --------------------------------------------------- */
Servo g_BaseServo, g_ShoulderServo, g_EllowServo, g_ArmServo, g_GripperServo;

/* Private variables -------------------------------------------------- */
/* Function definitions ----------------------------------------------- */
void Mani_Init(void)
{
  g_BaseServo.attach(SERVO_BASE);
  g_ShoulderServo.attach(SERVO_SHOULDER);
  g_EllowServo.attach(SERVO_ELLOW);
  g_ArmServo.attach(SERVO_ARM);
  g_GripperServo.attach(SERVO_GRIPPER);
}

void Mani_SetSpeedJoint(Manipulator_Angle_Vel_T vel)
{
  
}
/* End of file -------------------------------------------------------- */