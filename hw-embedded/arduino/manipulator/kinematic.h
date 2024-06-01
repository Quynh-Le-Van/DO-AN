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
#ifndef _KINEMATIC_H
#define _KINEMATIC_H
/* Includes ----------------------------------------------------------- */
#include <math.h>
#include "Arduino.h"

/* Public defines ---------------------------------------------------- */
/* Public enumerate/structure ---------------------------------------- */
/////// Manipulator Config //////////
typedef struct
{
  float x_pos;
  float y_pos;
  float z_pos;
} Manipulator_Pos_Config_T;

typedef struct
{
  float x_vel;
  float y_vel;
  float z_vel;
} Manipulator_Vel_Config_T;

typedef struct
{
  float joint_1;
  float joint_2;
  float joint_3;
  float joint_4;
} Manipulator_Angle_Config_T;

typedef struct
{
  float joint1_vel;
  float joint2_vel;
  float joint3_vel;
  float joint4_vel;
} Manipulator_Angle_Vel_T;

/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ---------------------------------------- */
Manipulator_Pos_Config_T ForwardKinematicManipulator(Manipulator_Angle_Config_T angle);
Manipulator_Angle_Config_T InverseKinematicManipulator(Manipulator_Pos_Config_T man_pos);

/* End of file -------------------------------------------------------- */
#endif //