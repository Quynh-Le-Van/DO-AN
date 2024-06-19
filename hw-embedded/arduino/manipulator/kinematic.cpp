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
#include "kinematic.h"

/* Private defines ---------------------------------------------------- */
#define LINK_ARM_1    (0.05)
#define LINK_ARM_2    (0.225)
#define LINK_ARM_3    (0.225)
#define LINK_ARM_4    (0.1)

#define RPM_TO_RAD_PER_SEC(rpm)       ((rpm) * 2 * 3.14159 / 60)
#define RAD_PER_SEC_TO_RPM(radPerSec) ((radPerSec) * 60 / (2 * 3.14159))
#define ROUND_FLOAT(x)                (round(x * 100000.0) / 100000.0)

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
Manipulator_Pos_Config_T ForwardKinematicManipulator(Manipulator_Angle_Config_T angle)
{
  Manipulator_Pos_Config_T man_pos;

  man_pos.x_pos =
    cos(angle.joint_1) * (cos(angle.joint_2) * LINK_ARM_2 + LINK_ARM_3 * cos(angle.joint_2 + angle.joint_3)) +
    LINK_ARM_3 * cos(angle.joint_1) * cos(angle.joint_2 + angle.joint_3 + angle.joint_4);

  man_pos.y_pos =
    sin(angle.joint_1) * (cos(angle.joint_2) * LINK_ARM_2 + LINK_ARM_3 * cos(angle.joint_2 + angle.joint_3)) +
    LINK_ARM_4 * sin(angle.joint_1) * cos(angle.joint_2 + angle.joint_3 + angle.joint_4);

  man_pos.z_pos = LINK_ARM_1 + LINK_ARM_2 * sin(angle.joint_2) +
                  LINK_ARM_3 * sin(angle.joint_2 + angle.joint_3) +
                  LINK_ARM_4 * sin(angle.joint_2 + angle.joint_3 + angle.joint_4);

  return man_pos;
}

Manipulator_Angle_Config_T InverseKinematicManipulator(Manipulator_Pos_Config_T man_pos)
{
  Manipulator_Angle_Config_T angle;
  float nx, ny, cos_theta3, sin_theta3, cos_theta2, sin_theta2;

  // Calculate angle 1
  if (man_pos.x_pos != 0)
    angle.joint_1 = atan2(man_pos.y_pos, man_pos.x_pos);
  else  
    angle.joint_1 = (man_pos.y_pos >= 0) ? M_PI/2 : - M_PI/2;

  nx = man_pos.x_pos * cos(angle.joint_1) + man_pos.y_pos * sin(angle.joint_1) -
       LINK_ARM_4;

  ny = man_pos.z_pos - LINK_ARM_1;

  nx = ROUND_FLOAT(nx);
  ny = ROUND_FLOAT(ny);

  // Calculate angle 3
  cos_theta3 = ((nx*nx) + (ny*ny) - (LINK_ARM_3*LINK_ARM_3) - (LINK_ARM_2*LINK_ARM_2)) / (2 * LINK_ARM_2 * LINK_ARM_3);
  // Option : ellow up(-) or ellow down (+)
  sin_theta3 = -sqrt(1 - (cos_theta3*cos_theta3));
  angle.joint_3 = atan2(sin_theta3, cos_theta3);

  // Calculate angle 2
  cos_theta2 = (nx * (LINK_ARM_3 * cos(angle.joint_3) + LINK_ARM_2) + LINK_ARM_3 * sin(angle.joint_3) * ny) / ( (LINK_ARM_3 * cos(angle.joint_3) + LINK_ARM_2)*(LINK_ARM_3 * cos(angle.joint_3) + LINK_ARM_2) + (LINK_ARM_3 * sin(angle.joint_3)) * (LINK_ARM_3 * sin(angle.joint_3)));
  sin_theta2 = sqrt(1 - (cos_theta2*cos_theta2));

  angle.joint_2 = atan2(sin_theta2, cos_theta2);

  // Calculate angle 4
  angle.joint_4 = - angle.joint_2 - angle.joint_3;

  return angle;
}

/* End of file -------------------------------------------------------- */
