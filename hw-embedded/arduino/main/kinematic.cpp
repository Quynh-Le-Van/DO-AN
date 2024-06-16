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
#define MOBILE_LENGTH (0.26)
#define MOBILE_WIDTH  (0.105)
#define WHEEL_RADIUS  (0.05)
#define LINK_ARM_1    (0.05)
#define LINK_ARM_2    (0.2)
#define LINK_ARM_3    (0.15)
#define LINK_ARM_4    (0.05)

#define RPM_TO_RAD_PER_SEC(rpm)       ((rpm) * 2 * 3.14159 / 60)
#define RAD_PER_SEC_TO_RPM(radPerSec) ((radPerSec) * 60 / (2 * 3.14159))

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
Mobile_Vel_Config_T ForwardKinematicMobileRobot(Wheel_Vel_Config_T wheel_vel)
{
  Mobile_Vel_Config_T mobile_vel;

  wheel_vel.w1_vel = RPM_TO_RAD_PER_SEC(wheel_vel.w1_vel);
  wheel_vel.w2_vel = RPM_TO_RAD_PER_SEC(wheel_vel.w2_vel);
  wheel_vel.w3_vel = RPM_TO_RAD_PER_SEC(wheel_vel.w3_vel);
  wheel_vel.w4_vel = RPM_TO_RAD_PER_SEC(wheel_vel.w4_vel);

  mobile_vel.x_vel =
    (wheel_vel.w1_vel + wheel_vel.w2_vel + wheel_vel.w3_vel + wheel_vel.w4_vel) * WHEEL_RADIUS / 4;

  mobile_vel.y_vel =
    (wheel_vel.w1_vel - wheel_vel.w2_vel + wheel_vel.w3_vel - wheel_vel.w4_vel) * WHEEL_RADIUS / 4;

  mobile_vel.theta_vel = (-wheel_vel.w1_vel + wheel_vel.w2_vel + wheel_vel.w3_vel - wheel_vel.w4_vel) *
                         WHEEL_RADIUS / (4 * (MOBILE_LENGTH + MOBILE_WIDTH));

  return mobile_vel;
}

Wheel_Vel_Config_T InverseKinematicMobileRobot(Mobile_Vel_Config_T mobile_vel)
{
  Wheel_Vel_Config_T wheel_vel;

  wheel_vel.w1_vel = (1 / WHEEL_RADIUS) * (mobile_vel.x_vel + mobile_vel.y_vel -
                                          (MOBILE_LENGTH + MOBILE_WIDTH) * mobile_vel.theta_vel);

  wheel_vel.w2_vel = (1 / WHEEL_RADIUS) * (mobile_vel.x_vel - mobile_vel.y_vel +
                                          (MOBILE_LENGTH + MOBILE_WIDTH) * mobile_vel.theta_vel);

  wheel_vel.w3_vel = (1 / WHEEL_RADIUS) * (mobile_vel.x_vel + mobile_vel.y_vel +
                                          (MOBILE_LENGTH + MOBILE_WIDTH) * mobile_vel.theta_vel);

  wheel_vel.w4_vel = (1 / WHEEL_RADIUS) * (mobile_vel.x_vel - mobile_vel.y_vel -
                                          (MOBILE_LENGTH + MOBILE_WIDTH) * mobile_vel.theta_vel);

  wheel_vel.w1_vel = RAD_PER_SEC_TO_RPM(wheel_vel.w1_vel);
  wheel_vel.w2_vel = RAD_PER_SEC_TO_RPM(wheel_vel.w2_vel);
  wheel_vel.w3_vel = RAD_PER_SEC_TO_RPM(wheel_vel.w3_vel);
  wheel_vel.w4_vel = RAD_PER_SEC_TO_RPM(wheel_vel.w4_vel);

  return wheel_vel;
}

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

  nx = man_pos.x_pos * cos(angle.joint_1) + man_pos.y_pos * sin(angle.joint_1) -
       LINK_ARM_4 * cos(angle.joint_2 + angle.joint_3 + angle.joint_4);

  ny = man_pos.z_pos - LINK_ARM_1 - LINK_ARM_4 * sin(angle.joint_2 + angle.joint_3 + angle.joint_4);

  // Calculate angle 1
  angle.joint_1 = atan2(man_pos.x_pos, man_pos.y_pos);

  // Calculate angle 1
  cos_theta3 = (sq(nx) + sq(ny) - sq(LINK_ARM_3) - sq(LINK_ARM_2)) / (2 * LINK_ARM_2 * LINK_ARM_3);
  // Option : ellow up(-) or ellow down (+)
  sin_theta3 = -sqrt(1 - sq(cos_theta3));

  angle.joint_3 = atan2(sin_theta3, cos_theta3);

  // Calculate angle 2
  cos_theta2 = (nx * (LINK_ARM_3 * cos(angle.joint_3) + LINK_ARM_2) + LINK_ARM_3 * sin(angle.joint_3) * ny) /
               (sq(LINK_ARM_3 * cos(angle.joint_3) + LINK_ARM_2) + sq(LINK_ARM_3 * sin(angle.joint_3)));

  sin_theta2 = (ny * (LINK_ARM_3 * cos(angle.joint_3) + LINK_ARM_2) - LINK_ARM_3 * sin(angle.joint_3) * nx) /
               (sq(LINK_ARM_3 * cos(angle.joint_3) + LINK_ARM_2) + sq(LINK_ARM_3 * sin(angle.joint_3)));

  angle.joint_2 = atan2(sin_theta2, cos_theta2);

  // Calculate angle 4
  angle.joint_4 = PI - angle.joint_2 - angle.joint_3;

  return angle;
}

/* End of file -------------------------------------------------------- */
