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
#include "ServoEasing.hpp"

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
bool Data_Check(String str);

/* Public variables --------------------------------------------------- */
ServoEasing  g_BaseServo, g_ShoulderServo, g_EllowServo, g_ArmServo, g_GripperServo;
Manipulator_Angle_Vel_T g_JointSpeedCommand;
Manipulator_Angle_Config_T g_JointAngleCommand;
Manipulator_Pos_Config_T g_ManipulatorPos;

/* Private variables -------------------------------------------------- */
/* Function definitions ----------------------------------------------- */
void Mani_Init(void)
{
  g_BaseServo.attach(SERVO_BASE, SERVO_INIT_ANGLE);
  g_ShoulderServo.attach(SERVO_SHOULDER, SERVO_INIT_ANGLE);
  g_EllowServo.attach(SERVO_ELLOW, SERVO_INIT_ANGLE);
  g_ArmServo.attach(SERVO_ARM, SERVO_INIT_ANGLE);
  g_GripperServo.attach(SERVO_GRIPPER, SERVO_INIT_ANGLE);

  g_BaseServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_ShoulderServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_EllowServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_ArmServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_GripperServo.setSpeed(SERVO_SPEED_DEFAULT);

}

void Mani_SetSpeedJoint(Manipulator_Angle_Vel_T vel)
{
  vel.joint1_vel = RAD_TO_DEG*(vel.joint1_vel);
  vel.joint2_vel = RAD_TO_DEG*(vel.joint2_vel);
  vel.joint3_vel = RAD_TO_DEG*(vel.joint3_vel);
  vel.joint4_vel = RAD_TO_DEG*(vel.joint4_vel);

  g_BaseServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_ShoulderServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_EllowServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_ArmServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_GripperServo.setSpeed(SERVO_SPEED_DEFAULT);
}

void Mani_SetPosition(Manipulator_Pos_Config_T pos, Manipulator_Angle_Vel_T vel)
{
  Manipulator_Angle_Config_T angle = InverseKinematicManipulator(pos);

  angle.joint_1 = RAD_TO_DEG*(angle.joint_1);
  angle.joint_2 = RAD_TO_DEG*(angle.joint_2);
  angle.joint_3 = RAD_TO_DEG*(angle.joint_3);
  angle.joint_4 = RAD_TO_DEG*(angle.joint_4);                                                                                                                                                                                                                                                                                                                                                                                                       

  g_BaseServo.setEaseTo(angle.joint_1, vel.joint1_vel);
  g_ShoulderServo.setEaseTo(angle.joint_2, vel.joint2_vel);
  g_EllowServo.setEaseTo(angle.joint_3, vel.joint3_vel);
  g_GripperServo.startEaseTo(angle.joint_4, vel.joint4_vel, START_UPDATE_BY_INTERRUPT);
}

// data format: "pos:x,y,z vel:theta1_v,theta2_v,theta3_v,theta4_v"
bool Mani_ReceiveData(String data) 
{
  if (data.startsWith("pos:") && data.indexOf("vel:") != -1) {
    int posColonPos = data.indexOf(':');
    int posCommaPos = data.indexOf(',', posColonPos);

    if (posColonPos != -1 && posCommaPos != -1) {
      g_ManipulatorPos.x_pos = (data.substring(posColonPos + 1, posCommaPos)).toDouble();

      int posCommaPos2 = data.indexOf(',', posCommaPos + 1);
      g_ManipulatorPos.y_pos = (data.substring(posCommaPos + 1, posCommaPos2)).toDouble();

      int posCommaPos3 = data.indexOf(',', posCommaPos2 + 1);
      g_ManipulatorPos.z_pos = (data.substring(posCommaPos2 + 1, posCommaPos3)).toDouble();

      int velColonPos = data.indexOf("vel:");
      int velCommaPos = data.indexOf(',', velColonPos + 4);

      if (velColonPos != -1 && velCommaPos != -1) {
        g_JointSpeedCommand.joint1_vel = (data.substring(velColonPos + 4, velCommaPos)).toDouble();

        int velCommaPos2 = data.indexOf(',', velCommaPos + 1);
        g_JointSpeedCommand.joint2_vel = (data.substring(velCommaPos + 1, velCommaPos2)).toDouble();

        int velCommaPos3 = data.indexOf(',', velCommaPos2 + 1);
        g_JointSpeedCommand.joint3_vel = (data.substring(velCommaPos2 + 1, velCommaPos3)).toDouble();

        int velCommaPos4 = data.indexOf(',', velCommaPos3 + 1);
        g_JointSpeedCommand.joint4_vel = (data.substring(velCommaPos3 + 1, velCommaPos4)).toDouble();

        return true;
      }
    }
  }
  return false;
}

/* End of file -------------------------------------------------------- */