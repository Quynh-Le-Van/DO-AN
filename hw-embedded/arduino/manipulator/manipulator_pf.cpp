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
#define MAX_DEGREE        (180)
#define MIN_DEGREE        (0)
#define OFFSET_DEGREE_0   (-150)
#define OFFSET_DEGREE_180 (60)

/* Private function prototypes ---------------------------------------- */
/* Public variables --------------------------------------------------- */
Manipulator_Angle_Vel_T g_JointSpeedCommand;
Manipulator_Angle_Config_T g_JointAngleCommand;
Manipulator_Pos_Config_T g_ManipulatorPos;

/* Private variables -------------------------------------------------- */
ServoEasing g_BaseServo, g_ShoulderServo, g_EllowServo, g_ArmServo, g_GripperServo;

/* Function definitions ----------------------------------------------- */
void Mani_Init(void)
{
  g_BaseServo.attach(SERVO_BASE, DEFAULT_MICROSECONDS_FOR_0_DEGREE + OFFSET_DEGREE_0,
                     DEFAULT_MICROSECONDS_FOR_180_DEGREE + OFFSET_DEGREE_180, MIN_DEGREE - 90, MAX_DEGREE - 90);
  g_ShoulderServo.attach(SERVO_SHOULDER, DEFAULT_MICROSECONDS_FOR_0_DEGREE + OFFSET_DEGREE_0,
                         DEFAULT_MICROSECONDS_FOR_180_DEGREE + OFFSET_DEGREE_180 + 5, MIN_DEGREE, MAX_DEGREE);
  g_EllowServo.attach(SERVO_ELLOW, DEFAULT_MICROSECONDS_FOR_0_DEGREE + OFFSET_DEGREE_0,
                      DEFAULT_MICROSECONDS_FOR_180_DEGREE + OFFSET_DEGREE_180, MIN_DEGREE, MAX_DEGREE);
  g_ArmServo.attach(SERVO_ARM, DEFAULT_MICROSECONDS_FOR_0_DEGREE + OFFSET_DEGREE_0,
                    DEFAULT_MICROSECONDS_FOR_180_DEGREE + OFFSET_DEGREE_180, MIN_DEGREE, MAX_DEGREE);
  g_GripperServo.attach(SERVO_GRIPPER, DEFAULT_MICROSECONDS_FOR_0_DEGREE + OFFSET_DEGREE_0,
                        DEFAULT_MICROSECONDS_FOR_180_DEGREE + OFFSET_DEGREE_180, MIN_DEGREE, MAX_DEGREE);

  g_BaseServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_ShoulderServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_EllowServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_ArmServo.setSpeed(SERVO_SPEED_DEFAULT);
  g_GripperServo.setSpeed(SERVO_SPEED_DEFAULT);
}

void Mani_SetSpeedJoint(Manipulator_Angle_Vel_T vel)
{
  vel.joint1_vel = RAD_TO_DEG * (vel.joint1_vel);
  vel.joint2_vel = RAD_TO_DEG * (vel.joint2_vel);
  vel.joint3_vel = RAD_TO_DEG * (vel.joint3_vel);
  vel.joint4_vel = RAD_TO_DEG * (vel.joint4_vel);

  g_BaseServo.setSpeed(vel.joint1_vel);
  g_ShoulderServo.setSpeed(vel.joint2_vel);
  g_EllowServo.setSpeed(vel.joint3_vel);
  g_ArmServo.setSpeed(vel.joint4_vel);
}

void Mani_SetPosition(Manipulator_Pos_Config_T pos)
{
  Manipulator_Angle_Config_T angle = InverseKinematicManipulator(pos);

  angle.joint_1 = RAD_TO_DEG * (angle.joint_1);
  angle.joint_2 = RAD_TO_DEG * (angle.joint_2);
  angle.joint_3 = RAD_TO_DEG * (angle.joint_3);
  angle.joint_4 = RAD_TO_DEG * (angle.joint_4);

  g_BaseServo.setEasingType(EASE_CUBIC_IN_OUT);
  g_ShoulderServo.setEasingType(EASE_CUBIC_IN_OUT);
  g_EllowServo.setEasingType(EASE_CUBIC_IN_OUT);
  g_ArmServo.setEasingType(EASE_CUBIC_IN_OUT);

  g_BaseServo.startEaseTo(abs(angle.joint_1));
  g_ShoulderServo.startEaseTo(abs(angle.joint_2));
  g_EllowServo.startEaseTo(abs(angle.joint_3));
  g_ArmServo.startEaseTo(abs(angle.joint_4));

  Serial.print(String("Ac: ") + angle.joint_1 + String(", ") + angle.joint_2 + String(", ") + angle.joint_3 +
               String(", ") + angle.joint_4 + String("\n"));
  Serial.print(String("Command: ") + pos.x_pos + String(", ") + pos.y_pos + String(", ") + pos.z_pos +
               String("\n"));
  // test:
}

void Mani_Gripper(bool isGrip)
{
  g_GripperServo.setEasingType(EASE_CUBIC_IN_OUT);

  if (isGrip)
  {
    g_GripperServo.startEaseTo(MAX_DEGREE);
  }
  else
  {
    g_GripperServo.startEaseTo(MIN_DEGREE);
  }
}

// data format: "pos:x,y,z vel:theta1_v,theta2_v,theta3_v,theta4_v"
MANI_DATA_TYPE_T Mani_ReceiveData(String data)
{
  if (data.startsWith("pos:"))
  {
    data            = data.substring(4);
    int firstComma  = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    int thirdComma  = data.indexOf(',', secondComma + 1);

    g_ManipulatorPos.x_pos = data.substring(0, firstComma).toFloat();
    g_ManipulatorPos.y_pos = data.substring(firstComma + 1, secondComma).toFloat();
    g_ManipulatorPos.z_pos = data.substring(secondComma + 1, thirdComma).toFloat();

    return MANI_DATA_POS;
  }
  else if (data.startsWith("vel:"))
  {
    data            = data.substring(4);
    int firstComma  = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    int thirdComma  = data.indexOf(',', secondComma + 1);
    int fourthComma = data.indexOf(',', thirdComma + 1);

    g_JointSpeedCommand.joint1_vel = data.substring(0, firstComma).toFloat();
    g_JointSpeedCommand.joint2_vel = data.substring(firstComma + 1, secondComma).toFloat();
    g_JointSpeedCommand.joint3_vel = data.substring(secondComma + 1, thirdComma).toFloat();
    g_JointSpeedCommand.joint4_vel = data.substring(thirdComma + 1, fourthComma).toFloat();

    return MANI_DATA_VEL;
  }
  else if (data.startsWith("pick"))
  {
    return MANI_DATA_PICK;
  }
  else
  {
    return MANI_DATA_ERROR;
  }
}

/* End of file -------------------------------------------------------- */