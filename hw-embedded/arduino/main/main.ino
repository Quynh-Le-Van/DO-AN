
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
#include "main.h"
#include <ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void MobileSpeedCommandCallback(const geometry_msgs::Twist &cmdSpeedMsg);
static void ManipulatorCommandCallback(const geometry_msgs::Point &cmdPos);

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
// ROS config
ros::NodeHandle nodeHandle;
ros::Subscriber<geometry_msgs::Twist> subMobileSpeedCmd("cmd_vel", MobileSpeedCommandCallback);
ros::Subscriber<geometry_msgs::Point> subManiPos("arm_position", ManipulatorCommandCallback);

geometry_msgs::Twist MobileSpeedMsg;
ros::Publisher pubMobileSpeed("pub_mobile_speed", &MobileSpeedMsg);

geometry_msgs::Point MobilePosMsg;
ros::Publisher pubMobilePosMsg("pub_mobile_pos", &MobilePosMsg);

/* Function definitions ----------------------------------------------- */
void setup()
{
  HW_PF_Init();

  // ROS Init
  // nodeHandle.initNode();
  // nodeHandle.getHardware()->setBaud(57600);
  // nodeHandle.subscribe(subMobileSpeedCmd);
  // nodeHandle.subscribe(subManiPos);
  // nodeHandle.advertise(pubMobileSpeed);
  // nodeHandle.advertise(pubMobilePosMsg);

  // while (!nodeHandle.connected())
  // {
  //   Serial2.println("Waiting for raspberry connect ...");
  //   nodeHandle.spinOnce();
  // }

}

void loop()
{

  // Serial2.print(String("Mobile: ") + g_MobileSpeedCommand.x_vel + String(", ") + g_MobileSpeedCommand.y_vel + String(", ") + g_MobileSpeedCommand.theta_vel + String(", "));
  // Serial2.println("");

  // Serial2.print(String("Manipulator: ") + g_ManiPosCommand.x_pos + String(", ") + g_ManiPosCommand.y_pos + String(", ") + g_ManiPosCommand.z_pos + String("\n"));

  // nodeHandle.spinOnce();
  delay(2000);
  Test_SetPin(1);
}

static void MobileSpeedCommandCallback(const geometry_msgs::Twist &cmdSpeedMsg)
{
  g_MobileSpeedCommand.x_vel     = cmdSpeedMsg.linear.x;
  g_MobileSpeedCommand.y_vel     = cmdSpeedMsg.linear.y;
  g_MobileSpeedCommand.theta_vel = cmdSpeedMsg.angular.z;
}

static void ManipulatorCommandCallback(const geometry_msgs::Point &cmdPos)
{
  g_ManiPosCommand.x_pos = cmdPos.x;
  g_ManiPosCommand.y_pos = cmdPos.y;
  g_ManiPosCommand.z_pos =cmdPos.z;
}
/* End of file -------------------------------------------------------- */
