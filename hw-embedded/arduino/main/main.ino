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

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
// ROS config
ros::NodeHandle nodeHandle;
ros::Subscriber<geometry_msgs::Twist> subMobileSpeedCmd("cmd_vel", MobileSpeedCommandCallback);

geometry_msgs::Twist MobileSpeedMsg;
ros::Publisher pubMobileSpeed("pub_mobile_speed", &MobileSpeedMsg);

geometry_msgs::Point MobilePosMsg;
ros::Publisher pubMobilePosMsg("pub_mobile_pos", &MobilePosMsg);

/* Function definitions ----------------------------------------------- */
void setup()
{
  HW_PF_Init();

  // ROS Init
  nodeHandle.initNode();
  nodeHandle.getHardware()->setBaud(57600);
  nodeHandle.subscribe(subMobileSpeedCmd);
  nodeHandle.advertise(pubMobileSpeed);
  nodeHandle.advertise(pubMobilePosMsg);

  while (!nodeHandle.connected())
  {
    nodeHandle.spinOnce();
  }
}

void loop()
{
  if (CHECK_PUBLISHER_RATE(preTimeCommand))
  {
    Mobile_SetSpeed(g_MobileSpeedCommand);
    g_MobileSpeedCurent = Mobile_ReadCurrentSpeed();
    g_MobilePositionCurent = Mobile_ReadCurrentPosition();

    MobileSpeedMsg.linear.x = g_MobileSpeedCurent.x_vel;
    MobileSpeedMsg.linear.y = g_MobileSpeedCurent.y_vel;
    MobileSpeedMsg.angular.z = g_MobileSpeedCurent.theta_vel;
    pubMobileSpeed.publish(&MobileSpeedMsg);

    MobilePosMsg.x = g_MobilePositionCurent.x_pos;
    MobilePosMsg.y = g_MobilePositionCurent.y_pos;
    MobilePosMsg.z = g_MobilePositionCurent.theta;
    pubMobilePosMsg.publish(&MobilePosMsg);

    preTimeCommand = millis();
  }

  nodeHandle.spinOnce();
}

static void MobileSpeedCommandCallback(const geometry_msgs::Twist &cmdSpeedMsg)
{
  g_MobileSpeedCommand.x_vel     = cmdSpeedMsg.linear.x;
  g_MobileSpeedCommand.y_vel     = cmdSpeedMsg.linear.y;
  g_MobileSpeedCommand.theta_vel = cmdSpeedMsg.angular.z;
}
/* End of file -------------------------------------------------------- */