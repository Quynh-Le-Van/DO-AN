
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
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void MobileSpeedCommandCallback(const geometry_msgs::Twist &cmdSpeedMsg);
static void ManipulatorCommandCallback(const geometry_msgs::Point &cmdPos);
static void IMUPublishData(void);
static void MobileOdomPublish(void);

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

// Odom config 
nav_msgs::Odometry odom;
ros::Publisher pubOdomMobile("odom_mobile", &odom);

// IMU config 
sensor_msgs::Imu imuData;
ros::Publisher pubIMUData("imu_data", &imuData);

/* Function definitions ----------------------------------------------- */
void setup()
{
  HW_PF_Init();

  // ROS Init
  nodeHandle.getHardware()->setBaud(115200);
  nodeHandle.initNode();
  nodeHandle.subscribe(subMobileSpeedCmd);
  nodeHandle.subscribe(subManiPos);
  nodeHandle.advertise(pubMobileSpeed);
  nodeHandle.advertise(pubMobilePosMsg);
  nodeHandle.advertise(pubIMUData);
  nodeHandle.advertise(pubOdomMobile);

  while (!nodeHandle.connected())
  {
    Serial2.println("Waiting for raspberry connect ");
    nodeHandle.spinOnce();
  }
}

void loop()
{

  // Serial2.print(String("Mobile: ") + g_MobileSpeedCommand.x_vel + String(", ") + g_MobileSpeedCommand.y_vel + String(", ") + g_MobileSpeedCommand.theta_vel + String(", "));
  // Serial2.println("");

  // Serial2.print(String("Manipulator: ") + g_ManiPosCommand.x_pos + String(", ") + g_ManiPosCommand.y_pos + String(", ") + g_ManiPosCommand.z_pos + String("\n"));

  Test_SetPin(1);
  IMUPublishData();
  MobileOdomPublish();
  // pubMobilePosMsg.publish(&MobilePosMsg);
  nodeHandle.spinOnce();
  // delay(1);
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

static void IMUPublishData(void)
{
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(g_IMU.heading);

  imuData.header.stamp = nodeHandle.now();
  imuData.header.frame_id = "imu_link";
  imuData.linear_acceleration.x = g_IMU.ax;
  imuData.linear_acceleration.y = g_IMU.ay;
  imuData.linear_acceleration.z = 0;

  imuData.angular_velocity.x = 0;
  imuData.angular_velocity.y = 0;
  imuData.angular_velocity.z = g_IMU.gz;

  imuData.orientation.w = odom_quat.w;
  imuData.orientation.x = odom_quat.x;
  imuData.orientation.y = odom_quat.y;
  imuData.orientation.z = odom_quat.z;

  pubIMUData.publish(&imuData);
  nodeHandle.spinOnce();
}

static void MobileOdomPublish(void)
{
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(g_IMU.heading);

  odom.header.stamp = nodeHandle.now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  
  //robot's position in x,y, and z
  odom.pose.pose.position.x = g_MobilePositionCurent.x_pos;
  odom.pose.pose.position.y = g_MobilePositionCurent.y_pos;
  odom.pose.pose.position.z = 0.0;
  
  //robot's heading in quaternion
  odom.pose.pose.orientation.x = odom_quat.x;
  odom.pose.pose.orientation.y = odom_quat.y;
  odom.pose.pose.orientation.z = odom_quat.z;
  odom.pose.pose.orientation.w = odom_quat.w;
  odom.pose.covariance[0] = 0.001;
  odom.pose.covariance[7] = 0.001;
  odom.pose.covariance[35] = 0.001;

  //linear speed from encoders
  odom.twist.twist.linear.x = g_MobileSpeedCommand.x_vel;
  odom.twist.twist.linear.y = g_MobileSpeedCommand.y_vel;
  odom.twist.twist.linear.z = 0.0;

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;

  //angular speed from encoders
  odom.twist.twist.angular.z = g_MobileSpeedCommand.theta_vel;
  odom.twist.covariance[0] = 0.0001;
  odom.twist.covariance[7] = 0.0001;
  odom.twist.covariance[35] = 0.0001;

  pubOdomMobile.publish(&odom);
  nodeHandle.spinOnce();
}
/* End of file -------------------------------------------------------- */
