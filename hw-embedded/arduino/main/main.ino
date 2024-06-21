
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
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void MobileSpeedCommandCallback(const geometry_msgs::Twist &cmdSpeedMsg);
static void ManipulatorCommandCallback(const geometry_msgs::Point &cmdPos);
static void IMUPublishData(void);
static void MobileOdomPublish(void);
static void OdomCallback(const nav_msgs::Odometry &odom);
static void ControlCommandCallback(const std_msgs::Int32& msg);
static void IsGripperCallback(const std_msgs::Bool &isGripper);

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
// ROS config
ros::NodeHandle nodeHandle;
ros::Subscriber<geometry_msgs::Twist> subMobileSpeedCmd("cmd_vel", MobileSpeedCommandCallback);
ros::Subscriber<geometry_msgs::Point> subManiPos("arm_position", ManipulatorCommandCallback);
ros::Subscriber<nav_msgs::Odometry> subOdom("odom", OdomCallback);
ros::Subscriber<std_msgs::Int32> subControlCommand("control_command", ControlCommandCallback);
ros::Subscriber<std_msgs::Bool> subGripper("gripper_state", IsGripperCallback); 

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
  // nodeHandle.getHardware()->setBaud(115200);
  // nodeHandle.initNode();
  // nodeHandle.subscribe(subMobileSpeedCmd);
  // nodeHandle.subscribe(subOdom);
  // nodeHandle.subscribe(subManiPos);
  // nodeHandle.subscribe(subControlCommand);
  // nodeHandle.subscribe(subGripper);
  // nodeHandle.advertise(pubMobileSpeed);
  // nodeHandle.advertise(pubMobilePosMsg);
  // nodeHandle.advertise(pubIMUData);
  // nodeHandle.advertise(pubOdomMobile);

  // while (!nodeHandle.connected())
  // {
  //   Serial2.println("Waiting for raspberry connect ");
  //   nodeHandle.spinOnce();
  // }
}

void loop()
{

  // Serial2.println(String("Pos: ") + g_OdomPos.x_pos + String(", ") + g_OdomPos.y_pos + String(", ") + g_OdomPos.theta + String("\n"));
  // Serial.println(String("Acc: ") + g_IMU.ax + String(", ") + g_IMU.ay + String(", ") + g_IMU.az + String("\n"));

  // Serial.print(g_IMU.ax, 5);
  // Serial.println("");

  // Implement task command
  // if (g_ControlCommand == COMMAND_CONTROL_MANUAL)
  // {
  //   Mobile_ControlManual();
  //   Mobile_TransmitData((void *)(&g_IsGripper), (g_IsGripper) ? (DATA_GRIPPER_CLOSE) : (DATA_GRIPPER_OPEN));
  //   Mobile_TransmitData((void *)(&g_ManiPosCommand), DATA_POS);

  //   g_preControlCommand = g_ControlCommand;
  // }
  // else if (g_ControlCommand == COMMAND_CONTROL_AUTO)
  // {
  //   Test_SetPin(1);
  //   g_preControlCommand = g_ControlCommand;
  // }

  // // Read data from sensor if needed
  // IMU_ReadAllData(); 

  // // Publish data to ros
  // IMUPublishData();
  // MobileOdomPublish();

  // // Spin once
  // nodeHandle.spinOnce();
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

static void OdomCallback(const nav_msgs::Odometry &odom)
{
  g_OdomPos.x_pos = odom.pose.pose.position.x;
  g_OdomPos.y_pos = odom.pose.pose.position.y;
  g_OdomPos.theta = g_IMU.heading;

  g_OdomVel.x_vel = odom.twist.twist.linear.x;
  g_OdomVel.y_vel = odom.twist.twist.linear.y;
  g_OdomVel.theta_vel = odom.twist.twist.angular.z;
}

static void ControlCommandCallback(const std_msgs::Int32& msg)
{
  g_ControlCommand = msg.data; 
}

static void IsGripperCallback(const std_msgs::Bool &isGripper)
{
  g_IsGripper == isGripper.data;
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
  odom.twist.twist.linear.x = g_MobileSpeedCurent.x_vel;
  odom.twist.twist.linear.y = g_MobileSpeedCurent.y_vel;
  odom.twist.twist.linear.z = 0.0;

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;

  //angular speed from encoders
  odom.twist.twist.angular.z = g_MobileSpeedCurent.theta_vel; //TODO: use BN0500
  odom.twist.covariance[0] = 0.0001;
  odom.twist.covariance[7] = 0.0001;
  odom.twist.covariance[35] = 0.0001;

  pubOdomMobile.publish(&odom);
  nodeHandle.spinOnce();
}
/* End of file -------------------------------------------------------- */
