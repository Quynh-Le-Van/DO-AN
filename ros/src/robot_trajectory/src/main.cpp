/* Includes ----------------------------------------------------------- */
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "robot_trajectory/PID_v1.hpp"
#include "robot_trajectory/mobile_trajectory.hpp"

/* Private macros ----------------------------------------------------- */
#define ROS_RATE        (10)     // unit Hz

/* Private function prototypes ---------------------------------------- */
void Mobile_CurPosCallback(const geometry_msgs::Point::ConstPtr& Pos_input);

/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Function definitions ----------------------------------------------- */

int main(int argc, char **argv)
{   
    // Config ROS parameters
    ros::init(argc, argv, "RobotTrajectory");
    ros::NodeHandle nodeHandle;
    ros::Subscriber subMobilePos = nodeHandle.subscribe("/pub_mobile_pos", 1000, Mobile_CurPosCallback);
    ros::Publisher pubMobileSpeed = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Rate loop_rate(ROS_RATE);

    // Init mobile platform
    Mobile_Init();
    
    while (ros::ok()) 
    {   
        geometry_msgs::Twist mobileSpeedCommand;

        // Generate trajectory for mobile platform
        Mobile_GenerateTrajectory();

        // Publish velocities of mobile platform
        mobileSpeedCommand.linear.x = g_MobileSpeedCommand.x_vel;
        mobileSpeedCommand.linear.y = g_MobileSpeedCommand.y_vel;
        mobileSpeedCommand.angular.z = g_MobileSpeedCommand.theta_vel;
        
        pubMobileSpeed.publish(mobileSpeedCommand);

        // Set ROS parameters
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();

    return 0;
}

void Mobile_CurPosCallback(const geometry_msgs::Point::ConstPtr& Pos_input)
{
    g_MobileCurPosition.x = Pos_input->x;
    g_MobileCurPosition.y = Pos_input->y;
    g_MobileCurPosition.theta = Pos_input->z;
}
/* End of file -------------------------------------------------------- */
