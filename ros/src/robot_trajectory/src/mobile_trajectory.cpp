/* Includes ----------------------------------------------------------- */
#include "robot_trajectory/mobile_trajectory.hpp"
#include "robot_trajectory/PID_v1.hpp"
#include <iostream>

/* Private macros ----------------------------------------------------- */
// Trajectory of Mobile Robot
#define MOBILE_TRAJECTORY_X(t)        (1.0 * t + 0.0)    
#define MOBILE_TRAJECTORY_Y(t)        (1.0 * t + 0.0)
#define MOBILE_TRAJECTORY_THETA(t)        (0.0)
#define MOBILE_ESTIMATE_TIME              (10.0)
#define MOBILE_ERROR                      (0.05)

// PID Configures
#define MOTOR_PID_KP     (1)
#define MOTOR_PID_KI     (2)
#define MOTOR_PID_KD     (3)

// Check conditions 
#define CHECK_MOBILE_REACH_GOAL(pos)                                               \
        (sqrt((pow(pos.x, 2) - pow(MOBILE_TRAJECTORY_X(MOBILE_ESTIMATE_TIME), 2))  \
            + (pow(pos.y, 2) - pow(MOBILE_TRAJECTORY_Y(MOBILE_ESTIMATE_TIME), 2))) \
        <= MOBILE_ERROR)                                        

/* Private function prototypes ---------------------------------------- */
/* Public variables --------------------------------------------------- */
Mobile_Speed_T g_MobileSpeedCommand;
Mobile_Position_T g_MobileCurPosition;
Mobile_Position_T mobilePosCommand;

/* Private variables -------------------------------------------------- */
Mobile_Control_Status_T robotStatus;
float timeOperating;

PID mobileXPID(&g_MobileCurPosition.x, &g_MobileSpeedCommand.x_vel,
              &mobilePosCommand.x, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, DIRECT);

PID mobileYPID(&g_MobileCurPosition.y, &g_MobileSpeedCommand.y_vel,
              &mobilePosCommand.y, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, DIRECT);
              
PID mobileZPID(&g_MobileCurPosition.theta, &g_MobileSpeedCommand.theta_vel,
              &mobilePosCommand.theta, MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, DIRECT);

/* Function definitions ----------------------------------------------- */
void Mobile_Init(void)
{
    mobileXPID.SetMode(AUTOMATIC);
    mobileXPID.SetSampleTime(1);
    mobileXPID.SetOutputLimits(-MOBILE_MAX_SPEED, MOBILE_MAX_SPEED);

    mobileYPID.SetMode(AUTOMATIC);
    mobileYPID.SetSampleTime(1);
    mobileYPID.SetOutputLimits(-MOBILE_MAX_SPEED, MOBILE_MAX_SPEED);

    mobileZPID.SetMode(AUTOMATIC);
    mobileZPID.SetSampleTime(1);
    mobileZPID.SetOutputLimits(-MOBILE_MAX_ANGULAR, MOBILE_MAX_ANGULAR);
}

void Mobile_GenerateTrajectory()
{
    // Set command position
    mobilePosCommand.x = MOBILE_TRAJECTORY_X(timeOperating);
    mobilePosCommand.y = MOBILE_TRAJECTORY_Y(timeOperating);
    mobilePosCommand.theta = MOBILE_TRAJECTORY_THETA(timeOperating);

    switch (robotStatus)
    {
    case MOBILE_START:
        timeOperating = 0;
        robotStatus = MOBILE_RUNNING;
        break;
    
    case MOBILE_RUNNING:
        mobileXPID.Compute();
        mobileXPID.Compute();
        mobileXPID.Compute();
        
        if (CHECK_MOBILE_REACH_GOAL(g_MobileCurPosition))
        {
            robotStatus = MOBILE_REACH_GOAL;
        }

        break;

    case MOBILE_REACH_GOAL:
        timeOperating = 0;
        g_MobileSpeedCommand.x_vel = 0;
        g_MobileSpeedCommand.y_vel = 0;
        g_MobileSpeedCommand.theta_vel = 0;
        break;

    default:
        break;
    }

    timeOperating += SAMPLE_TIME;
}

/* End of file -------------------------------------------------------- */
