#ifndef ROBOT_TRAJECTOR_H
#define ROBOT_TRAJECTOR_H

/* Include ----------------------------------------------------------- */
#include "robot_trajectory/PID_v1.hpp"
#include <stdint.h>
#include <math.h>

/* Public defines ---------------------------------------------------- */
#define MOBILE_MAX_SPEED  (1.5)     // unit m/s
#define MOBILE_MAX_ANGULAR   (1.0)     // unit rad/s
#define SAMPLE_TIME        (0.01)   // unit seconds


/* Public enumerate/structure ---------------------------------------- */
typedef enum
{
    MOBILE_START = 0,
    MOBILE_RUNNING,
    MOBILE_REACH_GOAL,
    ROBOT_UNKNOWN      
} Mobile_Control_Status_T;

typedef struct 
{
    double x;
    double y;
    double theta;
} Mobile_Position_T;

typedef struct
{
    double x_vel;
    double y_vel;
    double theta_vel;
} Mobile_Speed_T;

/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
extern Mobile_Speed_T g_MobileSpeedCommand;
extern Mobile_Position_T g_MobileCurPosition;

/* Public function prototypes ---------------------------------------- */
void Mobile_Init(void);
void Mobile_GenerateTrajectory(void);
void Manipulator_GenerateTrajectory(void);

/* End of file -------------------------------------------------------- */
#endif  // 