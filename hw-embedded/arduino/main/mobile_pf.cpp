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
#include "mobile_pf.h"

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
#define MOTOR_INVERSE_DIR(x)                                        \
  do                                                                \
  {                                                                 \
    digitalWrite(MOTOR_PIN_LIST[x].dir_l, HIGH);                    \
    digitalWrite(MOTOR_PIN_LIST[x].dir_r, LOW);                     \
    analogWrite(MOTOR_PIN_LIST[x].enable, abs(g_MotorMobile[x].pwmOut)); \
  } while (0)

#define MOTOR_FORWARD_DIR(x)                                        \
  do                                                                \
  {                                                                 \
    digitalWrite(MOTOR_PIN_LIST[x].dir_l, LOW);                     \
    digitalWrite(MOTOR_PIN_LIST[x].dir_r, HIGH);                    \
    analogWrite(MOTOR_PIN_LIST[x].enable, abs(g_MotorMobile[x].pwmOut)); \
  } while (0)

/* Public variables --------------------------------------------------- */
Motor_Config_T g_MotorMobile[MOTOR_MOBILE_UNKNOW];
Wheel_Vel_Config_T g_MotorSpeedCommand;
Mobile_Vel_Config_T g_MobileSpeedCommand;
Mobile_Vel_Config_T g_MobileSpeedCurent;
Mobile_Pos_Config_T g_MobilePositionCurent;  
uint32_t preTimeCommand;

/* Private variables -------------------------------------------------- */
// PID MOTOR
PID motorPID1(&g_MotorMobile[MOTOR_MOBILE_1].velCurrent, &g_MotorMobile[MOTOR_MOBILE_1].pwmOut,
              &g_MotorSpeedCommand.w1_vel, MOTOR1_PID_KP, MOTOR1_PID_KI, MOTOR1_PID_KD, DIRECT);
PID motorPID2(&g_MotorMobile[MOTOR_MOBILE_2].velCurrent, &g_MotorMobile[MOTOR_MOBILE_2].pwmOut,
              &g_MotorSpeedCommand.w2_vel, MOTOR2_PID_KP, MOTOR2_PID_KI, MOTOR2_PID_KD, DIRECT);
PID motorPID3(&g_MotorMobile[MOTOR_MOBILE_3].velCurrent, &g_MotorMobile[MOTOR_MOBILE_3].pwmOut,
              &g_MotorSpeedCommand.w3_vel, MOTOR3_PID_KP, MOTOR3_PID_KI, MOTOR3_PID_KD, DIRECT);
PID motorPID4(&g_MotorMobile[MOTOR_MOBILE_4].velCurrent, &g_MotorMobile[MOTOR_MOBILE_4].pwmOut,
              &g_MotorSpeedCommand.w4_vel, MOTOR4_PID_KP, MOTOR4_PID_KI, MOTOR4_PID_KD, DIRECT);

// PID Mobile Tracking Trajectory
Mobile_Pos_Config_T mobileTrajCur;
Mobile_Pos_Config_T mobileTrajRef;
Mobile_Vel_Config_T mobileVelCmd;

PID moTrajXPID(&mobileTrajCur.x_pos, &mobileVelCmd.x_vel, &mobileTrajRef.x_pos, TRAJX_PID_KP, TRAJX_PID_KI, TRAJX_PID_KD, DIRECT);
PID moTrajXPID(&mobileTrajCur.y_pos, &mobileVelCmd.y_vel, &mobileTrajRef.y_pos, TRAJY_PID_KP, TRAJY_PID_KI, TRAJY_PID_KD, DIRECT);
PID moTrajXPID(&mobileTrajCur.theta, &mobileVelCmd.theta_vel, &mobileTrajRef.theta, TRAJTHE_PID_KP, TRAJTHE_PID_KI, TRAJTHE_PID_KD, DIRECT);

static Mobile_Pos_Config_T mobilePos;

#define INFO(_ea, _eb, _pwm, _dirl, _dirr)                                             \
  {                                                                                    \
    .encoder_A = _ea, .encoder_B = _eb, .enable = _pwm, .dir_l = _dirl, .dir_r = _dirr \
  }

Motor_Config_Pin_T MOTOR_PIN_LIST[MOTOR_MOBILE_UNKNOW] = {
  INFO(MOTOR_ENCODER_A_1, MOTOR_ENCODER_B_1, MOTOR_EN_1, MOTOR_L_1, MOTOR_R_1),
  INFO(MOTOR_ENCODER_A_2, MOTOR_ENCODER_B_2, MOTOR_EN_2, MOTOR_L_2, MOTOR_R_2),
  INFO(MOTOR_ENCODER_A_3, MOTOR_ENCODER_B_3, MOTOR_EN_3, MOTOR_L_3, MOTOR_R_3),
  INFO(MOTOR_ENCODER_A_4, MOTOR_ENCODER_B_4, MOTOR_EN_4, MOTOR_L_4, MOTOR_R_4)
};

#undef INFO

/* Private function prototypes ---------------------------------------- */
static void Motor_ReadEncoderCallback(void);
static void Motor_ReadVelocityCallBack(void);
static void Motor_SetVel(void);

/* Function definitions ----------------------------------------------- */
void Mobile_PIDInit(void)
{
  motorPID1.SetMode(AUTOMATIC);
  motorPID1.SetSampleTime(10);
  motorPID1.SetOutputLimits(-PWM_DEFAULT_VALUE, PWM_DEFAULT_VALUE);

  motorPID2.SetMode(AUTOMATIC);
  motorPID2.SetSampleTime(10);
  motorPID2.SetOutputLimits(-PWM_DEFAULT_VALUE, PWM_DEFAULT_VALUE);

  motorPID3.SetMode(AUTOMATIC);
  motorPID3.SetSampleTime(10);
  motorPID3.SetOutputLimits(-PWM_DEFAULT_VALUE, PWM_DEFAULT_VALUE);

  motorPID4.SetMode(AUTOMATIC);
  motorPID4.SetSampleTime(10);
  motorPID4.SetOutputLimits(-PWM_DEFAULT_VALUE, PWM_DEFAULT_VALUE);
}

void Mobile_SetSpeed(Mobile_Vel_Config_T speedCommand)
{
  Wheel_Vel_Config_T wheelVelCmd = InverseKinematicMobileRobot(speedCommand);
  g_MotorSpeedCommand.w1_vel     = wheelVelCmd.w1_vel;
  g_MotorSpeedCommand.w2_vel     = wheelVelCmd.w2_vel;
  g_MotorSpeedCommand.w3_vel     = wheelVelCmd.w3_vel;
  g_MotorSpeedCommand.w4_vel     = wheelVelCmd.w4_vel;
  Motor_SetVel();
}

Mobile_Vel_Config_T Mobile_ReadCurrentSpeed(void)
{
  Mobile_Vel_Config_T mobileVel;
  Wheel_Vel_Config_T wheelVel;

  wheelVel.w1_vel = g_MotorMobile[MOTOR_MOBILE_1].velCurrent;
  wheelVel.w2_vel = g_MotorMobile[MOTOR_MOBILE_2].velCurrent;
  wheelVel.w3_vel = g_MotorMobile[MOTOR_MOBILE_3].velCurrent;
  wheelVel.w4_vel = g_MotorMobile[MOTOR_MOBILE_4].velCurrent;

  mobileVel = ForwardKinematicMobileRobot(wheelVel);
  return mobileVel;
}

Mobile_Pos_Config_T Mobile_ReadCurrentPosition(void)
{
  static uint16_t preTime       = 0;
  Mobile_Vel_Config_T mobileVel = Mobile_ReadCurrentSpeed();

  mobilePos.theta += mobileVel.theta_vel * float(millis() - preTime) / 1000.0;
  mobilePos.x_pos += mobileVel.x_vel * (float(millis() - preTime) / 1000.0) * cos(mobileVel.theta_vel);
  mobilePos.y_pos += mobileVel.y_vel * (float(millis() - preTime) / 1000.0) * sin(mobileVel.theta_vel);

  return mobilePos;
}

/*
 * Note : Just call this function in timer1 interupt
 */
static void Motor_ReadEncoderCallback(void)
{
  for (uint8_t index = 0; index < MOTOR_MOBILE_UNKNOW; index++)
  {
    uint8_t state;
    uint8_t CHA = digitalRead(MOTOR_PIN_LIST[index].encoder_A);
    uint8_t CHB = digitalRead(MOTOR_PIN_LIST[index].encoder_B);

    state = ((CHA << 2) | (CHB << 3));
    state |= g_MotorMobile[index].prestate;

    switch (state)
    {
    case 0:
    case 5:
    case 10:
    case 15:
      break;

    case 1:
    case 7:
    case 8:
    case 14:
      g_MotorMobile[index].encoderCount++;
      break;

    case 2:
    case 4:
    case 11:
    case 13:
      g_MotorMobile[index].encoderCount--;
      break;

    case 3:
    case 12:
      g_MotorMobile[index].encoderCount += 2;
      break;

    default:
      g_MotorMobile[index].encoderCount -= 2;
      break;
    }

    g_MotorMobile[index].prestate = (state >> 2);

    if (g_MotorMobile[index].encoderCount > ENCODER_MAX_VALUE_PPR)
    {
      g_MotorMobile[index].posCount++;
      g_MotorMobile[index].encoderCount = 0;
    }
    else if (g_MotorMobile[index].encoderCount < -ENCODER_MAX_VALUE_PPR)
    {
      g_MotorMobile[index].posCount--;
      g_MotorMobile[index].encoderCount = 0;
    }
  }
}

static void Motor_ReadVelocityCallBack(void)
{
  for (uint8_t index = 0; index < MOTOR_MOBILE_UNKNOW; index++)
  {
    g_MotorMobile[index].velCurrent =
      60.0 * g_MotorMobile[index].encoderCount / (ENCODER_MAX_VALUE_PPR * TIMER2_SAMPLE_TIME);
    g_MotorMobile[index].encoderCount = 0;
  }
}

static void Motor_SetVel()
{
  motorPID1.Compute();
  motorPID2.Compute();
  motorPID3.Compute();
  motorPID4.Compute();

  for (uint8_t index = 0; index < MOTOR_MOBILE_UNKNOW; index++)
  {
    if (g_MotorMobile[index].pwmOut >= 0)
    {
      MOTOR_FORWARD_DIR(index);
    }
    else
    {
      MOTOR_INVERSE_DIR(index);
    }
  }
}

void Mobile_TrackingTrajectory()
{
#define SAMPLE_TIME (0.01)  // second
#define TIME_SIM    (10)    // second 
 
  for(float t = 0; i < TIME_SIM; t+=SAMPLE_TIME)
  {
    mobileTrajRef.x_pos = 1*cos(0.1*t);
    mobileTrajRef.y_pos = 1*sin(0.1*t);
    mobileTrajRef.theta = 0;

    

  }

#endif SAMPLE_TIME 
#endif TIME_SIM 
}

// Timer 1 Interupt
ISR(TIMER1_OVF_vect)
{
  TCNT1 = 65400;
  Motor_ReadEncoderCallback();
}

// Timer 2 Interupt
ISR(TIMER2_OVF_vect)
{
  TCNT2 = 99;
  Motor_ReadVelocityCallBack();
}

void Test_SetPin(double x)
{ 
  Mobile_Vel_Config_T speedCommand;
  
  speedCommand.x_vel = 0.5;
  speedCommand.y_vel = 0.5;
  speedCommand.theta_vel = 0;

  Mobile_SetSpeed(speedCommand);

  // g_MotorMobile[MOTOR_MOBILE_1].pwmOut = x;
  // g_MotorMobile[MOTOR_MOBILE_2].pwmOut = x;
  // g_MotorMobile[MOTOR_MOBILE_3].pwmOut = x;
  // g_MotorMobile[MOTOR_MOBILE_4].pwmOut = x;

  // for (uint8_t index = 0; index < MOTOR_MOBILE_UNKNOW; index++)
  // {
  //   if (g_MotorMobile[index].pwmOut >= 0)
  //   {
  //     MOTOR_FORWARD_DIR(index);
  //   }
  //   else
  //   {
  //     MOTOR_INVERSE_DIR(index);
  //   }
  // }
}
/* End of file -------------------------------------------------------- */