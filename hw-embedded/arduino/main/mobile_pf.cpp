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
#include <Math.h>

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  MOBILE_START,
  MOBILE_ONGOING,
  MOBILE_STOP
} MOBILE_STATE_T;

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
Motor_Config_T g_MotorMobile[MOTOR_MOBILE_UNKNOW];
Wheel_Vel_Config_T g_MotorSpeedCommand;
Mobile_Vel_Config_T g_MobileSpeedCommand;
Mobile_Vel_Config_T g_MobileSpeedCurent;
Mobile_Pos_Config_T g_MobilePositionCurent;
uint32_t preTimeCommand;

/* Private variables -------------------------------------------------- */
// PID MOTOR
PID motorPID1(&g_MotorMobile[MOTOR_MOBILE_1].velCurrentFilter, &g_MotorMobile[MOTOR_MOBILE_1].pwmOut,
              &g_MotorSpeedCommand.w1_vel, MOTOR1_PID_KP, MOTOR1_PID_KI, MOTOR1_PID_KD, DIRECT);
PID motorPID2(&g_MotorMobile[MOTOR_MOBILE_2].velCurrentFilter, &g_MotorMobile[MOTOR_MOBILE_2].pwmOut,
              &g_MotorSpeedCommand.w2_vel, MOTOR2_PID_KP, MOTOR2_PID_KI, MOTOR2_PID_KD, DIRECT);
PID motorPID3(&g_MotorMobile[MOTOR_MOBILE_3].velCurrentFilter, &g_MotorMobile[MOTOR_MOBILE_3].pwmOut,
              &g_MotorSpeedCommand.w3_vel, MOTOR3_PID_KP, MOTOR3_PID_KI, MOTOR3_PID_KD, DIRECT);
PID motorPID4(&g_MotorMobile[MOTOR_MOBILE_4].velCurrentFilter, &g_MotorMobile[MOTOR_MOBILE_4].pwmOut,
              &g_MotorSpeedCommand.w4_vel, MOTOR4_PID_KP, MOTOR4_PID_KI, MOTOR4_PID_KD, DIRECT);

// PID Mobile Tracking Trajectory
Mobile_Pos_Config_T mobileTrajCur;
Mobile_Pos_Config_T mobileTrajRef;
Mobile_Vel_Config_T mobileVelCmd;

PID moTrajXPID(&g_MobilePositionCurent.x_pos, &mobileVelCmd.x_vel, &mobileTrajRef.x_pos, TRAJX_PID_KP,
               TRAJX_PID_KI, TRAJX_PID_KD, DIRECT);
PID moTrajYPID(&g_MobilePositionCurent.y_pos, &mobileVelCmd.y_vel, &mobileTrajRef.y_pos, TRAJY_PID_KP,
               TRAJY_PID_KI, TRAJY_PID_KD, DIRECT);
PID moTrajTHEPID(&g_MobilePositionCurent.theta, &mobileVelCmd.theta_vel, &mobileTrajRef.theta, TRAJTHE_PID_KP,
                 TRAJTHE_PID_KI, TRAJTHE_PID_KD, DIRECT);

static Mobile_Pos_Config_T mobilePos;
static double timerInterval = 0;

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

// Auto tuning PID config
PIDAutotuner tuner = PIDAutotuner();
float tuningTargetInputValue = 95.49, tunningLoopInterval = 10000, tuningmicroseconds;
float kp, kd, ki;

/* Private function prototypes ---------------------------------------- */
static void Motor_ReadEncoderCallback(void);
static void Motor_ReadVelocityCallBack(void);
static void Motor_SetVel(void);
static Mobile_Vel_Config_T TransferFunctionToGlobal(Mobile_Vel_Config_T localMobile, double theta);
static void TuningPID_Init(void);
static void TuningPID_Loop(void);

/* Function definitions ----------------------------------------------- */
void Mobile_PIDInit(void)
{
  motorPID1.SetMode(AUTOMATIC);
  motorPID1.SetSampleTime(1);
  motorPID1.SetOutputLimits(-PWM_DEFAULT_VALUE, PWM_DEFAULT_VALUE);

  motorPID2.SetMode(AUTOMATIC);
  motorPID2.SetSampleTime(1);
  motorPID2.SetOutputLimits(-PWM_DEFAULT_VALUE, PWM_DEFAULT_VALUE);

  motorPID3.SetMode(AUTOMATIC);
  motorPID3.SetSampleTime(1);
  motorPID3.SetOutputLimits(-PWM_DEFAULT_VALUE, PWM_DEFAULT_VALUE);

  motorPID4.SetMode(AUTOMATIC);
  motorPID4.SetSampleTime(1);
  motorPID4.SetOutputLimits(-PWM_DEFAULT_VALUE, PWM_DEFAULT_VALUE);

  moTrajXPID.SetMode(AUTOMATIC);
  moTrajXPID.SetSampleTime(1);
  moTrajXPID.SetOutputLimits(-MOBILE_MAX_LINEAR_VEL, MOBILE_MAX_LINEAR_VEL);

  moTrajYPID.SetMode(AUTOMATIC);
  moTrajYPID.SetSampleTime(1);
  moTrajYPID.SetOutputLimits(-MOBILE_MAX_LINEAR_VEL, MOBILE_MAX_LINEAR_VEL);

  moTrajTHEPID.SetMode(AUTOMATIC);
  moTrajTHEPID.SetSampleTime(10);
  moTrajTHEPID.SetOutputLimits(-MOBILE_MAX_ANGULAR_VEL, MOBILE_MAX_ANGULAR_VEL);
  TuningPID_Init();
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
  static float preTime = 0;
  float dx, dy, dtheta;

  // g_MobileSpeedCurent.x_vel = round(g_MobileSpeedCurent.x_vel * 1000) / 1000.0;
  // g_MobileSpeedCurent.y_vel = round(g_MobileSpeedCurent.y_vel * 1000) / 1000.0;
  // g_MobileSpeedCurent.theta_vel = round(g_MobileSpeedCurent.theta_vel * 1000) / 1000.0;
  g_MobileSpeedCurent = Mobile_ReadCurrentSpeed();

  dtheta                       = (g_MobileSpeedCurent.theta_vel * (float)(millis() - preTime) / 1000.0);
  g_MobilePositionCurent.theta = g_MobilePositionCurent.theta + dtheta;

  g_MobileSpeedCurent = TransferFunctionToGlobal(Mobile_ReadCurrentSpeed(), g_MobilePositionCurent.theta);

  dx = (g_MobileSpeedCurent.x_vel * (float)(millis() - preTime) / 1000.0);
  dy = (g_MobileSpeedCurent.y_vel * (float)(millis() - preTime) / 1000.0);

  g_MobilePositionCurent.x_pos = g_MobilePositionCurent.x_pos + dx;
  g_MobilePositionCurent.y_pos = g_MobilePositionCurent.y_pos + dy;

  // Serial.print(dx, 5);
  // Serial.println(String(""));

  preTime = millis();
  return g_MobilePositionCurent;
}

void Mobile_TrackingTrajectory()
{
#define SAMPLE_TIME (0.01) // second
#define TIME_SIM    (100)  // second

  static MOBILE_STATE_T mobileState = MOBILE_START;

  unsigned long startTime       = 0;
  unsigned long desiredDuration = TIME_SIM; // 10 seconds
  unsigned long currentTime     = millis();
  double tmpTime                = 0;
  double interval               = 0;
  switch (mobileState)
  {
  case MOBILE_START:
    mobileState = MOBILE_ONGOING;
    break;

  case MOBILE_ONGOING:
    while (1)
    {
      if (millis() - currentTime >= 10)
      {
        tmpTime     = (double)((millis() - currentTime) / 1000.0);
        currentTime = millis();

        interval += tmpTime;

        //  Get desired trajectoy
        mobileTrajRef.x_pos = 0;
        mobileTrajRef.y_pos = 0;
        mobileTrajRef.theta = interval;

        // Calculate command velocity
        moTrajXPID.Compute();
        moTrajYPID.Compute();
        moTrajTHEPID.Compute();
        Mobile_SetSpeed(mobileVelCmd);

        // Serial.println(String("Desired: ") + mobileTrajRef.x_pos + String(", ") + mobileTrajRef.y_pos +
        // String(", "));
        Serial.println(String("Actual: ") + g_MobilePositionCurent.x_pos + String(", ") +
                       g_MobilePositionCurent.y_pos + String(", ") + g_MobilePositionCurent.theta);
        Serial.println(String("Desired vel: ") + mobileVelCmd.x_vel + String(", ") + mobileVelCmd.y_vel +
                       String(", ") + mobileVelCmd.theta_vel);
        // Serial.println(String("Actual vel: ") + g_MobileSpeedCommand.x_vel + String(", ") +
        // g_MobileSpeedCommand.y_vel + String(", ") + g_MobileSpeedCommand.theta_vel);

        // Serial.println(g_MobilePositionCurent.theta);
        // delay(10);
        // Check reach goal
        // if (g_MobilePositionCurent.theta >= 1.57)
        // {
        //   //  Get desired trajectoy
        //   mobileTrajRef.x_pos = 0;
        //   mobileTrajRef.y_pos = 0;
        //   mobileTrajRef.theta = 1.57;

        //   // Calculate command velocity
        //   moTrajXPID.Compute();
        //   moTrajYPID.Compute();
        //   moTrajTHEPID.Compute();
        //   Mobile_SetSpeed(mobileVelCmd);
        //   break;
        // }
      }
    }

    mobileState = MOBILE_STOP;
    break;

  case MOBILE_STOP:
    // Mobile_Vel_Config_T speedCommand;
    // speedCommand.x_vel     = 0;
    // speedCommand.y_vel     = 0;
    // speedCommand.theta_vel = 0;
    // Mobile_SetSpeed(speedCommand);
    //  Get desired trajectoy
    mobileTrajRef.x_pos = 0;
    mobileTrajRef.y_pos = 0;
    mobileTrajRef.theta = 1.57;

    // Calculate command velocity
    moTrajXPID.Compute();
    moTrajYPID.Compute();
    moTrajTHEPID.Compute();
    Mobile_SetSpeed(mobileVelCmd);
    break;

  default:
    break;
  }

#undef SAMPLE_TIME
#undef TIME_SIM
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

    g_MotorMobile[index].velCurrentFilter = 0.854 * g_MotorMobile[index].velCurrentFilter +
                                            0.0728 * g_MotorMobile[index].velCurrent +
                                            0.0728 * g_MotorMobile[index].preVel;

    g_MotorMobile[index].preVel       = g_MotorMobile[index].velCurrent;
    g_MotorMobile[index].encoderCount = 0;
  }
}

static void Motor_SetVel()
{
  motorPID1.Compute();
  motorPID2.Compute();
  motorPID3.Compute();
  motorPID4.Compute();

  g_MotorMobile[MOTOR_MOBILE_3].pwmOut = 1.3 * g_MotorMobile[MOTOR_MOBILE_3].pwmOut;

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

static Mobile_Vel_Config_T TransferFunctionToGlobal(Mobile_Vel_Config_T local, double theta)
{
  Mobile_Vel_Config_T global;

  global.x_vel = local.x_vel * cos(theta) - local.y_vel * sin(theta);
  global.y_vel = local.x_vel * sin(theta) + local.y_vel * cos(theta);

  return global;
}

static void TuningPID_Init()
{
  tuner.setTargetInputValue(tuningTargetInputValue);
  tuner.setOutputRange(0, 255);
  tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);
  tuner.startTuningLoop(micros());
}

static void TuningPID_Loop(void)
{
  tuner.startTuningLoop(micros());

  while (!tuner.isFinished())
  {
    // This loop must run at the same speed as the PID control loop being tuned
    long prevMicroseconds = tuningmicroseconds;
    tuningmicroseconds          = micros();

    // Get input value here (temperature, encoder position, velocity, etc)
    double input = g_MotorMobile[MOTOR_MOBILE_1].velCurrentFilter;

    // Call tunePID() with the input value and current time in tuningmicroseconds
    double output = tuner.tunePID(input, tuningmicroseconds);

    // Set the output - tunePid() will return values within the range configured
    // by setOutputRange(). Don't change the value or the tuning results will be
    // incorrect.
    g_MotorMobile[MOTOR_MOBILE_1].pwmOut = output;
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
    // This loop must run at the same speed as the PID control loop being tuned
    while (micros() - tuningmicroseconds < tunningLoopInterval)
      delayMicroseconds(1);
  }

  Serial.print(String("Value: ") + tuningTargetInputValue + String(", ") + g_MotorMobile[MOTOR_MOBILE_1].velCurrentFilter);
  Serial.println("");

  Serial.print(String("PID: ") + tuner.getKp() + String(", ") + tuner.getKi() + String(", ") +
                tuner.getKd());
  Serial.println("");
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
  timerInterval += 0.01;
  Motor_ReadVelocityCallBack();
  Mobile_ReadCurrentPosition();
}

void Test_SetPin(double x)
{
  // x = 255;
  // Mobile_Vel_Config_T speedCommand;

  // speedCommand.x_vel     = 0.5;
  // speedCommand.y_vel     = 0;
  // speedCommand.theta_vel = 0;
  // Mobile_SetSpeed(speedCommand);

  // Serial.print(g_MotorSpeedCommand.w1_vel + String(", "));
  // Serial.print(g_MotorSpeedCommand.w2_vel + String(", "));
  // Serial.print(g_MotorSpeedCommand.w3_vel + String(", "));
  // Serial.println(g_MotorSpeedCommand.w4_vel);

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
  // Serial.print(0 + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_1].velCurrent + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_2].velCurrent + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_3].velCurrent + String(", "));
  // Serial.println(g_MotorMobile[MOTOR_MOBILE_4].velCurrent);

  // Serial.print(g_MotorMobile[MOTOR_MOBILE_1].velCurrent + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_2].velCurrent + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_3].velCurrent + String(", "));
  // Serial.println(g_MotorMobile[MOTOR_MOBILE_4].velCurrentFilter);

  // Serial.print(g_MotorMobile[MOTOR_MOBILE_1].encoderCount + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_2].encoderCount + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_3].encoderCount + String(", "));
  // Serial.println(g_MotorMobile[MOTOR_MOBILE_4].encoderCount);

  // g_MobileSpeedCurent = TransferFunctionToGlobal(Mobile_ReadCurrentSpeed(), g_MobilePositionCurent.theta);
  // Mobile_ReadCurrentPosition();
  // Mobile_TrackingTrajectory();
  TuningPID_Loop();
  // Serial.print(String("Curent Position: ") + g_MobilePositionCurent.x_pos + String(", "));
  // Serial.print(g_MobilePositionCurent.y_pos + String(", "));
  // Serial.println(g_MobilePositionCurent.theta);

  // Serial.print(String("Desired Position: ") + mobileTrajRef.x_pos + String(", "));
  // Serial.print(mobileTrajRef.y_pos + String(", "));
  // Serial.println(mobileTrajRef.theta);

  // Serial.print(g_MobileSpeedCurent.x_vel + String(", "));
  // Serial.print(g_MobileSpeedCurent.y_vel + String(", "));
  // Serial.print(g_MobileSpeedCurent.theta_vel,4);
  // Serial.println("");
}
/* End of file -------------------------------------------------------- */