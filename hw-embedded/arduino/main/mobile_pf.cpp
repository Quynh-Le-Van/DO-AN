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
#include "math.h"
// #include "ServoEasing.hpp"

/* Private enumerate/structure ---------------------------------------- */
typedef enum
{
  MOBILE_START,
  MOBILE_GO_TO_A,
  MANI_PICK_UP,
  MOBILE_GO_TO_B,
  MANI_PUT_DOWN,
  MOBILE_GO_TO_HOME,
} MOBILE_STATE_T;

/* Private macros ----------------------------------------------------- */
#define MOTOR_INVERSE_DIR(x)                                             \
  do                                                                     \
  {                                                                      \
    digitalWrite(MOTOR_PIN_LIST[x].dir_l, HIGH);                         \
    digitalWrite(MOTOR_PIN_LIST[x].dir_r, LOW);                          \
    analogWrite(MOTOR_PIN_LIST[x].enable, abs(g_MotorMobile[x].pwmOut)); \
  } while (0)

#define MOTOR_FORWARD_DIR(x)                                             \
  do                                                                     \
  {                                                                      \
    digitalWrite(MOTOR_PIN_LIST[x].dir_l, LOW);                          \
    digitalWrite(MOTOR_PIN_LIST[x].dir_r, HIGH);                         \
    analogWrite(MOTOR_PIN_LIST[x].enable, abs(g_MotorMobile[x].pwmOut)); \
  } while (0)

/* Public variables --------------------------------------------------- */
Motor_Config_T g_MotorMobile[MOTOR_MOBILE_UNKNOW];
Wheel_Vel_Config_T g_MotorSpeedCommand;
Mobile_Vel_Config_T g_MobileSpeedCommand;
Mobile_Vel_Config_T g_MobileSpeedCurent;
Mobile_Pos_Config_T g_MobilePositionCurent;

Manipulator_Pos_Config_T g_ManiPosCommand;

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
static double yawValue, preYawValue;

PID moTrajXPID(&g_MobilePositionCurent.x_pos, &mobileVelCmd.x_vel, &mobileTrajRef.x_pos, TRAJX_PID_KP,
               TRAJX_PID_KI, TRAJX_PID_KD, DIRECT);
PID moTrajYPID(&g_MobilePositionCurent.y_pos, &mobileVelCmd.y_vel, &mobileTrajRef.y_pos, TRAJY_PID_KP,
               TRAJY_PID_KI, TRAJY_PID_KD, DIRECT);
PID moTrajTHEPID(&yawValue, &mobileVelCmd.theta_vel, &mobileTrajRef.theta, TRAJTHE_PID_KP, TRAJTHE_PID_KI,
                 TRAJTHE_PID_KD, DIRECT);

static Mobile_Pos_Config_T mobilePos;
static double timerInterval = 0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

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
static Mobile_Vel_Config_T TransferFunctionToGlobal(Mobile_Vel_Config_T localMobile, double theta);
static void MPU_Getangle(void);
static double Mobile_CheckDistance(double x, double y, double x1, double y1);

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

  moTrajXPID.SetMode(AUTOMATIC);
  moTrajXPID.SetSampleTime(1);
  moTrajXPID.SetOutputLimits(-MOBILE_MAX_LINEAR_VEL, MOBILE_MAX_LINEAR_VEL);
  moTrajXPID.SetLowFilter(false, 7558.11666728512);

  moTrajYPID.SetMode(AUTOMATIC);
  moTrajYPID.SetSampleTime(1);
  moTrajYPID.SetOutputLimits(-MOBILE_MAX_LINEAR_VEL, MOBILE_MAX_LINEAR_VEL);
  moTrajYPID.SetLowFilter(false, 0.00000138);

  moTrajTHEPID.SetMode(AUTOMATIC);
  moTrajTHEPID.SetSampleTime(1);
  moTrajTHEPID.SetOutputLimits(-MOBILE_MAX_ANGULAR_VEL, MOBILE_MAX_ANGULAR_VEL);
  moTrajTHEPID.SetLowFilter(false, 0);

  MPU_Init();

  // Servo1.attach(9, 45);
  // Servo2.attach(10, 45);
}

void MPU_Init(void)
{
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

  g_MobileSpeedCurent.x_vel     = round(g_MobileSpeedCurent.x_vel * 1000.0) / 1000.0;
  g_MobileSpeedCurent.y_vel     = round(g_MobileSpeedCurent.y_vel * 1000.0) / 1000.0;
  g_MobileSpeedCurent.theta_vel = round(g_MobileSpeedCurent.theta_vel * 1000.0) / 1000.0;

  dtheta                       = (g_MobileSpeedCurent.theta_vel * (float)(millis() - preTime) / 1000.000);
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

void Mobile_TransmitData(void *data, Data_Type_T typedata)
{
  if (typedata == DATA_POS)
  {
    Manipulator_Pos_Config_T *pos = (Manipulator_Pos_Config_T *)data;
    char tmpString[128];
    sprintf(tmpString, "pos:%f,%f,%f\n", pos->x_pos, pos->y_pos, pos->z_pos);
    Serial.print(tmpString);
  }
  else if (typedata == DATA_VEL)
  {
    Manipulator_Vel_Config_T *vel = (Manipulator_Vel_Config_T *)data;
    char tmpString[128];
    sprintf(tmpString, "vel:%f,%f,%f\n", vel->x_vel, vel->y_vel, vel->z_vel);
    Serial.print(tmpString);
  }
  else if (typedata == DATA_GRIPPER_OPEN)
  {
    Serial.println("gripperclose");
  }
  else if (typedata == DATA_GRIPPER_OPEN)
  {
    Serial.println("gripperopen");
  }
}

void Mobile_TrackingTrajectory()
{
#define SAMPLE_TIME (0.01) // second
#define TIME_SIM    (100)  // second

  static MOBILE_STATE_T mobileState = MOBILE_START;

  unsigned long startTime       = 0;
  unsigned long desiredDuration = TIME_SIM; // 10 seconds
  unsigned long currentTime     = 0;
  double tmpTime                = 0;

  switch (mobileState)
  {
  case MOBILE_START:

    // Start position 
    mobileTrajRef.x_pos = 0;
    mobileTrajRef.y_pos = 0;
    mobileTrajRef.theta = 0;

    // Calculate command velocity
    moTrajXPID.Compute();
    moTrajYPID.Compute();
    moTrajTHEPID.Compute();
    Mobile_SetSpeed(mobileVelCmd);

    mobileState = MOBILE_GO_TO_A;
    break;

  case MOBILE_GO_TO_A:
    while (1)
    {
      MPU_Getangle();
      currentTime = millis();

      if (millis() - currentTime >= 10)
      {
        tmpTime     = (double)((millis() - currentTime) / 1000.0);
        currentTime = millis();

        //  Get desired trajectoy
        mobileTrajRef.x_pos = 0;
        mobileTrajRef.y_pos = 0;
        mobileTrajRef.theta = 0;

        // Calculate command velocity
        moTrajXPID.Compute();
        moTrajYPID.Compute();
        moTrajTHEPID.Compute();
        Mobile_SetSpeed(mobileVelCmd);

        if (Mobile_CheckDistance(1, 1, g_MobilePositionCurent.x_pos, g_MobilePositionCurent.y_pos) <= 0.1)
        {
          // Get desired pose at A
          mobileTrajRef.x_pos = 0;
          mobileTrajRef.y_pos = 0;
          mobileTrajRef.theta = 0;

          // Calculate command velocity
          moTrajXPID.Compute();
          moTrajYPID.Compute();
          moTrajTHEPID.Compute();
          Mobile_SetSpeed(mobileVelCmd);

          mobileState = MANI_PICK_UP;
          break;
        }
      }
    }
    break;

  case MANI_PICK_UP:
    currentTime = millis();

    while (1)
    {
      if (millis() - currentTime >= 5000)
      {
        Manipulator_Pos_Config_T pos = { 0.55, 0, 0.05 };
        Mobile_TransmitData((void *)(&pos), DATA_POS);
        currentTime = millis();
      }
      else if (millis() - currentTime >= 10000)
      {
        char tmpStr[128] = "gripperopen";
        Mobile_TransmitData((void *)tmpStr, DATA_GRIPPER_OPEN);
        currentTime = millis();
      }
      else if (millis() - currentTime >= 15000)
      {
        char tmpStr[128] = "gripperclose";
        Mobile_TransmitData((void *)tmpStr , DATA_GRIPPER_CLOSE);
        currentTime = millis();
      }
      else if (millis() - currentTime >= 20000)
      {
        Manipulator_Pos_Config_T pos = { 0.2, 0, 0.3};
        Mobile_TransmitData((void *)(&pos), DATA_POS);
        currentTime = millis();
      }
      else if (millis() - currentTime >= 25000)
      {
        mobileState = MOBILE_GO_TO_B;
        break;
      }
    }
    break;

  case MOBILE_GO_TO_B:
    while (1)
    {
      MPU_Getangle();
      currentTime = millis();

      if (millis() - currentTime >= 10)
      {
        tmpTime     = (double)((millis() - currentTime) / 1000.0);
        currentTime = millis();

        //  Get desired trajectoy
        mobileTrajRef.x_pos = 0;
        mobileTrajRef.y_pos = 0;
        mobileTrajRef.theta = 0;

        // Calculate command velocity
        moTrajXPID.Compute();
        moTrajYPID.Compute();
        moTrajTHEPID.Compute();
        Mobile_SetSpeed(mobileVelCmd);

        if (Mobile_CheckDistance(1, 1, g_MobilePositionCurent.x_pos, g_MobilePositionCurent.y_pos) <= 0.1)
        {
          // Get desired pose at A
          mobileTrajRef.x_pos = 0;
          mobileTrajRef.y_pos = 0;
          mobileTrajRef.theta = 0;

          // Calculate command velocity
          moTrajXPID.Compute();
          moTrajYPID.Compute();
          moTrajTHEPID.Compute();
          Mobile_SetSpeed(mobileVelCmd);

          mobileState = MANI_PICK_UP;
          break;
        }
      }
    }
    break;
  
  case MANI_PUT_DOWN:
    currentTime = millis();

    while (1)
    {
      if (millis() - currentTime >= 5000)
      {
        Manipulator_Pos_Config_T pos = { 0.55, 0, 0.05 };
        Mobile_TransmitData((void *)(&pos), DATA_POS);
        currentTime = millis();
      }
      else if (millis() - currentTime >= 10000)
      {
        char tmpStr[128] = "gripperopen";
        Mobile_TransmitData((void *)tmpStr, DATA_GRIPPER_OPEN);
        currentTime = millis();
      }
      else if (millis() - currentTime >= 15000)
      {
        char tmpStr[128] = "gripperclose";
        Mobile_TransmitData((void *)tmpStr, DATA_GRIPPER_CLOSE);
        currentTime = millis();
      }
      else if (millis() - currentTime >= 20000)
      {
        Manipulator_Pos_Config_T pos = { 0.2, 0, 0.3};
        Mobile_TransmitData((void *)(&pos), DATA_POS);
        currentTime = millis();
      }
      else if (millis() - currentTime >= 25000)
      {
        mobileState = MOBILE_GO_TO_B;
        break;
      }
    }
    break; 

  case MOBILE_GO_TO_HOME:
    while (1)
    {
      MPU_Getangle();
      currentTime = millis();

      if (millis() - currentTime >= 10)
      {
        tmpTime     = (double)((millis() - currentTime) / 1000.0);
        currentTime = millis();

        //  Get desired trajectoy
        mobileTrajRef.x_pos = 0;
        mobileTrajRef.y_pos = 0;
        mobileTrajRef.theta = 0;

        // Calculate command velocity
        moTrajXPID.Compute();
        moTrajYPID.Compute();
        moTrajTHEPID.Compute();
        Mobile_SetSpeed(mobileVelCmd);

        if (Mobile_CheckDistance(1, 1, g_MobilePositionCurent.x_pos, g_MobilePositionCurent.y_pos) <= 0.1)
        {
          // Get desired pose at A
          mobileTrajRef.x_pos = 0;
          mobileTrajRef.y_pos = 0;
          mobileTrajRef.theta = 0;

          // Calculate command velocity
          moTrajXPID.Compute();
          moTrajYPID.Compute();
          moTrajTHEPID.Compute();
          Mobile_SetSpeed(mobileVelCmd);

          mobileState = MANI_PICK_UP;
          break;
        }
      }
    }
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

static Mobile_Vel_Config_T TransferFunctionToGlobal(Mobile_Vel_Config_T local, double theta)
{
  Mobile_Vel_Config_T global;

  global.x_vel = local.x_vel * cos(theta) - local.y_vel * sin(theta);
  global.y_vel = local.x_vel * sin(theta) + local.y_vel * cos(theta);

  return global;
}

static void MPU_Getangle(void)
{
}

static double Mobile_CheckDistance(double x, double y, double x1, double y1)
{
  double dx       = x1 - x;
  double dy       = y1 - y;
  double distance = sqrt(dx * dx + dy * dy);
  return distance;
}

/*-------------------Timer 1 Interupt-----------------*/
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
  x = 255;
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
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_1].velCurrent + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_2].velCurrent + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_3].velCurrent + String(", "));
  // Serial.println(g_MotorMobile[MOTOR_MOBILE_4].velCurrent);

  // Serial.print(g_MotorMobile[MOTOR_MOBILE_1].encoderCount + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_2].encoderCount + String(", "));
  // Serial.print(g_MotorMobile[MOTOR_MOBILE_3].encoderCount + String(", "));
  // Serial.println(g_MotorMobile[MOTOR_MOBILE_4].encoderCount);

  // g_MobileSpeedCurent = TransferFunctionToGlobal(Mobile_ReadCurrentSpeed(), g_MobilePositionCurent.theta);
  // Mobile_ReadCurrentPosition();
  Mobile_TrackingTrajectory();
  // MPU_Getangle();

  // Serial.print(String("Curent Position: ") + g_MobilePositionCurent.x_pos + String(", "));
  // Serial.print(g_MobilePositionCurent.y_pos + String(", "));
  // Serial.println(g_MobilePositionCurent.theta);

  // Serial.print(String("Desired Position: ") + mobileTrajRef.x_pos + String(", "));
  // Serial.print(mobileTrajRef.y_pos + String(", "));
  // Serial.println(mobileTrajRef.theta);

  // Serial.print(g_MobileSpeedCurent.x_vel + String(", "));
  // Serial.print(g_MobileSpeedCurent.y_vel + String(", "));
  // Serial.println(g_MobileSpeedCurent.theta_vel);
}
/* End of file -------------------------------------------------------- */