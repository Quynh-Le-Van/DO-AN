#include "tuningPID.h"

double kp = 0.5, ki = 15, kd = 0;
byte ATuneModeRemember = 2;
bool tuning            = false;
double input = 80, output = 50, setpoint = 95.49;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID_ATune aTune(&input, &output);

static void changeAutoTune();
static void AutoTuneHelper(bool start);
static void SerialReceive();

void AutoPIDInit(void)
{
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-PWM_DEFAULT_VALUE, PWM_DEFAULT_VALUE);

  changeAutoTune();
  // Serial.begin(115200);
}

void AutoPIDLoop(void)
{
  while (1)
  {
    input = g_MotorMobile[MOTOR_MOBILE_1].velCurrentFilter;
    if (tuning)
    {
      byte val = (aTune.Runtime());
      if (val != 0)
      {
        tuning = false;
      }
      if (!tuning)
      { // we're done, set the tuning parameters
        kp = aTune.GetKp();
        ki = aTune.GetKi();
        kd = aTune.GetKd();
        myPID.SetTunings(kp, ki, kd);
        AutoTuneHelper(false);
        Serial.print(String("PID: ") + kp + String(", ") + ki + String(", ") +
              kd);
        Serial.println("");
      }
    }
    else
    {
      myPID.Compute();

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
    }

    if(tuning)
      Serial.println("tuning mode");

    Serial.print(String("Value: ") + input + String(", ") + output + String(", ") +
                 g_MotorMobile[MOTOR_MOBILE_1].velCurrentFilter);
    Serial.println("");

    Serial.print(String("PID: ") + myPID.GetKp() + String(", ") + myPID.GetKi() + String(", ") +
                 myPID.GetKd());
    Serial.println("");

    SerialReceive();
  }
}

static void AutoTuneHelper(boolean start)
{
  if (start)
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}

static void changeAutoTune()
{
  if (!tuning)
  {
    // Set the output to the desired starting frequency.
    output = 100;
    aTune.SetNoiseBand(1);
    aTune.SetOutputStep(20);
    aTune.SetLookbackSec((int)20);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { // cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

static void SerialReceive()
{
  if (Serial.available())
  {
    char b = Serial.read();
    Serial.flush();
    if ((b == '1' && !tuning) || (b != '1' && tuning))
      changeAutoTune();
  }
}
