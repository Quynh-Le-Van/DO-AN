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
/* Public defines ---------------------------------------------------- */
#define MOTOR_EN       (4)
#define MOTOR_IN_1     (5)
#define MOTOR_IN_2     (6)
#define ENCODER_CH_A   (8)
#define ENCODER_CH_B   (7)

#define ENCODER_PPR    (998)
#define MOTOR_VOLREF   (24)

#define INVERSE_DIR()                   \
        do                              \
        {                               \
          digitalWrite(MOTOR_IN_1, 1);  \
          digitalWrite(MOTOR_IN_2, 0);  \
        }while(0)                       \

#define FORWARD_DIR()                   \
        do                              \
        {                               \
          digitalWrite(MOTOR_IN_1, 0);  \
          digitalWrite(MOTOR_IN_2, 1);  \
        }while(0)                       \

/* Public enumerate/structure ---------------------------------------- */
/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
uint8_t prestate = 0;
long encoderCount = 0;
long posCount = 0;
bool g_Count = true;

/* Public function prototypes ---------------------------------------- */
void Motor_ReadEncoderCallback(void);
void TransFunction_Generator(void);
float Calculate_VelCur(void);        // unit : RPM

void setup() 
{
  Serial.begin(9600);
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_IN_1, OUTPUT);
  pinMode(MOTOR_IN_2, OUTPUT);
  pinMode(ENCODER_CH_A, INPUT_PULLUP);
  pinMode(ENCODER_CH_B, INPUT_PULLUP);

  analogWrite(MOTOR_EN, 50);
  digitalWrite(MOTOR_IN_1, 1);
  digitalWrite(MOTOR_IN_2, 0);

  //Config Timer 1 
  cli();                                
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  
  TCCR1B |= (1 << CS11);    // prescale = 8
  TCNT1 = 65450;
  TIMSK1 = (1 << TOIE1);    // Overflow interrupt enable 
  sei();

  randomSeed(analogRead(0));
}

void loop() 
{
  if (g_Count)
  {
    g_Count = false;
    TransFunction_Generator();
  }
}


void Motor_ReadEncoderCallback(void)
{
  uint8_t state = 0;
  uint8_t a = digitalRead(ENCODER_CH_A);
  uint8_t b = digitalRead(ENCODER_CH_B);

  state = ((a << 2) | (b << 3));
  state |= (prestate);

  switch (state) 
  {
  case 0: case 5: case 10: case 15:
  break;

  case 1: case 7: case 8: case 14:
    encoderCount++;
  break;

  case 2: case 4: case 11: case 13:
    encoderCount--;
  break;

  case 3: case 12:
    encoderCount += 2;
  break;

  default:
    encoderCount -= 2;
  break;
  }

  prestate = (state >> 2);

  if (encoderCount >= ENCODER_PPR)
  {
    posCount++;
    encoderCount = 0;
  }
  else if (encoderCount <= -ENCODER_PPR)
  {
    posCount--;
    encoderCount = 0;
  }
}

void TransFunction_Generator(void)
{
  uint8_t numSample = 200;
  long rdNum, vol;

  for(uint8_t index = 0; index < numSample; index++)
  {
    rdNum = random(-MOTOR_VOLREF, MOTOR_VOLREF + 1);
    vol = map(rdNum, -MOTOR_VOLREF, MOTOR_VOLREF, -255, 255);

    if (vol < 0)
      INVERSE_DIR();
    else
      FORWARD_DIR();

    analogWrite(MOTOR_EN, abs(vol));

    for (int i = 0; i < 5; i++)
    {
      float vel = Calculate_VelCur();
      Serial.print(rdNum + String(" : "));
      Serial.println(vel);

      delay(200);  // sample time 0.2s
    }
  }
}


float Calculate_VelCur(void)
{
  static long preTime = 0, preEncoder = 0;
  float vel;

  vel = 60.0 * 1000.0 * ((float(encoderCount - preEncoder) / (ENCODER_PPR)) / (millis() - preTime));
  preEncoder = encoderCount;
  preTime = millis();

  return vel;
}

ISR (TIMER1_OVF_vect) 
{
  TCNT1 = 65450;
  Motor_ReadEncoderCallback();

}

/* End of file -------------------------------------------------------- */