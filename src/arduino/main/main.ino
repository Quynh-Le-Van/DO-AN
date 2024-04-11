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

/* Public defines ---------------------------------------------------- */
/* Public enumerate/structure ---------------------------------------- */
/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */

void setup() {

  // put your setup code here, to run once:
  Serial.begin(9600);

  // Config Timer 1 for 1ms interval
  cli();                                
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  
  TCCR1B |= (1 << CS11) | (1 << CS10);    // prescale = 64
  TCNT1 = 40536;
  TIMSK1 = (1 << TOIE1);                  // Overflow interrupt enable 
  sei();


}

void loop() 
{
  // put your main code here, to run repeatedly:

}

ISR (TIMER1_OVF_vect) 
{
  TCNT1 = 40536;
  Motor_ReadEncoderCallback();
}
/* End of file -------------------------------------------------------- */
