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
#include "manipulator.h"

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void setup()
{
  HW_PF_Init();
}

void loop()
{
  if (Serial.available())
  {
    String data = Serial.readStringUntil('\n');
    if (Mani_ReceiveData(data))
    {

    }
    else
    {

    }
  }
} 
/* End of file -------------------------------------------------------- */