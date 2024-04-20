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
#ifndef _PLATFORM_H
#define _PLATFORM_H
/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include "Arduino.h"

/* Public defines ---------------------------------------------------- */
#define MAX_VALUE_8_BIT   (255)
#define PWM_DEFAULT_VALUE (200)

#define TIMER2_SAMPLE_TIME (0.01) // 10ms
#define TIMER1_SAMPLE_TIME (0.0018)

#define PUBLISHER_RATE          (10)
#define CHECK_PUBLISHER_RATE(x) (millis() - x >= 1000 / PUBLISHER_RATE)

#define MOTOR_PID_KP (1)
#define MOTOR_PID_KD (1)
#define MOTOR_PID_KI (1)

/* Public enumerate/structure ---------------------------------------- */
/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ---------------------------------------- */
/* End of file -------------------------------------------------------- */
#endif //