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
#define PWM_DEFAULT_VALUE (255)

#define TIMER2_SAMPLE_TIME (0.009984) // 10ms
#define TIMER1_SAMPLE_TIME (0.0018)

#define PUBLISHER_RATE          (10)
#define CHECK_PUBLISHER_RATE(x) (millis() - x >= 1000 / PUBLISHER_RATE)

#define MOTOR1_PID_KP (0.6)
#define MOTOR1_PID_KD (0)
#define MOTOR1_PID_KI (16)

#define MOTOR2_PID_KP (0.5)
#define MOTOR2_PID_KD (0)
#define MOTOR2_PID_KI (15)

#define MOTOR3_PID_KP (0.6)
#define MOTOR3_PID_KD (0)
#define MOTOR3_PID_KI (16)

#define MOTOR4_PID_KP (0.7)
#define MOTOR4_PID_KD (0)
#define MOTOR4_PID_KI (18)

#define TRAJX_PID_KP   (20)
#define TRAJX_PID_KI   (0)
#define TRAJX_PID_KD   (0.05)

#define TRAJY_PID_KP   (20)
#define TRAJY_PID_KI   (0)
#define TRAJY_PID_KD   (0.05)

#define TRAJTHE_PID_KP   (15)
#define TRAJTHE_PID_KI   (0)
#define TRAJTHE_PID_KD   (0)

/* Public enumerate/structure ---------------------------------------- */
/* Public macros ----------------------------------------------------- */
#define ROUND_TO_DECIMAL_PLACES(number, decimalPlaces) (round((number) * pow(10, (decimalPlaces))) / pow(10, (decimalPlaces)))

/* Public variables --------------------------------------------------- */
/* Public function prototypes ---------------------------------------- */
/* End of file -------------------------------------------------------- */
#endif //