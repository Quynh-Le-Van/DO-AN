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

#define DEGREE_TO_RAD(x)       (((x) * M_PI) / (180.0))
#define PUBLISHER_RATE          (10)
#define CHECK_PUBLISHER_RATE(x) (millis() - (x) >= 1000 / PUBLISHER_RATE)

#define MOTOR1_PID_KP (0.460464335792483)
#define MOTOR1_PID_KD (0)
#define MOTOR1_PID_KI (15.4926513743787)

#define MOTOR2_PID_KP (0.460464335792483)
#define MOTOR2_PID_KD (0)
#define MOTOR2_PID_KI (15.4926513743787)

#define MOTOR3_PID_KP (0.460464335792483)
#define MOTOR3_PID_KD (0)
#define MOTOR3_PID_KI (15.4926513743787)

#define MOTOR4_PID_KP (0.460464335792483)
#define MOTOR4_PID_KD (0)
#define MOTOR4_PID_KI (15.4926513743787)

#define TRAJX_PID_KP   (16)
#define TRAJX_PID_KI   (0)
#define TRAJX_PID_KD   (0)

#define TRAJY_PID_KP   (16)
#define TRAJY_PID_KI   (0)
#define TRAJY_PID_KD   (0)

#define TRAJTHE_PID_KP   (16)
#define TRAJTHE_PID_KI   (0)
#define TRAJTHE_PID_KD   (0)

/* Public enumerate/structure ---------------------------------------- */
/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ---------------------------------------- */
/* End of file -------------------------------------------------------- */
#endif //