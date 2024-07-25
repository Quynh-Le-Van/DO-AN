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
#include "hw_init.h"

/* Public defines ---------------------------------------------------- */
/* Public enumerate/structure ---------------------------------------- */
/* Public macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ---------------------------------------- */
void HW_PF_Init(void)
{
    Serial.begin(57600);
    Serial2.begin(9600);

    // Timer Init
    // Timer 1
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;

    TCCR1B |= (1 << CS11);  // prescale = 8
    TCNT1  = 65400;         //
    TIMSK1 = (1 << TOIE1);  // Overflow interrupt enable

    // Timer 2
    TCCR2A = 0;
    TCCR2B = 0;
    TIMSK2 = 0;

    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  // 1024 prescaler
    TCNT2 = 99;                                         // preload timer  10ms
    TIMSK2 |= (1 << TOIE2);
    sei();

    // GPIO Init
    pinMode(MOTOR_EN_1, OUTPUT);
    pinMode(MOTOR_L_1, OUTPUT);
    pinMode(MOTOR_R_1, OUTPUT);
    pinMode(MOTOR_ENCODER_A_1, INPUT_PULLUP);
    pinMode(MOTOR_ENCODER_B_1, INPUT_PULLUP);

    digitalWrite(MOTOR_EN_1, LOW);
    digitalWrite(MOTOR_L_1, LOW);
    digitalWrite(MOTOR_R_1, LOW);

    pinMode(MOTOR_EN_2, OUTPUT);
    pinMode(MOTOR_L_2, OUTPUT);
    pinMode(MOTOR_R_2, OUTPUT);
    pinMode(MOTOR_ENCODER_A_2, INPUT_PULLUP);
    pinMode(MOTOR_ENCODER_B_2, INPUT_PULLUP);

    digitalWrite(MOTOR_EN_2, LOW);
    digitalWrite(MOTOR_L_2, LOW);
    digitalWrite(MOTOR_R_2, LOW);

    pinMode(MOTOR_EN_3, OUTPUT);
    pinMode(MOTOR_L_3, OUTPUT);
    pinMode(MOTOR_R_3, OUTPUT);
    pinMode(MOTOR_ENCODER_A_3, INPUT_PULLUP);
    pinMode(MOTOR_ENCODER_B_3, INPUT_PULLUP);

    digitalWrite(MOTOR_EN_3, LOW);
    digitalWrite(MOTOR_L_3, LOW);
    digitalWrite(MOTOR_R_3, LOW);

    pinMode(MOTOR_EN_4, OUTPUT);
    pinMode(MOTOR_L_4, OUTPUT);
    pinMode(MOTOR_R_4, OUTPUT);
    pinMode(MOTOR_ENCODER_A_4, INPUT_PULLUP);
    pinMode(MOTOR_ENCODER_B_4, INPUT_PULLUP);

    digitalWrite(MOTOR_EN_4, LOW);
    digitalWrite(MOTOR_L_4, LOW);
    digitalWrite(MOTOR_R_4, LOW);
        
    // PID init
    Mobile_PIDInit();

}

/* End of file -------------------------------------------------------- */