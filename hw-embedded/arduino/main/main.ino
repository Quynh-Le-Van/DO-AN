#include <Arduino.h>

#include "ServoEasing.hpp"

ServoEasing Servo1;
ServoEasing Servo2;

void setup() {
    Servo1.attach(9, 45);
    Servo2.attach(10, 45);
}
void loop() {
    Servo1.setEasingType(EASE_CUBIC_IN_OUT); // EASE_LINEAR is default
    Servo2.setEasingType(EASE_CUBIC_IN_OUT); // EASE_LINEAR is default

    // Servo1.easeTo(180, 40);                                 // Blocking call
    Servo1.setEaseTo(0, 40);  // Non blocking call
    Servo2.startEaseTo(90, 40, START_UPDATE_BY_INTERRUPT);  // Non blocking call
    
    // Now the servo is moving to the end position independently of your program.
    delay(5000);

    Servo1.startEaseTo(180, 40);  // Non blocking call
    Servo2.startEaseTo(0, 40, START_UPDATE_BY_INTERRUPT);  // Non blocking call

    // Servo1.easeTo(90, 40);                                 // Blocking call

    delay(5000);


}