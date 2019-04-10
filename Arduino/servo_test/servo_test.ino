/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep

 Modified by Galen Savidge 4/6/2019
 https://galensavidge.com/
*/

#include <Servo.h>

#define SERVO_PIN 9

Servo myservo;  // create servo object to control a servo

int pulse_time = 1500;    // variable to store the servo position

void setup() {
  myservo.attach(SERVO_PIN);  // attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(pulse_time);
}

void loop() {
//  for (; pulse_time < 1600; pulse_time += 1) {
//    // in steps of 1 degree
//    myservo.writeMicroseconds(pulse_time);
//    delay(10);
//  }
//  for (; pulse_time > 1400; pulse_time -= 1) {
//    // in steps of 1 degree
//    myservo.writeMicroseconds(pulse_time);
//    delay(10);
//  }
    myservo.writeMicroseconds(1600);
    delay(500);
    myservo.writeMicroseconds(1400);
    delay(500);
}
