#include <Servo.h>
/*
// Rest position for Marvin
  posTwist = 1484;  // front & slightly right of center to rest on bucket
  posBase  = 2000;  // back flat
  posElbow = 1000;  // closed (folded)
  posWrist = 1444;  // straight out
  //posWrist = 1575;  // straight out
  posHand  = 1200;  // closed gripper

  baseTwist.attach(5);
  baseShoulder.attach(6);
  armElbow.attach(9);
  armWrist.attach(10);
  gripperHand.attach(11); 
*/
Servo servo_test;

void setup() {
  servo_test.attach(5);  // control pin 9
  servo_test.writeMicroseconds(1500);  // center position
}

void loop() {
  // do nothing - stay centered
}
