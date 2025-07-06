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
  // servo_test.attach(5);  // Twist
  // servo_test.writeMicroseconds(1484);  // Home position 1484 Twist

  // servo_test.attach(6);  // Shoulder
  // servo_test.writeMicroseconds(2000);  // Home position 2000 Shoulder

  // servo_test.attach(9);  // Elbow
  // servo_test.writeMicroseconds(1000);  // Home position 1000;  // closed (folded) Elbow

  // servo_test.attach(10);  // Wrist
  // servo_test.writeMicroseconds(1444);  // Home position 1444;  // straight out Wrist //posWrist = 1575;  // straight out

  // servo_test.attach(11);  // Gripper
  // servo_test.writeMicroseconds(1200);  // Home position 1200 Gripper (open?)

  
}

void loop() {
  // do nothing - stay centered
}
