////////////////////////////////////////////////////////////////////////////////
// Analog (potentiometer) tester for Robotic Arms
// Arm #2 (mounted on Marvin SRRC bot)
// uses moveAll() to move all servos simultaneously
////////////////////////////////////////////////////////////////////////////////
#define code_version "200426_1328"

// !! be sure arm is in rest position before power on !!

/*
read 4 pots, set for servos
read PB, save servo positions when pressed
display all saved positions after PB
*/

// twist servo is HiTec HS-7950TH High Voltage Torque geared
// base servo is HiTec HS-7950TH High Voltage Torque geared
// elbow servo is HiTec HS-7950TH High Voltage Torque geared
// wrist servos is HiTec HS-5685MH HV Ultra Torque geared
// gripper is HiTec HS-422 (update to higher torque?)

// todo:

// line 564:
//RH: // will trigger immediately as switch power comes from servo dc/dc??

// add rotational servo at wrist??

/*
// servo positions below, lower is clockwise, higher is counter clockwise
servo positions for geared servo assemblies:

twist (base twist)
1000 facing right (drop sample?)
2000 facing left
rest: 1500 front & centered (1550?)

base (shoulder)
2000 back flat (rest)
1000 front ~40 deg

elbow
1000 closed (rest)
2000 open straight

wrist
1000 down
1650 straight out (rest)
2000 up

gripper
1050 closed (rest)
2000 open

  // Rest position for Marvin
  posTwist = 1484;  // front & slightly right of center to rest on bucket
  posBase  = 2000;  // back flat
  posElbow = 1000;  // closed (folded)
  posWrist = 1444;  // straight out
  //posWrist = 1575;  // straight out
  posHand  = 1200;  // closed gripper

*/

// ---------------------------------
// PWM pins UNO: 3, 5, 6, 9, 10, 11
// PWM pins MEGA/DUE: 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13

#include <Servo.h> 

Servo baseTwist;
Servo baseShoulder;
Servo armElbow;
Servo armWrist;
Servo gripperHand;

int posTwist, posTwistNew;
int posBase, posBaseNew;
int posElbow, posElbowNew;
int posWrist, posWristNew;
int posHand, posHandNew;

int moveTwistDone;
int moveBaseDone;
int moveElbowDone;
int moveWristDone;
int moveHandDone;

// combos (size & rate): 10/20 & 25, 20 & 50, 30 & 100
int stepSize = 1; // 20 seems good, lower is smoother
int stepRate = 6; // 25 for faster moves, 50 or 100? for more smoothness

// various pins for external communication:
#define PB_PIN 2           // PB used as power kill safetly switch
//#define TriggerIn 4        // remote trigger, active high to trigger arm pickup
//#define sampleSelect 7     // 1 for hooked precached sample, 0 for rock, etc. on the ground (unused??)
#define SampleCollected 8  // response to remote trigger, active high when done
#define PowerEnable 13     // pin #13 to enable dc/dc converter for arm, active low (high to shut off power)

// when PB used as power kill safetly switch:
int PB_latch = 0;  // to kill power & keep it off

// debug
int debug = 1;  // 1 = serial port messages

int read_PB_save;
int read_baseTwist;
int read_baseShoulder;
int read_armElbow;
int read_armWrist;
int read_gripperHand;

int servoSampleSet[10][5];  // to store last 10 settings from 5 servos
int sampleIndex = 0;   // to keep track of sample index


////////////////////////////////////////////////////////////////////////////////
void setup() {
////////////////////////////////////////////////////////////////////////////////

  // turn off power right away until rest position set!
  pinMode(PowerEnable, OUTPUT);     // pin 13
  digitalWrite(PowerEnable, HIGH);  // turn off the dc/dc converter for arm

  if (debug >= 1) {
    Serial.begin(115200);  // be sure serial monitor is set for 'newline' line ending??
    Serial.println();
    Serial.println("Dark Lord Robotics:"); 
    Serial.println("One Bot to Rule Them All, and in the Darkness, Bind Them..."); 
    Serial.println();
    Serial.println("arm2_analog_tester_0");
    Serial.println(code_version);
  }

  //pinMode(PB_save,INPUT_PULLUP);  // RobotGeek PB has pulldown built in

  pinMode(PB_PIN,INPUT_PULLUP);  // pin 2, to read pushbutton for manual trigger

  pinMode(SampleCollected, OUTPUT);    // pin 8
  digitalWrite(SampleCollected, LOW);  // no sample yet, LED off

  // move down to pin 3? one less power wire for gripper. 3/5/6/9/10  ???
  baseTwist.attach(5);
  baseShoulder.attach(6);
  armElbow.attach(9);
  armWrist.attach(10);
  gripperHand.attach(11);

  // Rest position for Marvin (twist centered, base back, elbow closed, wrist straight, hand closed)
  posTwist = 1484;  // front & slightly right of center to rest on bucket
  posBase  = 2000;  // back flat
  posElbow = 1000;  // closed (folded)
  posWrist = 1444;  // straight out
  //posWrist = 1575;  // straight out
  posHand  = 2000;  // open gripper
  //posHand  = 1200;  // closed gripper

  posTwistNew = posTwist;  // update new position
  posBaseNew  = posBase;   // update new position
  posElbowNew = posElbow;  // update new position
  posWristNew = posWrist;  // update new position
  posHandNew  = posHand;   // update new position

  // be sure arm is in rest position before power on!
  digitalWrite(PowerEnable, LOW);  // turn on the dc/dc converter for arm
  digitalWrite(SampleCollected, HIGH);  // LED on
  moveAll();  // be sure arm is in rest position before power on!

  if (debug >= 1) {
    Serial.println();
    Serial.println("*** setup done ***");
  }
  delay(2000);

}  // end void setup


////////////////////////////////////////////////////////////////////////////////
void loop() {
////////////////////////////////////////////////////////////////////////////////


//-----------------------------------------------
// read PB and potentiometers

  read_PB_save      = digitalRead(A0);
  read_baseTwist    = analogRead(A1);
  read_baseShoulder = analogRead(A2);
  read_armElbow     = analogRead(A3);
  read_armWrist     = analogRead(A4);
  //read_gripperHand  = analogRead(A5); // maybe later...

  digitalWrite(SampleCollected, read_PB_save);   // turn the LED on if PB = 1

  Serial.println();
  Serial.println("*** Readings ***");
  Serial.print("read_PB_save: ");
  Serial.println(read_PB_save);
  Serial.print("read_baseTwist: ");
  Serial.println(read_baseTwist);
  Serial.print("read_baseShoulder: ");
  Serial.println(read_baseShoulder);
  Serial.print("read_armElbow: ");
  Serial.println(read_armElbow);
  Serial.print("read_armWrist: ");
  Serial.println(read_armWrist);

  //posTwistNew = map(read_baseTwist, 0, 1023, 0, 1000) + 1000;  // until pot fixed
  posBaseNew  = map(read_baseShoulder, 0, 1023, 0, 1000) + 1000;
  posElbowNew = map(read_armElbow, 0, 1023, 0, 1000) + 1000;
  posWristNew = map(read_armWrist, 0, 1023, 0, 1000) + 1000;
  //posHandNew  = map(read_gripperHand, 0, 1023, 0, 1000) + 1000;

  Serial.println();
  Serial.println("*** New Servo Positions ***");
  Serial.print("posTwistNew: ");
  Serial.println(posTwistNew);
  Serial.print("posBaseNew: ");
  Serial.println(posBaseNew);
  Serial.print("posElbowNew: ");
  Serial.println(posElbowNew);
  Serial.print("posWristNew: ");
  Serial.println(posWristNew);
  Serial.print("posHandNew: ");
  Serial.println(posHandNew);

  // PB_save pressed? save current position settings
  if (read_PB_save == 1) {
    servoSampleSet[sampleIndex][0] = posTwist;
    servoSampleSet[sampleIndex][1] = posBase;
    servoSampleSet[sampleIndex][2] = posElbow;
    servoSampleSet[sampleIndex][3] = posWrist;
    servoSampleSet[sampleIndex][4] = posHand;
    sampleIndex = sampleIndex + 1;   // to keep track of sample index
    read_PB_save = 0;
    Serial.println();
    Serial.println("************************************************************");
    Serial.println("*** Saved Servo Samples ***");
    for (int i = 0; i < sampleIndex; i++) { // to cover max move range
      Serial.println();
      Serial.print("*** SampleSet: ");
      Serial.println(i);
      Serial.println(servoSampleSet[i][0]);
      Serial.println(servoSampleSet[i][1]);
      Serial.println(servoSampleSet[i][2]);
      Serial.println(servoSampleSet[i][3]);
      Serial.println(servoSampleSet[i][4]);
    }
    Serial.println();
    Serial.println("************************************************************");
    delay(5000);  // debug
  }

  // PB used as power kill safetly switch
  if (digitalRead(PB_PIN) == 0 || PB_latch == 1) {
    digitalWrite(PowerEnable, HIGH);  // turn off the dc/dc converter for arm
    PB_latch = 1;
        Serial.println();
        Serial.print("PB_PIN:  ");
        Serial.println(digitalRead(PB_PIN));
        Serial.print("PB_latch:  ");
        Serial.println(PB_latch);
        Serial.println();
        Serial.println("*** arm power off ***");
        Serial.println();
    delay(5000);  // debug
  }

  moveAll();  // move all servos to new positions simultaneously

  delay(500);

}  // end void loop


////////////////////////////////////////////////////////////////////////////////
// functions
////////////////////////////////////////////////////////////////////////////////


//-----------------------------------------------
void moveAll() {
  // move all servos to new positions simultaneously

  moveTwistDone = 0;
  moveBaseDone  = 0;
  moveElbowDone = 0;
  moveWristDone = 0;
  moveHandDone  = 0;

  for (int i = 1; i < 1000; i++) { // to cover max move range

    // PB used as power kill safety switch
    if (digitalRead(PB_PIN) == 0 || PB_latch == 1) {
      digitalWrite(PowerEnable, HIGH);  // turn off the dc/dc converter for arm
      PB_latch = 1;
      if (debug >= 1) {
        Serial.println();
        Serial.print("PB_PIN:  ");
        Serial.println(digitalRead(PB_PIN));
        Serial.print("PB_latch:  ");
        Serial.println(PB_latch);
        Serial.println();
        Serial.println("*** arm power off ***");
        Serial.println();
      }
      i = 1000;
      break;
    }

    delay(stepRate);               // waits # ms for the servo to reach the position
    // check progress, end loop if all done
    if (moveTwistDone == 1 && moveBaseDone == 1 && moveElbowDone == 1 && moveWristDone == 1 && moveHandDone == 1) {
      i = 1000;
      if (debug >= 1) {
        Serial.println();
        Serial.println("Done, set i = 1000");
      }
    }

    // set base twist servo
    if (posTwistNew > posTwist) {
      posTwist = posTwist + stepSize;  // stepSize in microseconds
    }
    else if (posTwistNew < posTwist) {
      posTwist = posTwist - stepSize;  // stepSize in microseconds
    }
    else if (posTwistNew == posTwist) {
      moveTwistDone = 1;
    }
    baseTwist.writeMicroseconds(posTwist);  // set Twist servo

    // set base servo
    if (posBaseNew > posBase) {
      posBase = posBase + stepSize;  // stepSize in microseconds
    }
    else if (posBaseNew < posBase) {
      posBase = posBase - stepSize;  // stepSize in microseconds
    }
    else if (posBaseNew == posBase) {
      moveBaseDone = 1;
    }
    baseShoulder.writeMicroseconds(posBase);  // set Shoulder servo

    // set elbow servo
    if (posElbowNew > posElbow) {
      posElbow = posElbow + stepSize;  // stepSize in microseconds
    }
    else if (posElbowNew < posElbow) {
      posElbow = posElbow - stepSize;  // stepSize in microseconds
    }
    else if (posElbowNew == posElbow) {
      moveElbowDone = 1;
    }
    armElbow.writeMicroseconds(posElbow);  // set Elbow servo

    // set wrist servo
    if (posWristNew > posWrist) {
      posWrist = posWrist + stepSize;  // stepSize in microseconds
    }
    else if (posWristNew < posWrist) {
      posWrist = posWrist - stepSize;  // stepSize in microseconds
    }
    else if (posWristNew == posWrist) {
      moveWristDone = 1;
    }
    armWrist.writeMicroseconds(posWrist);  // set Wrist servo

    // set hand servo
    if (posHandNew > posHand) {
      posHand = posHand + stepSize;  // stepSize in microseconds
    }
    else if (posHandNew < posHand) {
      posHand = posHand - stepSize;  // stepSize in microseconds
    }
    else if (posHandNew == posHand) {
      moveHandDone = 1;
    }
    gripperHand.writeMicroseconds(posHand);  // set Hand servo
  }
  if (debug >= 1) {
    Serial.println();
    Serial.println("*** moveAll done ***");
    Serial.print("posTwist: ");
    Serial.println(posTwist);
    Serial.print("posBase:  ");
    Serial.println(posBase);
    Serial.print("posElbow: ");
    Serial.println(posElbow);
    Serial.print("posWrist: ");
    Serial.println(posWrist);
    Serial.print("posHand:  ");
    Serial.println(posHand);
  }
}  // end moveAll
