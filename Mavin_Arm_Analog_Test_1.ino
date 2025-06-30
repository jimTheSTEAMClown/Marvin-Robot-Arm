// =========================================42===================================80
// Source: STEAM Clown - www.steamclown.org 
// GitHub: https://github.com/jimTheSTEAMClown/Marvin-Robot-Arm 
#define code_author "Hacker: Jim Burnham - STEAM Clown - www.steamclown.org - Engineer, Maker, Propmaster & Adrenologist" // <-- Author
// This example code is covered under the GNU Lesser General Public License v3.0 
// and any docs and lesson examples is licensed under the CC BY-NC-SA 3.0. 
// https://creativecommons.org/licenses/by-nc-sa/3.0/ 
#define code_license "GNU Lesser General Public License v3.0"
#define code_usage "CC BY-NC-SA 3.0"
// Create Date:			06/27/2025
#define code_project_name " Test Servo positions for Marvin's Arm.  Read Pot Analog values, to determine a Servo position "
#define code_name "Marvin_Arm_Analog_Test_1.ino"
#define code_description "Test Servo positions for Marvin's Arm.  Read Pot Analog values, to determine a Servo position - adaptation of Ralph's marvin code "
// Dependencies:
#define target_board " Arduino Uno R3 "
#define target_IDE " Arduino IDE 2.3.5 " 
#define code_dependencies " TBD " // List any libraries and other dependencies
// Version / Revision / Change Log:
// #define code_version "0.01" //  0.01 - Created 2025-06-30 by STEAM Clown 
#define code_version "1.0.1" // Updated with additional template items, for SVCTE Mechatronics Class - 2025-06-30 by STEAM Clown 
#define code_last_modified "2025-06-30 - by STEAM Clown "
// Additional Comments: 
//
// ============================================================================
//
// You can use comment tags like these to flag TODOs or bugs during development:
// TODO: Tune PID constants for outdoor conditions
// FIXME: Motors jitter on low battery
// NOTE: This sensor model has a max range of ~200cm

// ============================================================================
// Variables, Libraries, Flags
// ============================================================================

// ------------------------------------------
// Libraries
#include <Servo.h> 

// ------------------------------------------
// info varialble to show code Information & Usage
int info = 1;  // 1 = serial port information/usage messages, 0 = no info messages

// ------------------------------------------
// debug varialble to show code debug messages
int debug = 1;  // 1 = serial port messages,, 0 = no debug messages

// ------------------------------------------
// Global Constants & Pins

// ----------------------
// Global Variables
// ----------------------
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
// ----------------------
// Read POT values from Analog pins
int read_PB_save;
int read_baseTwist;
int read_baseShoulder;
int read_armElbow;
int read_armWrist;
int read_gripperHand;


// ----------------------
// Pin Definitions
// ----------------------
const int LED = 13; // board LED pin, to be used as some process indicator
// const int trigPin = 9; // Explain pin, module and use
// const int echoPin = 10; // Explain pin, module and use



// ----------------------
// Function Prototypes - List your functions, then impliment them after the void loop() function
// ----------------------
// void setupLED();
// void blinkLED();
void printVersion();

// ============================================================================
void setup() {
// ------------------------------------------
  /* Define stuff that needs to happen  immediately at power on:
      This could be saftey topics and settings
  */
// ------------------------------------------
  /* Initialze the Serial Consol I/O
      Here you can tell the user any safety tips, requirements, or expectations
  */  
  Serial.begin(115200);  // be sure serial monitor is set for 'newline' line ending??
  if (info >= 1) {
    printVersion();
    }
  Serial.println(" ------------------------------------------ "); 
  Serial.println(" --- Begin Of SETUP --- "); 
  Serial.println(" ------------------------------------------ ");
    
  Serial.println();
  Serial.println(" ------------------------------------------ "); 
  Serial.println(" --- Done With SETUP --- "); 
  Serial.println(" ------------------------------------------ "); 
  
  delay(2000);

}  // end void setup

// ============================================================================
void loop() {
// ------------------------------------------
//-----------------------------------------------
// read PB and potentiometers

  read_PB_save      = digitalRead(A0); //TODO: Check if I need this Analog for something else?
  read_baseTwist    = analogRead(A1);
  read_baseShoulder = analogRead(A2);
  read_armElbow     = analogRead(A3);
  read_armWrist     = analogRead(A4);
  //read_gripperHand  = analogRead(A5); // maybe later...

  // digitalWrite(SampleCollected, read_PB_save);   // turn the LED on if PB = 1

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
  delay(1000);
}

/**
 * @brief Prints Author, License, Dependencies & Code firmware version info to Serial Monitor
 */
// TODO: fix format & check spelling
void printVersion() {
  Serial.println("Author, License, Dependencies & Code firmware version info");
  Serial.println("------------------------------------------");  
  Serial.print("Code Author: ");
  Serial.println(code_author);
  Serial.print("Code / Sketch Name: ");
  Serial.println(code_name);
  Serial.println("Code Description: ");
  Serial.println(code_description);
  Serial.println(code_version);
  Serial.println(code_last_modified);
  Serial.print("Target Board: ");
  Serial.println(target_board);
  Serial.print("Target_IDE: ");
  Serial.println(target_IDE);
  Serial.println("Code Dependencies: ");
  Serial.println(code_dependencies);
  Serial.println("Code License Types: ");
  Serial.println(code_license);
  Serial.println(code_usage);
  Serial.println("------------------------------------------");  
  Serial.println(" ");  
}
