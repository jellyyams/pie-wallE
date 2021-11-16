#include <Servo.h>

// servo setup
Servo leftEyeServo;     // create servo object to control wall-e's left eye and head tilt (in conjunction with rightEyeServo)
Servo rightEyeServo;    // create servo object to control wall-e's right eye and head tilt (in conjunction with leftEyeServo)
Servo headPanServo;     // create servo object to control wall-e's head shaking
Servo topNeckServo;     // create servo object to control wall-e's head nodding
Servo botNeckServo;     // create servo object to control wall-e's head bobbing (in conjunction with topNeckServo)
Servo leftArmServo;     // create servo object to control wall-e's left arm rotation
Servo rightArmServo;    // create servo object to control wall-e's right arm rotation

//30-160 deg, center 95, range 65 each direction, 130 total
int servoPos = 95;

void setup() {
  // put your setup code here, to run once:
  
  leftEyeServo.attach(3);  // attaches the servo on pin __ to the leftEyeServo object
//  rightEyeServo.attach(5);  // attaches the servo on pin __ to the rightEyeServo object
//  headPanServo.attach(6);  // attaches the servo on pin __ to the headPanServo object
//  topNeckServo.attach(9);  // attaches the servo on pin __ to the topNeckServo object
//  botNeckServo.attach(10);  // attaches the servo on pin __ to the botNeckServo object

  leftEyeServo.write(servoPos);
//  rightEyeServo.write(servoPos);
//  headPanServo.write(servoPos);
//  topNeckServo.write(servoPos);
//  botNeckServo.write(servoPos);
//  leftArmServo.write(servoPos);
//  rightArmServo.write(servoPos);


}

void loop() {
  // put your main code here, to run repeatedly:


}
