
#include <Servo.h>

int microphonePin = A0; 
int microphoneValue = 0; 

// servo setup
Servo leftEyeServo;     // create servo object to control wall-e's left eye and head tilt (in conjunction with rightEyeServo)
Servo rightEyeServo;    // create servo object to control wall-e's right eye and head tilt (in conjunction with leftEyeServo)
Servo headPanServo;     // create servo object to control wall-e's head shaking
Servo topNeckServo;     // create servo object to control wall-e's head nodding
Servo botNeckServo;     // create servo object to control wall-e's head bobbing (in conjunction with topNeckServo)
Servo leftArmServo;     // create servo object to control wall-e's left arm rotation
Servo rightArmServo;    // create servo object to control wall-e's right arm rotation


// Set program parameters

int currBPM = 70;    // 70-190 initialize music BPM. Will update with change in music based on mic input
bool waitToStart = false; // wait for something to send in the Serial input before starting (in order to manually start on the beat)

int minDelay = 110;
int assumedServoSpeed = 8; // ms/deg

int leftEyeRangeMax = 55;
int leftEyeCenter = 150;

int rightEyeRangeMax = 55;
int rightEyeCenter = 30;


// Initalize global variables

int headPanPos;
int topNeckPos;
int botNeckPos;
int leftArmPos;
int rightArmPos;

unsigned long stepLength = 600;      // in milliseconds, calculated from bpm
unsigned long beatStart = 0;
unsigned long oldMilli = 0;
unsigned long currMilli = 0;
unsigned long nextMilli = stepLength;        
unsigned long allowedError = 50; // milliseconds we can be off by
unsigned long realStepLength = stepLength;

double leftEyeRange = 50;
int leftEyeMin = leftEyeCenter-leftEyeRange;
int leftEyeMax = leftEyeCenter-leftEyeRange;
int leftEyePos = leftEyeCenter;
int leftEyeStep = leftEyeRange;
int leftEyeDir = 1;
int nextLeftEye = leftEyePos + leftEyeDir*leftEyeStep;
unsigned long leftEyeLastSwitch = 0;
double leftEyeRealBPM = 0;

double rightEyeRange = 50;
int rightEyeMin = rightEyeCenter-rightEyeRange;
int rightEyeMax = rightEyeCenter-rightEyeRange;
int rightEyePos = rightEyeCenter;
int rightEyeStep = rightEyeRange;
int rightEyeDir = -1;
int nextRightEye = rightEyePos + rightEyeDir*rightEyeStep;
unsigned long rightEyeLastSwitch = 0;
double rightEyeRealBPM = 0;




void setup() {
  // put your setup code here, to run once:

  // Set up serial and attach servos
  Serial.begin(9600); // open the serial port at 9600 bps:
  leftEyeServo.attach(3);  // attaches the servo on pin __ to the leftEyeServo object
  rightEyeServo.attach(5);  // attaches the servo on pin __ to the rightEyeServo object
//  headPanServo.attach(11);  // attaches the servo on pin __ to the headPanServo object
//  topNeckServo.attach(9);  // attaches the servo on pin __ to the topNeckServo object
//  botNeckServo.attach(13);  // attaches the servo on pin __ to the botNeckServo object

  // Move servos to starting positions
  leftEyePos = leftEyeCenter;
  rightEyePos = rightEyeCenter;
  leftEyeServo.write(leftEyePos);
  rightEyeServo.write(rightEyePos);
  
  // Calculate the starting ranges and step length based on BPM
  calcStepLengthAndRanges();

  // Wait to begin main loop until user inputs something if waitToStart = true (in order to manually start on beat)
  if (waitToStart) {
    while (Serial.available() == 0) {
      // Wait for User to Input Data
    }
    serial_flush();
  }

}




void loop() {

  
  delay(stepLength);
}
