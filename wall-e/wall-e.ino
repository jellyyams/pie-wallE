
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
bool waitToStart = true; // wait for something to send in the Serial input before starting (in order to manually start on the beat)

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

void serial_flush(void) {
  // manually flushes the serial input so that user input values aren't misread
  while (Serial.available()) Serial.read();
}

void calcStepLengthAndRanges() {

  // Calculate and set leftEye movement parameters
  leftEyeRange =  60.0*1000.0/(double(assumedServoSpeed)*double(currBPM))/2 ;
  leftEyeMin = leftEyeCenter-leftEyeRange/2;
  leftEyeMax = leftEyeCenter+leftEyeRange/2;
  leftEyeStep = int(leftEyeRange);
  nextLeftEye = leftEyePos + leftEyeDir*leftEyeStep;
  if (int(leftEyeRange) > leftEyeRangeMax) {
    leftEyeRange = leftEyeRangeMax;
  }

  // Calculate and set rightEye movement parameters
  rightEyeRange =  60.0*1000.0/(double(assumedServoSpeed)*double(currBPM))/2 ;
  rightEyeMin = rightEyeCenter-rightEyeRange/2;
  rightEyeMax = rightEyeCenter+rightEyeRange/2;
  rightEyeStep = int(rightEyeRange);
  nextRightEye = rightEyePos + rightEyeDir*rightEyeStep;
  if (int(rightEyeRange) > rightEyeRangeMax) {
    rightEyeRange = rightEyeRangeMax;
  }

  // Set loop delay, and check that it's not so small as to overload the arduino
  stepLength = leftEyeRange*assumedServoSpeed;  // eventually need to set up for sub-range movement steps
  if (stepLength < minDelay) {
    Serial.println("Error: stepLength is too small.");
    stepLength = minDelay;
  }
  
  
  
}

void nodHead() {
  // calculates and sends servo commands for one step of the nodHead dance move
  
}

void shakeHead() {
  // calculates and sends servo commands for one step of the shakeHead dance move
}

void bobHead() {
  // calculates and sends servo commands for one step of the bobHead dance move
}

void swingArms() {
  // calculates and sends servo commands for one step of the swingArms dance move
}

void tiltHead() {
  // calculates and sends servo commands for one step of the tiltHead dance move
}

void updateBPM(int val){
  Serial.println(val); 
  if (val > 350){
    currBPM = 75; 
  } else if (val > 450) {
    currBPM = 85; 
  } else {
    currBPM = 70; 
  }
}


void loop() {
  microphoneValue = analogRead(microphonePin); 
  updateBPM(microphoneValue); 
  calcStepLengthAndRanges();

  // check the current time
  currMilli = millis();
  

  
//  Serial.print("millis="); Serial.println(currMilli);


  // wait until nextMilli to start the movement step
//  while (currMilli <= nextMilli) {
//    // wait for stepLength before starting the next step
//    delay(minDelay);
//  }
//  if (abs(currMilli-nextMilli) > allowedError) {
//    Serial.print("Error: running "); Serial.print(currMilli-nextMilli); Serial.println(" ms behind");
//  }

  // calculate how far off we are from the nextMilli we were aiming for
  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
  nextMilli = currMilli + stepLength;

  // Print current time and movement parameters for debugging
  
//  Serial.print("currMilli="); Serial.println(currMilli);
//  Serial.print("stepLength="); Serial.println(stepLength);
//  Serial.print("realStepLength="); Serial.println(realStepLength);
//
//  Serial.print("leftEyePos="); Serial.println(leftEyePos);
//  Serial.print("leftEyeRealBPM="); Serial.println(leftEyeRealBPM);
//  Serial.print("leftEyeMin="); Serial.println(leftEyeMin);
//  Serial.print("leftEyeMax="); Serial.println(leftEyeMax);
//  Serial.print("leftEyeRange="); Serial.println(leftEyeRange);

   Serial.print("rightEyePos="); Serial.println(rightEyePos);
//  Serial.print("rightEyeRealBPM="); Serial.println(rightEyeRealBPM);
  Serial.print("rightEyeMin="); Serial.println(rightEyeMin);
  Serial.print("rightEyeMax="); Serial.println(rightEyeMax);
  Serial.print("rightEyeRange="); Serial.println(rightEyeRange);
  
  // if next position is outside of bounds, send back one step the opposite direction
  if (nextLeftEye > leftEyeMax ) {
    leftEyeDir = -1;
    nextLeftEye = leftEyeMax-leftEyeStep;
    leftEyeRealBPM = (1/double(currMilli-leftEyeLastSwitch))*1000*60/2; // calculate the actual time it took to complete the whole range
    leftEyeLastSwitch = currMilli;
  } else if (nextLeftEye < leftEyeMin) {
    leftEyeDir = 1;
    nextLeftEye = leftEyeMin+leftEyeStep;
    leftEyeRealBPM = (1/double(currMilli-leftEyeLastSwitch))*1000*60/2; // calculate the actual time it took to complete the whole range
    leftEyeLastSwitch = currMilli;
  }

  // write new servo position
  leftEyeServo.write(nextLeftEye);
  leftEyePos = nextLeftEye;
  nextLeftEye = leftEyePos + leftEyeDir*leftEyeStep;

  // if next position is outside of bounds, send back one step the opposite direction
  if (nextRightEye > rightEyeMax ) {
    rightEyeDir = -1;
    nextRightEye = rightEyeMax-rightEyeStep;
    rightEyeRealBPM = (1/double(currMilli-rightEyeLastSwitch))*1000*60/2; // calculate the actual time it took to complete the whole range
    rightEyeLastSwitch = currMilli;
  } else if (nextRightEye < rightEyeMin) {
    rightEyeDir = 1;
    nextRightEye = rightEyeMin+rightEyeStep;
    rightEyeRealBPM = (1/double(currMilli-rightEyeLastSwitch))*1000*60/2; // calculate the actual time it took to complete the whole range
    rightEyeLastSwitch = currMilli;
  }

  // write new servo position
  rightEyeServo.write(nextRightEye);
  rightEyePos = nextRightEye;
  nextRightEye = rightEyePos + rightEyeDir*rightEyeStep;

  
  
  delay(stepLength);
}
