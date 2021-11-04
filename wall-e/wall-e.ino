
#include <Servo.h>

// servo setup
Servo leftEyeServo;     // create servo object to control wall-e's left eye and head tilt (in conjunction with rightEyeServo)
Servo rightEyeServo;    // create servo object to control wall-e's right eye and head tilt (in conjunction with leftEyeServo)
Servo headPanServo;     // create servo object to control wall-e's head shaking
Servo topNeckServo;     // create servo object to control wall-e's head nodding
Servo botNeckServo;     // create servo object to control wall-e's head bobbing (in conjunction with topNeckServo)
Servo leftArmServo;     // create servo object to control wall-e's left arm rotation
Servo rightArmServo;    // create servo object to control wall-e's right arm rotation


int headPanPos;
int topNeckPos;
int botNeckPos;
int leftArmPos;
int rightArmPos;

int currBPM = 80;    // 70-190 initialize music BPM. Eventually will update with change in music
unsigned long stepLength = 600;      // in milliseconds, calculated from bpm
unsigned long beatStart = 0;
unsigned long oldMilli = 0;
unsigned long currMilli = 0;
unsigned long nextMilli = stepLength;        
unsigned long allowedError = 50; // milliseconds we can be off by

unsigned long realStepLength = stepLength;

int minDelay = 110;
int assumedServoSpeed = 8; // ms/deg

int leftEyeRangeMax = 85;
int leftEyeCenter = 90;

double leftEyeRange = 50;
int leftEyeMin = leftEyeCenter-leftEyeRange;
int leftEyeMax = leftEyeCenter-leftEyeRange;
int leftEyePos = leftEyeCenter;
int leftEyeStep = leftEyeRange;
int leftEyeDir = 1;
int nextleftEye = leftEyePos + leftEyeDir*leftEyeStep;
unsigned long leftEyeLastSwitch = 0;
double leftEyeRealBPM = 0;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400); // open the serial port at 9600 bps:
  leftEyeServo.attach(3);  // attaches the servo on pin __ to the leftEyeServo object
//  rightEyeServo.attach(10);  // attaches the servo on pin __ to the rightEyeServo object
//  headPanServo.attach(11);  // attaches the servo on pin __ to the headPanServo object
//  topNeckServo.attach(9);  // attaches the servo on pin __ to the topNeckServo object
//  botNeckServo.attach(13);  // attaches the servo on pin __ to the botNeckServo object

}
void serial_flush(void) {
  // manually flushes the serial input so that user input values aren't misread
  while (Serial.available()) Serial.read();
}

void calcStepLengthAndRanges() {
  leftEyeRange =  60.0*1000.0/(double(assumedServoSpeed)*double(currBPM))/2 ;
  leftEyeStep = int(leftEyeRange);
  stepLength = leftEyeRange*assumedServoSpeed;
  if (int(leftEyeRange) > leftEyeRangeMax) {
    leftEyeRange = leftEyeRangeMax;
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



void loop() {
  // put your main code here, to run repeatedly:
  delay(stepLength);
  // check the current time and wait until nextMili to start the next step
  currMilli = millis();
//  Serial.print("millis="); Serial.println(currMilli);

  calcStepLengthAndRanges();
  leftEyeMin = leftEyeCenter-leftEyeRange/2;
  leftEyeMax = leftEyeCenter+leftEyeRange/2;

//  while (currMilli <= nextMilli) {
//    // wait for stepLength before starting the next step
//    delay(minDelay);
//  }
//  if (abs(currMilli-nextMilli) > allowedError) {
//    Serial.print("Error: running "); Serial.print(currMilli-nextMilli); Serial.println(" ms behind");
//  }
  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
  nextMilli = currMilli + stepLength;

//  Serial.print("currMilli="); Serial.println(currMilli);
  Serial.print("stepLength="); Serial.println(stepLength);
//  Serial.print("realStepLength="); Serial.println(realStepLength);
  Serial.print("leftEyePos="); Serial.println(leftEyePos);
  Serial.print("leftEyeRealBPM="); Serial.println(leftEyeRealBPM);
//  Serial.print("leftEyeMin="); Serial.println(leftEyeMin);
//  Serial.print("leftEyeMax="); Serial.println(leftEyeMax);

  Serial.print("leftEyeRange="); Serial.println(leftEyeRange);
  
  // run or continue these dance moves
  leftEyeServo.write(nextleftEye);
  leftEyePos = nextleftEye;
  nextleftEye = leftEyePos + leftEyeDir*leftEyeStep;

  if (nextleftEye > leftEyeMax ) {
    leftEyeDir = -1;
    nextleftEye = leftEyeMax-leftEyeStep;
    leftEyeRealBPM = (1/double(currMilli-leftEyeLastSwitch))*1000*60/2;
    leftEyeLastSwitch = currMilli;
  } else if (nextleftEye < leftEyeMin) {
    leftEyeDir = 1;
    nextleftEye = leftEyeMin+leftEyeStep;
    leftEyeRealBPM = (1/double(currMilli-leftEyeLastSwitch))*1000*60/2;
    leftEyeLastSwitch = currMilli;
  }

  
  
  

  
  


}
