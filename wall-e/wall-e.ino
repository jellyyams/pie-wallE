
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

bool waitToStart = true;

int headPanPos;
int topNeckPos;
int botNeckPos;
int leftArmPos;
int rightArmPos;

int currBPM = 70;    // 70-190 initialize music BPM. Eventually will update with change in music
unsigned long stepLength = 600;      // in milliseconds, calculated from bpm
unsigned long beatStart = 0;
unsigned long oldMilli = 0;
unsigned long currMilli = 0;
unsigned long nextMilli = stepLength;        
unsigned long allowedError = 50; // milliseconds we can be off by

unsigned long realStepLength = stepLength;

int minDelay = 110;
int assumedServoSpeed = 8; // ms/deg

int leftEyeRangeMax = 55;
int leftEyeCenter = 150;

double leftEyeRange = 50;
int leftEyeMin = leftEyeCenter-leftEyeRange;
int leftEyeMax = leftEyeCenter-leftEyeRange;
int leftEyePos = leftEyeCenter;
int leftEyeStep = leftEyeRange;
int leftEyeDir = 1;
int nextLeftEye = leftEyePos + leftEyeDir*leftEyeStep;
unsigned long leftEyeLastSwitch = 0;
double leftEyeRealBPM = 0;


int rightEyeRangeMax = 55;
int rightEyeCenter = 30;

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
  Serial.begin(9600); // open the serial port at 9600 bps:
  leftEyeServo.attach(3);  // attaches the servo on pin __ to the leftEyeServo object
  rightEyeServo.attach(5);  // attaches the servo on pin __ to the rightEyeServo object
//  headPanServo.attach(11);  // attaches the servo on pin __ to the headPanServo object
//  topNeckServo.attach(9);  // attaches the servo on pin __ to the topNeckServo object
//  botNeckServo.attach(13);  // attaches the servo on pin __ to the botNeckServo object

  leftEyePos = leftEyeCenter;
  rightEyePos = rightEyeCenter;

  leftEyeServo.write(leftEyePos);
  rightEyeServo.write(rightEyePos);
  

  calcStepLengthAndRanges();


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
  leftEyeRange =  60.0*1000.0/(double(assumedServoSpeed)*double(currBPM))/2 ;
  leftEyeMin = leftEyeCenter-leftEyeRange/2;
  leftEyeMax = leftEyeCenter+leftEyeRange/2;
  leftEyeStep = int(leftEyeRange);
  nextLeftEye = leftEyePos + leftEyeDir*leftEyeStep;

  if (int(leftEyeRange) > leftEyeRangeMax) {
    leftEyeRange = leftEyeRangeMax;
  }

  rightEyeRange =  60.0*1000.0/(double(assumedServoSpeed)*double(currBPM))/2 ;
  rightEyeMin = rightEyeCenter-rightEyeRange/2;
  rightEyeMax = rightEyeCenter+rightEyeRange/2;
  rightEyeStep = int(rightEyeRange);
  nextRightEye = rightEyePos + rightEyeDir*rightEyeStep;
  if (int(rightEyeRange) > rightEyeRangeMax) {
    rightEyeRange = rightEyeRangeMax;
  }

  stepLength = leftEyeRange*assumedServoSpeed;
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
  
//   if(!Serial) {  //check if Serial is available... if not,
//    Serial.end();      // close serial port
//    delay(100);        //wait 100 millis
//    Serial.begin(250000); // reenable serial again
//    Serial.println("Serial lost and reenabled");
//   }

  // check the current time and wait until nextMili to start the next step
  currMilli = millis();
//  Serial.print("millis="); Serial.println(currMilli);

//  calcStepLengthAndRanges();

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
  

  

  if (nextLeftEye > leftEyeMax ) {
    leftEyeDir = -1;
    nextLeftEye = leftEyeMax-leftEyeStep;
    leftEyeRealBPM = (1/double(currMilli-leftEyeLastSwitch))*1000*60/2;
    leftEyeLastSwitch = currMilli;
  } else if (nextLeftEye < leftEyeMin) {
    leftEyeDir = 1;
    nextLeftEye = leftEyeMin+leftEyeStep;
    leftEyeRealBPM = (1/double(currMilli-leftEyeLastSwitch))*1000*60/2;
    leftEyeLastSwitch = currMilli;
  }

  leftEyeServo.write(nextLeftEye);
  leftEyePos = nextLeftEye;
  nextLeftEye = leftEyePos + leftEyeDir*leftEyeStep;


  if (nextRightEye > rightEyeMax ) {
    rightEyeDir = -1;
    nextRightEye = rightEyeMax-rightEyeStep;
    rightEyeRealBPM = (1/double(currMilli-rightEyeLastSwitch))*1000*60/2;
    rightEyeLastSwitch = currMilli;
  } else if (nextRightEye < rightEyeMin) {
    rightEyeDir = 1;
    nextRightEye = rightEyeMin+rightEyeStep;
    rightEyeRealBPM = (1/double(currMilli-rightEyeLastSwitch))*1000*60/2;
    rightEyeLastSwitch = currMilli;
  }

  rightEyeServo.write(nextRightEye);
  rightEyePos = nextRightEye;
  nextRightEye = rightEyePos + rightEyeDir*rightEyeStep;

  
  
  

  
  

  delay(stepLength);
}
