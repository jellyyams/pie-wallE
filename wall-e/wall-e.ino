#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

int microphonePin = A0; 
int microphoneValue = 0; 

double allBPM = 70;

double goalBpm[7] =     {allBPM,allBPM,allBPM,allBPM,allBPM,allBPM,allBPM};
double centerPos[7] =   {95, 95}; // (deg)
double maxVel[7] =      {60, 60}; // (deg/s)
double maxRange[7] =    {60, 60}; // (deg)

int timeMult[7]; 
double stepDist[7];
double range[7];
double closestBPM[7];
double minPos[7];
double maxPos[7];
double curPos[7];
double nextPos[7];
unsigned long lastSwitch[7];
double realBPM[7];
double realVel[7];

double maxError = 1.0;
double minStep = 0.1;


// Servo driver setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

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
int leftEyeMax = leftEyeCenter+leftEyeRange;
int leftEyePos = leftEyeCenter;
int leftEyeStep = leftEyeRange;
int leftEyeDir = 1;
int nextLeftEye = leftEyePos + leftEyeDir*leftEyeStep;
unsigned long leftEyeLastSwitch = 0;
double leftEyeRealBPM = 0;

double rightEyeRange = 50;
int rightEyeMin = rightEyeCenter-rightEyeRange;
int rightEyeMax = rightEyeCenter+rightEyeRange;
int rightEyePos = rightEyeCenter;
int rightEyeStep = rightEyeRange;
int rightEyeDir = -1;
int nextRightEye = rightEyePos + rightEyeDir*rightEyeStep;
unsigned long rightEyeLastSwitch = 0;
double rightEyeRealBPM = 0;




void setup() {
  // put your setup code here, to run once:

  // Set up servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

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
  calcFullSpeedVals();

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

void calcFullSpeedVals() {

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

void matchBPM(uint8_t s, double BPM) {
  int newTimeMult;
  double newStep;
  double newRange;
  double newBPM;
  double newError;
  double closestBPM;
  double closestError=100;
  
  newTimeMult = 1;
  while (newTimeMult <= maxTimeMult[s]) {
    Serial.println(newTimeMult);
    if (abs(closestError) < maxError) {
      return;
    }
    newStep = stepFromTimeMult(newTimeMult,maxVel[s]);
    Serial.print("newStep="); Serial.println(newStep);
    newRange = calcRange(BPM, newTimeMult, newStep);
    Serial.print("newRange="); Serial.println(newRange);
    if (newRange > maxRange[s]) {
      newRange = maxRange[s];
      Serial.print("Too large. newRange="); Serial.println(newRange);
      newStep = calcStep(BPM, newTimeMult, newRange);
      Serial.print("newStep="); Serial.println(newStep);
    }
    newBPM = calcBPM(newTimeMult, newStep, newRange);
    Serial.print("newBPM="); Serial.println(newBPM);
    newError = newBPM-BPM;
    Serial.print("newError="); Serial.println(newError);
    if (newError > -0.01 && abs(newError) < abs(closestError) && newStep >= minStep) {
      closestError = BPM-newBPM;
      
      timeMult[s] = newTimeMult;
      stepDist[s] = newStep;
      range[s] = newRange;
      minPos[s] = center[s]-range[s]/2;
      maxPos[s] = center[s]+range[s]/2;
      closestBPM[s] = newBPM;
    }
    newTimeMult++;
  }
}
double stepFromTimeMult(double newTimeMult, double vel) {
  return vel*double(newTimeMult*baseTime)/1000.0;
}
double calcStep(double bpm,int newTimeMult, double newRange) {
  return newRange*bpm*double(newTimeMult*baseTime)/(60.0*1000.0);
}
double calcRange(double bpm,int newTimeMult, double newStep) {
  return 60.0*1000.0*newStep/(bpm*double(newTimeMult*baseTime));
}
double calcBPM(int newTimeMult, double newStep, double newRange) {
  return newStep*1000.0*60.0/(newRange*double(newTimeMult*baseTime));
}
double calcTimeMult(double bpm, double newStep, double newRange) {
  return 60.0*1000.0/(newRange*bpm*double(baseTime));
}

void adjustBPM(uint8_t s, double reduction){
  if (realBPM[s] < bpm[s]) {
    matchBPM(s, bpm[s]+(bpm[s]-realBPM[s]));
  }
}
void calcNextPos(uint8_t s) {
  // calc next position
  nextPos[s] = curPos[s] + dir[s]*stepDist[s];
  // switch directions and record real measurements at each end of range
  if (nextPos[s] > maxPos[s] ) {
    dir[s] = -1;
    nextPos[s] = maxPos[s]- stepDist[s];
    realBPM[s] = (1/double(currMilli-lastSwitch[s]))*1000*60; // calculate the actual time it took to complete the whole range
    realVel[s] = (1/double(currMilli-lastSwitch[s]))*1000*range[s]; // calculate the actual angular speed (deg/s)
    lastSwitch[s] = currMilli;
  } else if (nextPos[s] < minPos[s]) {
    dir[s] = 1;
    nextPos[s] = maxPos[s]- stepDist[s];
    realBPM[s] = (1/double(currMilli-lastSwitch[s]))*1000*60; // calculate the actual time it took to complete the whole range
    realVel[s] = (1/double(currMilli-lastSwitch[s]))*1000*range[s]; // calculate the actual angular speed (deg/s)
    lastSwitch[s] = currMilli;
    adjustBPM(s,1);
  }
}

void printVals(uint8_t s) {
  Serial.print("s"); Serial.print(s); Serial.print(" stepDist="); Serial.println(stepDist[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" curPos="); Serial.println(curPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" range="); Serial.println(range[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" minPos="); Serial.println(minPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" maxPos="); Serial.println(maxPos[s]);
  Serial.print("s"); Serial.print(s); Serial.print(" realBPM="); Serial.println(realBPM[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" realVel="); Serial.println(realVel[s]);
}

void setServo(uint8_t s) {
  // calculate pulselen from deg (of nextPos)
  pulselen = map(nextPos[s], 0, 180, SERVOMIN, SERVOMAX);
  // set PWM to pulselen
  pwm.setPWM(s, 0, pulselen);
  // update curPos
  curPos[s] = nextPos[0];
}


void nodHead() {
  // sends servo commands for one step of the nodHead dance move
}

void shakeHead() {
  // sends servo commands for one step of the shakeHead dance move
}

void bobHead() {
  // sends servo commands for one step of the bobHead dance move
}

void swingArms() {
  // sends servo commands for one step of the swingArms dance move
}

void tiltHead() {
  // sends servo commands for one step of the tiltHead dance move
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
  
  // update bpm from beat sensing
//  microphoneValue = analogRead(microphonePin); 
//  updateBPM(microphoneValue); 

  // check the current time
  currMilli = millis();

  // calculate how far off we are from the nextMilli we were aiming for
  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
  nextMilli = currMilli + stepLength;

  // Print current time and movement parameters for debugging
//  Serial.print("currMilli="); Serial.println(currMilli);
//  Serial.print("realStepLength="); Serial.println(realStepLength);

  for (uint8_t i; i <=1; i++) {
    calcNextPos(0);
    printVals(0);
    setServo(0); // set servo to nextPos, update curPos to nextPos
  }
  
 adjustedDelay = int(nextMilli-millis());
 delay(adjustedDelay);
//  delay(timestep);

}
