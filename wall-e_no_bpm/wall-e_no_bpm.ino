#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

// Servo driver setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN  130 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  430 // This is the 'maximum' pulse length count (out of 4096)



// Trial Parameters
double allBPM = 70;
unsigned long baseTime = 20;
int servosRunning = 5;
bool waitToStart = false; // wait for something to send in the Serial input before starting (in order to manually start on the beat)


// Other Tuning and Setup Parameters
double goalBPM[7] =     {allBPM,allBPM,allBPM,allBPM,allBPM,allBPM,allBPM};
double centerPos[7] =   {60, 140, 107, 95, 95}; // (deg)
double maxRange[7] =    {30, 30, 106, 30, 30}; // total angular travel (deg)
double maxVel[7] =      {60, 60, 60, 60, 60}; // (deg/s)
double maxError = 1.0;
double minStep = 0.1;
double maxTimeMult = 5;
//int microphonePin = A0; 
//int microphoneValue = 0; 


// Initalization of other global variables

int timeMult[7] = {1,1,1,1,1}; 
double stepDist[7] = {0.5,0.5,0.5,0.5,0.5};
double range[7];
double closestBPM[7];
double minPos[7];
double maxPos[7];
double curPos[7];
double nextPos[7];
int dir[7] = {1,1,1,1,1,1,1};
unsigned long lastSwitch[7];
double realBPM[7] = {allBPM,allBPM,allBPM,allBPM,allBPM,allBPM,allBPM};
double realVel[7];
int currMult[7] = {1,1,1,1,1};

unsigned long oldMilli = 0;
unsigned long currMilli = 0;
unsigned long nextMilli = baseTime;        
unsigned long realStepLength = baseTime;
unsigned long adjustedDelay = baseTime;

void setup() {
  // put your setup code here, to run once:

  // Set up servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  // Set up serial
  Serial.begin(74880); // open the serial port at 9600 bps:

  // Move servos to starting positions and calculate the starting ranges and step dist based on BPM
  for (uint8_t i = 0; i < servosRunning; i++) {
//    matchBPM(i,goalBPM[i]);
    range[i] = maxRange[i];
    minPos[i] = centerPos[i]-range[i]/2;
    maxPos[i] = centerPos[i]+range[i]/2;
    curPos[i] = minPos[i];
    nextPos[i] = minPos[i];
    setServo(i);
  }
  
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

void matchBPM(uint8_t s, double BPM) {
  int newTimeMult;
  double newStep;
  double newRange;
  double newBPM;
  double newError;
  double closestError=100;
  
  newTimeMult = 1;
  while (newTimeMult <= maxTimeMult) {
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
      minPos[s] = centerPos[s]-range[s]/2;
      maxPos[s] = centerPos[s]+range[s]/2;
      closestBPM[s] = double(newBPM);
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
  if (realBPM[s] < goalBPM[s]) {
    matchBPM(s, goalBPM[s]+(goalBPM[s]-realBPM[s])/reduction);
  }
}
void updatePos(uint8_t s, bool adj, bool disp) {
  if (currMult[s] == timeMult[s]) {
    currMult[s] = 1;
    calcNextPos(s,adj);
    if (disp) { printVals(s); }
    setServo(s);
  } else {
    currMult[s] = currMult[s] + 1;
  }
}
void calcNextPos(uint8_t s, bool adj) {  
  // calc next position
  nextPos[s] = curPos[s] + dir[s]*stepDist[s];
  // switch directions and record real measurements at each end of range
  if (nextPos[s] > maxPos[s]) {
    dir[s] = -1;
    nextPos[s] = maxPos[s]- stepDist[s];
    realBPM[s] = (1/double(currMilli-lastSwitch[s]))*1000*60; // calculate the actual time it took to complete the whole range
    realVel[s] = (1/double(currMilli-lastSwitch[s]))*1000*range[s]; // calculate the actual angular speed (deg/s)
    lastSwitch[s] = currMilli;
  } else if (nextPos[s] < minPos[s]) {
    dir[s] = 1;
    nextPos[s] = minPos[s]+ stepDist[s];
    realBPM[s] = (1/double(currMilli-lastSwitch[s]))*1000*60; // calculate the actual time it took to complete the whole range
    realVel[s] = (1/double(currMilli-lastSwitch[s]))*1000*range[s]; // calculate the actual angular speed (deg/s)
    lastSwitch[s] = currMilli;
    if (adj) {adjustBPM(s,2);}
  }
}

void printVals(uint8_t s) {
//  Serial.print("s"); Serial.print(s); Serial.print(" stepDist="); Serial.println(stepDist[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" currMult="); Serial.println(currMult[s]);
  Serial.print("s"); Serial.print(s); Serial.print(" curPos="); Serial.println(curPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" range="); Serial.println(range[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" minPos="); Serial.println(minPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" maxPos="); Serial.println(maxPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" realBPM="); Serial.println(realBPM[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" realVel="); Serial.println(realVel[s]);
}

void setServo(uint8_t s) {
  // Set servo s to nextPos, update curPos to nextPos
  // calculate pulselen from deg (of nextPos)
  int pulselen = map(nextPos[s], 30, 160, SERVOMIN, SERVOMAX);
  Serial.println(pulselen);
  // set PWM to pulselen
  pwm.setPWM(s, 0, pulselen);
  // update curPos
  curPos[s] = nextPos[s];
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

//void updateBPM(int val){
//  Serial.println(val); 
//  if (val > 350){
//    currBPM = 75; 
//  } else if (val > 450) {
//    currBPM = 85; 
//  } else {
//    currBPM = 70; 
//  }
//}

void loop() {
  
  // update bpm from beat sensing
//  microphoneValue = analogRead(microphonePin); 
//  updateBPM(microphoneValue); 

  // check the current time
  currMilli = millis();

  // calculate how far off we are from the nextMilli we were aiming for
  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
  nextMilli = currMilli + baseTime;

  // Print current time and movement parameters for debugging
//  Serial.print("currMilli="); Serial.println(currMilli);
//  Serial.print("realStepLength="); Serial.println(realStepLength);

//  for (uint8_t i = 0; i < servosRunning; i++) {
////    Serial.println(i);
//    updatePos(i, false, false); // which servo to update, whether to adjust bpm, whether to print values
//  }

  updatePos(2, false, false);

  printVals(2);
  
 adjustedDelay = int(nextMilli-millis());
 delay(adjustedDelay);
//  delay(timestep);

}
