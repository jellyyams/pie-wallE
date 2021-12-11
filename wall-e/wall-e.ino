#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo driver setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN  130 // This is the 'minimum' pulse length count (out of 4096) -> results in angle of ~30 degs, the same as if commanded by servoObject.write(30)
#define SERVOMAX  430 // This is the 'maximum' pulse length count (out of 4096) -> results in angle of ~160 degs, the same as if commanded by servoObject.write(160)


// Trial Parameters
double allBPM = 60; // bpm to set all of the servos to (so that it can be adjusted between servos so that some parts can move at half speed of others)
unsigned long baseTime = 30; // the ideal length of each loop (ms)
bool waitToStart = false; // whether to wait for something to send in the Serial input before starting (in order to manually start on the beat)
bool dispCalc = false; // whether to display movement parameter calculations in the matchBPM() function
bool useBPM = true;
bool adjBPM = false; // whether to run matchBPM with a corrected value (aka the adjustBPM() function) every time realBPM is calculated
int servosRunning = 7; // only try to run and find movement parameters for servos that have goalBPM, centerPos, maxRange, and maxVel defined


// Other Tuning and Setup Parameters

// servo driver pin numbers
uint8_t eyeRPin =  0;
uint8_t eyeLPin =  1;
uint8_t headPin =  2;
uint8_t neckTPin = 3;
uint8_t neckBPin = 4;
uint8_t armLPin =  5;
uint8_t armRPin =  6;
// to reduce the number of variable names, individual servo parameters are held in arrays, 
// indexed by the pin they are pluged into (listed above)
//                       0          1           2         3         4         5         6
double goalBPM[7] =     {allBPM/2,  allBPM/2,   allBPM,   allBPM,   allBPM,   allBPM,   allBPM};
double centerPos[7] =   {95,        95,         107,      95,       105,       95,       95}; // (deg)
double maxRange[7] =    {30,        30,         106,      130,      110,       30,       30}; // maximum total angular travel defined by mechanical limits (deg)
double maxVel[7] =      {20,        20,         60,       60,       60,       60,       60}; // (deg/s)
double maxError = 1.0; // max difference between newBPM or closestBPM and goalBPM
double minStep = 0.1; // minimum step distance (deg)
double maxTimeMult = 5; // maximum baseTime multiplier to check

//double centerPos[7] =   {95, 95, 60, 60, 60}; // (deg)
//double maxRange[7] =    {30, 30, 30, 30, 30}; // maximum total angular travel defined by mechanical limits (deg)


// Initalization of other global variables

int timeMult[7] = {1,1,1,1,1,1,1}; // initalize with default values
double stepDist[7] = {0.5,0.5,0.5,0.5,0.5,0.5,0.5}; // initalize with default values (deg)
double range[7]; // calculated ideal travel of servo (deg)
double closestBPM[7]; // closest bpm to goal bpm that can be calculated with contraints
double minPos[7]; // bottom of travel defined by range and centerPos - NOT necessarily the mechanical limits of the servo (deg)
double maxPos[7]; // top of travel defined by range and centerPos    -                      "                             (deg)
double curPos[7]; // (deg)
double nextPos[7]; // (deg)
int dir[7] = {-1,1,1,1,-1,1,1}; // direction servo is currently moving, + for towards maxPos (CC), - for towards minPos (CW)
unsigned long lastSwitch[7]; // timestamp of last change of direction (ms)
double realBPM[7] = {allBPM,allBPM,allBPM,allBPM,allBPM,allBPM,allBPM}; // calculated from time of last direction change
double realVel[7]; // calculated from time of last direction change and range travelled (deg/s)
int currMult[7] = {1,1,1,1,1,1,1}; // counter to only update servo position after timeMult baseTime intervals

unsigned long oldMilli = 0; // (ms timestamp)
unsigned long currMilli = 0; // (ms timestamp)
unsigned long nextMilli = baseTime; // (ms timestamp)       
unsigned long realStepLength = baseTime; // real time between the oldMilli and currMilli to check if adjustedDelay is working (ms)
unsigned long adjustedDelay = baseTime - currMilli; // time to wait at the end of each loop in order to make timesteps the right length (ms)

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
    // default values to allow non bpm-matching trials
    range[i] = maxRange[i];
    minPos[i] = centerPos[i]-range[i]/2;
    maxPos[i] = centerPos[i]+range[i]/2;
    curPos[i] = minPos[i];
    nextPos[i] = minPos[i];
    
    if (useBPM) {matchBPM(i,goalBPM[i]);} // comment out to allow non bpm-matching trials
    
    setServo(i); // set servo to one end
  }

  setEyeModeLift();
  
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

double stepFromTimeMult(double newTimeMult, double vel) {
  // calculates step distance using time between steps (defined by newTimeMult*baseTime) and an angular speed of the servo (vel, in deg/s)
/*
vel (deg)     stepDist (deg)            1000 (ms)
--------- =   ----------------------  x ---------
(s)           timeMult*baseTime (ms)    (s)

--> Solve for stepDist
                  vel (deg)                                (s)
stepDist (deg) =  ---------   x timeMult*baseTime (ms)  x  -------
                  (s)                                      1000 (ms)
*/
  return vel*double(newTimeMult*baseTime)/1000.0;
}
double calcRange(double bpm,int newTimeMult, double newStep) {
  // calculates range of movement based on a bpm, the time between steps (defined by newTimeMult*baseTime), and the step distance (deg)
/*
range (deg)   (min)           60 (s)   1000 (ms)      stepDist (deg)
----------- = --------   x    ----  x  -------    x   ----------------------
(beat)        bpm (beat)      (min)    (s)            timeMult*baseTime (ms)
*/
  return 60.0*1000.0*newStep/(bpm*double(newTimeMult*baseTime));
}
double calcStep(double bpm,int newTimeMult, double newRange) {
  // calculates step distance based on a bpm, the time between steps (defined by newTimeMult*baseTime), and the range of movement (deg)
/*
                  range (deg)     bpm (beat)    (min)       (s)
stepDist (deg) =  -----------  x  ----------  x ------    x ---------  x  timeMult*baseTime (ms)
                  (beat)          (min)         60 (s)      1000 (ms)
*/
  return newRange*bpm*double(newTimeMult*baseTime)/(60.0*1000.0);
}
double calcBPM(int newTimeMult, double newStep, double newRange) {
  // calculates bpm based on the time between steps (defined by newTimeMult*baseTime), the step distance (deg), and the range of movement (deg)
/*
bpm (beat)    (beat)          stepDist (deg)                1000 (ms)     60 (s)
---------- =  -----------  x  ----------------------     x  ---------   x ------
(min)         range (deg)     timeMult*baseTime (ms)        (s)           (min)
*/
  return newStep*1000.0*60.0/(newRange*double(newTimeMult*baseTime));
}

void matchBPM(uint8_t s, double BPM) {
  // changes a servo's timeMult, stepDist, and range to hit a given BPM

/*  1. Choose a baseTime multiplier (start with one, for max smoothness)
 *  2. Use that and the max servos speed (maxVel) to determine the range the servo 
 *     would need to move through to match the bpm without pausing to the ends
 *  3. If that range is outside of the servo's maxRange, use the max range and slow 
 *     the servo down so that it just reaches the ends of the range at each beat. To 
 *     do this, we calculate a new step distance with the defined variables.
 *  4. Check how close we got to the goal BPM. If we're within the margin of error, 
 *     and the new step distance isn't too small, record these values. If either of
 *     these are false, start over with timeMult++ (this can fix the minimum step
 *     distance problem - which is usually the issue, as the goalBPM<->newBPM error is 
 *     usually minimal because it's calculated from the same equations it's being tested
 *     on.)
 */
  
  int newTimeMult;
  double newStep;
  double newRange;
  double newBPM;
  double newError;
  double closestError=100;
  
  newTimeMult = 1; // the multiple of the baseTime to update servo position on
  while (newTimeMult <= maxTimeMult) { // prepare to test all timeMults 1->max
    if (dispCalc) {Serial.println(newTimeMult);}
    if (abs(closestError) < maxError) { // if result from last loop was close enough, leave function
      return;
    }
    newStep = stepFromTimeMult(newTimeMult,maxVel[s]); // calculate step distance from timeMult and maxVel
    if (dispCalc) {Serial.print("newStep="); Serial.println(newStep);}
    newRange = calcRange(BPM, newTimeMult, newStep);  // calculate range to achive bpm with defined baseTime multiple and step distance
    if (dispCalc) {Serial.print("newRange="); Serial.println(newRange);}
    if (newRange > maxRange[s]) { // if the ideal range is outside of maxRange, then use maxRange and move servo slower (smaller step size)
      newRange = maxRange[s];
      if (dispCalc) {Serial.print("Too large. newRange="); Serial.println(newRange);}
      newStep = calcStep(BPM, newTimeMult, newRange);
      if (dispCalc) {Serial.print("newStep="); Serial.println(newStep);}
    }
    newBPM = calcBPM(newTimeMult, newStep, newRange); // calculate the bpm with parameters determined above to check our work
    if (dispCalc) {Serial.print("newBPM="); Serial.println(newBPM);}
    newError = newBPM-BPM;
    if (dispCalc) {Serial.print("newError="); Serial.println(newError);}
    if (newError > -0.01 && abs(newError) < abs(closestError) && newStep >= minStep) { 
      // if new parameters give a closer BPM and the step distance isn't too small, record it. Otherwise, try the next largest timeMult
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

void adjustBPM(uint8_t s, double reduction){
  // runs the matchBPM function again with a slightly higher bpm if realBPM is too slow
  if (realBPM[s] < goalBPM[s]) {
    matchBPM(s, goalBPM[s]+(goalBPM[s]-realBPM[s])/reduction);
  }
}

void updatePos(uint8_t s, bool adj, bool disp) {
  // run this each loop for each servo to move them at the right speed and range
  if (currMult[s] == timeMult[s]) { // check if it's been the right number of baseTime steps to move the servo again
    // if yes, reset the counter and get next position
    currMult[s] = 1;
    calcNextPos(s,adj);
    if (disp) { printVals(1); } // print values for this servo if user wants them displayed (for debugging)
    setServo(s); // maps nextPos (degs) to pwm pulse length and sends to servo driver, then updates currPos to nextPos
  } else {
    // if no, just add to counter and don't move servo
    currMult[s] = currMult[s] + 1;
  }
}

void calcNextPos(uint8_t s, bool adj) {  
  // calc next position for servo at port s
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
    if (adj && useBPM) {adjustBPM(s,2);}
  }
}

void printVals(uint8_t s) {
  // WARNING: Uncomment only currently relevant lines. Printing too many statements per loop can overflow Arduino/COM buffer and cause failure
//  Serial.print("s"); Serial.print(s); Serial.print(" stepDist="); Serial.println(stepDist[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" curPos="); Serial.println(curPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" dir="); Serial.println(dir[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" range="); Serial.println(range[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" minPos="); Serial.println(minPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" maxPos="); Serial.println(maxPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" realBPM="); Serial.println(realBPM[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" realVel="); Serial.println(realVel[s]);
  Serial.print("s"); Serial.print(s); Serial.print(" timeMult="); Serial.println(timeMult[s]);
}

void setServo(uint8_t s) {
  // Set servo s to nextPos, update curPos to nextPos
  // calculate pulselen from deg (of nextPos)
  int pulselen = map(nextPos[s], 30, 160, SERVOMIN, SERVOMAX);
  // set PWM to pulselen
  pwm.setPWM(s, 0, pulselen);
  // update curPos
  curPos[s] = nextPos[s];
}

void runAllServos() {
  for (uint8_t i = 0; i < servosRunning; i++) { // run the first 5 (or other val for servosRunning) servos and skip the rest
    updatePos(i, adjBPM, false); // which servo to update, whether to adjust bpm, whether to print values
  }
}

void nodHead() {
  // sends servo commands for one step of the nodHead dance move
  updatePos(neckTPin, adjBPM, false);
}

void shakeHead() {
  // sends servo commands for one step of the shakeHead dance move
  updatePos(headPin, adjBPM, false);
}

void bobHead() {
  // sends servo commands for one step of the bobHead dance move
  updatePos(neckTPin, adjBPM, false);
  updatePos(neckBPin, adjBPM, false);
}

void swingArms() {
  // sends servo commands for one step of the swingArms dance move
  updatePos(armRPin, adjBPM, false);
  updatePos(armLPin, adjBPM, false);
}

void setEyeModeLift() {
  // set right eye to opposite side of range from left eye so that eyes raise and lower together
  dir[eyeRPin] = -1;
  curPos[eyeRPin] = maxPos[eyeRPin];
  nextPos[eyeRPin] = maxPos[eyeRPin];
  dir[eyeLPin] = 1;
  curPos[eyeLPin] = minPos[eyeLPin];
  nextPos[eyeLPin] = minPos[eyeLPin];
}

void setEyeModeTilt() {
  dir[eyeRPin] = 1;
  curPos[eyeRPin] = minPos[eyeRPin];
  nextPos[eyeRPin] = minPos[eyeRPin];
  dir[eyeLPin] = 1;
  curPos[eyeLPin] = minPos[eyeLPin];
  nextPos[eyeLPin] = minPos[eyeLPin];
}

void moveEyes() {
  // sends servo commands for one step of the tiltHead dance move
  updatePos(eyeRPin, adjBPM, false);
  updatePos(eyeLPin, adjBPM, false);
}


void loop() {


  // check the current time
  currMilli = millis();
  // calculate how long the last loop actually took and record milli for next loop
  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
  // Print current time and real step length for debugging
  // WARNING: printing too many statements per loop can overflow Arduino/COM buffer and cause failure
//  Serial.print("currMilli="); Serial.println(currMilli);
//  Serial.print("realStepLength="); Serial.println(realStepLength);

//  runAllServos();
//  moveEyes();
//  shakeHead();
//  nodHead();
  bobHead();
//  swingArms();



//  setServoPos(0,95);
//  setServoPos(1,50);
  
//  printVals(0); // print values for only one of the servos
  printVals(5); // print values for only one of the servos
  printVals(6); // print values for only one of the servos


  // dynamically adjust length of delay based on how much time has already been spent in this loop and when the next one should start
  nextMilli = currMilli + baseTime; // determine next step based on "current time" recorded at beginning of loop
  adjustedDelay = int(nextMilli-millis()); // calculate how many more ms to wait before starting next loop
  delay(adjustedDelay);

}
