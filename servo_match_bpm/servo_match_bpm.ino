#include <Wire.h>
#include <Servo.h>

#include <Adafruit_PWMServoDriver.h>

// servo driver board setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
uint8_t s1Port = 0;

double BPM = 90;
int baseTime = 15;
int timeMult = 1;

int timestep = baseTime*timeMult;
int adjustedDelay = timestep;
bool waitToStart = false;

int s1Center = 90;
double s1MaxRange = 58;
double s1MaxVel = 60;
int s1MaxTimeMult = 4;
double s1MaxError = 2;

double s1Step = 1;

double s1Range = s1MaxRange;
double s1Min = s1Center-s1Range/2;
double s1Max = s1Center+s1Range/2;
double s1Pos = s1Min;
int s1Dir = 1;
double s1Next = s1Pos + s1Dir*s1Step;

unsigned long s1LastSwitch = 0;
double s1RealBPM = 0;
double s1RealSpeed = 0; // deg/s


unsigned long currMilli = 0;
unsigned long oldMilli = 0;
unsigned long nextMilli = timestep;
unsigned long realStepLength;
unsigned long nextBeat = int(1/BPM *60.0*1000.0);


int closestTimeMult;
double closestStep;
double closestRange;
double closestBPM;
double closestError = 100;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600); // open the serial port at 9600 bps:
  
  // Set up servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  matchBPM(BPM, s1MaxTimeMult, s1MaxRange, s1MaxVel, s1MaxError);
  timeMult = closestTimeMult;
  s1Step = closestStep;
  s1Range = closestRange;
  s1Min = s1Center-s1Range/2;
  s1Max = s1Center+s1Range/2;

  if (waitToStart) {
    while (Serial.available() == 0) {
      // Wait for User to Input Data
    }
    serial_flush();
    nextBeat = millis() + int(1/BPM *60.0*1000.0);
  }

}

void setServo(uint8_t servoPort, double deg) {
  // calculate pulselen from deg
  uint16_t pulselen = map(deg, 0, 180, SERVOMIN, SERVOMAX);

  // set PWM to pulselen
  pwm.setPWM(servoPort, 0, pulselen);
}

void checkNewStep() {
  if (Serial.available() > 0) {
    s1Step = Serial.parseFloat();
    serial_flush();
  }
}

void checkNewRange() {
  if (Serial.available() > 0) {
    s1Range = Serial.parseFloat();
    serial_flush();
    s1Min = s1Center-s1Range/2;
    s1Max = s1Center+s1Range/2;
  }
}

void checkNewTime() {
  if (Serial.available() > 0) {
    timestep = Serial.parseInt();
    serial_flush();
  }
}

void checkNewBPM() {
  if (Serial.available() > 0) {
    BPM = Serial.parseFloat();
    serial_flush();
    
    matchBPM(BPM, s1MaxTimeMult, s1MaxRange, s1MaxVel, s1MaxError);
    timeMult = closestTimeMult;
    s1Step = closestStep;
    s1Range = closestRange;
    s1Min = s1Center-s1Range/2;
    s1Max = s1Center+s1Range/2;
  }
}

void matchBPM(double bpm, double maxTimeMult, double maxRange, double maxVel, double maxError){
  // using maxVel, calc step at timeMult=1
  // using step, timeMult, and bpm, calc range
  // if range>maxRange, calc step using maxRange, timeMult, and bpm
  int newTimeMult;
  double newStep;
  double newRange;
  double newBPM;
  double newError;
  newTimeMult = 1;
  while (newTimeMult <= maxTimeMult) {
    Serial.println(newTimeMult);
    if (abs(closestError) < maxError) {
      return;
    }
    newStep = stepFromTimeMult(newTimeMult,maxVel);
    Serial.print("newStep="); Serial.println(newStep);
    newRange = calcRange(bpm, newTimeMult, newStep);
    Serial.print("newRange="); Serial.println(newRange);
    if (newRange > maxRange) {
      newRange = maxRange;
      Serial.print("Too large. newRange="); Serial.println(newRange);
      newStep = calcStep(bpm, newTimeMult, newRange);
      Serial.print("newStep="); Serial.println(newStep);
    }
    newBPM = calcBPM(newTimeMult, newStep, newRange);
    Serial.print("newBPM="); Serial.println(newBPM);
    newError = newBPM-bpm;
    Serial.print("newError="); Serial.println(newError);
    if (newError > -0.01 && abs(newError) < abs(closestError)) {
      closestTimeMult = newTimeMult;
      closestStep = newStep;
      closestRange = newRange;
      closestBPM = newBPM;
      closestError = bpm-closestBPM;
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

void adjustBPM(double reduction){
  if (s1RealBPM < BPM) {
    matchBPM(BPM+(BPM-s1RealBPM)/reduction, s1MaxTimeMult, s1MaxRange, s1MaxVel, s1MaxError);
    timeMult = closestTimeMult;
    s1Step = closestStep;
    s1Range = closestRange;
  }
  
}
  
void serial_flush(void) {
  // manually flushes the serial input so that user input values aren't misread
  while (Serial.available()) Serial.read();
}

void loop() {
  // put your main code here, to run repeatedly:


  currMilli = millis();

  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
  nextMilli = currMilli + timestep;

  

//  checkNewBPM();
  
  
  
//  Serial.print("currMilli="); Serial.println(currMilli);
  Serial.print("realStepLength="); Serial.println(realStepLength);
//  Serial.print("s1Pos="); Serial.println(s1Pos);
//  Serial.print("s1Step="); Serial.println(s1Step);
//  Serial.print("s1Range="); Serial.println(s1Range);
//  Serial.print("s1Min="); Serial.println(s1Min);
//  Serial.print("s1Max="); Serial.println(s1Max);
  Serial.print("s1RealBPM="); Serial.println(s1RealBPM);
//  Serial.print("s1RealSpeed="); Serial.println(s1RealSpeed);
//  Serial.print("closestBPM="); Serial.println(closestBPM);


  if (s1Next > s1Max ) {
    s1Dir = -1;
    s1Next = s1Max-s1Step;
    s1RealBPM = (1/double(currMilli-s1LastSwitch))*1000*60; // calculate the actual time it took to complete the whole range
    s1RealSpeed = (1/double(currMilli-s1LastSwitch))*1000*s1Range; // calculate the actual angular speed (deg/s)
    s1LastSwitch = currMilli;
  } else if (s1Next < s1Min) {
    s1Dir = 1;
    s1Next = s1Min+s1Step;
    s1RealBPM = (1/double(currMilli-s1LastSwitch))*1000*60; // calculate the actual time it took to complete the whole range
    s1RealSpeed = (1/double(currMilli-s1LastSwitch))*1000*s1Range; // calculate the actual angular speed (deg/s)
    s1LastSwitch = currMilli;
    adjustBPM(1);
  }

  // write new servo position
  setServo(s1Port,s1Pos);
  s1Pos = s1Next;
  s1Next = s1Pos + s1Dir*s1Step;

  
  adjustedDelay = int(nextMilli-millis());
  delay(adjustedDelay);
//  delay(timestep);

}
