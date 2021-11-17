#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

// servo driver board setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
uint8_t s1Port = 0;

double s1Step = 1;
int baseTime = 15;
int timeMult = 1;

int timestep = baseTime*timeMult;
bool waitToStart = false;

int s1Center = 90;
double s1Range = 40;

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600); // open the serial port at 9600 bps:
  
  // Set up servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  if (waitToStart) {
    while (Serial.available() == 0) {
      // Wait for User to Input Data
    }
    serial_flush();
  }

}

void setServo(uint8_t servoPort, double deg) {
  // calculate pulselen from deg
  uint16_t pulselen = map(deg, 0, 180, SERVOMIN, SERVOMAX);
  Serial.println(pulselen);

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
    s1Min = s1Center-s1Range/2;
    s1Max = s1Center+s1Range/2;
    serial_flush();
  }
}

void checkNewTime() {
  if (Serial.available() > 0) {
    timestep = Serial.parseInt();
    serial_flush();
  }
}

void matchBPM(double bpm, int maxTimeMult, double maxStep, double maxRange){
  int newTime = 15;
  double newStep = 1;
  double newRange = maxRange;
  while (newTime <
}

double calcBPM(int newTime, double newStep, double newRange) {
  
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

  
  Serial.print("currMilli="); Serial.println(currMilli);
  Serial.print("realStepLength="); Serial.println(realStepLength);
//  Serial.print("s1Pos="); Serial.println(s1Pos);
  Serial.print("s1Step="); Serial.println(s1Step);
  Serial.print("s1Range="); Serial.println(s1Range);
//  Serial.print("s1Min="); Serial.println(s1Min);
//  Serial.print("s1Max="); Serial.println(s1Max);
//  Serial.print("s1RealBPM="); Serial.println(s1RealBPM);
  Serial.print("s1RealSpeed="); Serial.println(s1RealSpeed);


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
  }

  // write new servo position
  setServo(s1Port,s1Pos);
  s1Pos = s1Next;
  s1Next = s1Pos + s1Dir*s1Step;

  delay(timestep);

}
