#include <Wire.h>

// import servo driver library
#include <Adafruit_PWMServoDriver.h>

// servo driver board setup
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
#define SERVOMIN  130 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  430 // This is the 'maximum' pulse length count (out of 4096)


// script/testing parameters

double s1Step = 1;
int timestep = 700;

int s1Center = 107;
double s1Range = 3;
uint8_t s1Port = 2;

bool waitToStart = false;


// initialize other global variables

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
  
  Serial.begin(74880); // open the serial port at 9600 bps:
  
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
  uint16_t pulselen = map(deg, 30, 160, SERVOMIN, SERVOMAX);
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

void serial_flush(void) {
  // manually flushes the serial input so that user input values aren't misread
  while (Serial.available()) Serial.read();
}

void loop() {

  // check real loop speed
  currMilli = millis();
  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
  nextMilli = currMilli + timestep;

  // write new servo position
  setServo(s1Port,s1Pos);

  // find next position
  s1Pos = s1Next;
  s1Next = s1Pos + s1Dir*s1Step;
  
  // check than next position is not outside bounds, if so then correct it
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

  // print values for debugging - uncommenting more than 4 lines can cause serial buffer overflow errors
//  Serial.print("currMilli="); Serial.println(currMilli);
//  Serial.print("realStepLength="); Serial.println(realStepLength);
  Serial.print("s1Pos="); Serial.println(s1Pos);
//  Serial.print("s1Step="); Serial.println(s1Step);
//  Serial.print("s1Range="); Serial.println(s1Range);
//  Serial.print("s1Min="); Serial.println(s1Min);
//  Serial.print("s1Max="); Serial.println(s1Max);
//  Serial.print("s1RealBPM="); Serial.println(s1RealBPM);
//  Serial.print("s1RealSpeed="); Serial.println(s1RealSpeed);

  delay(timestep);

}
