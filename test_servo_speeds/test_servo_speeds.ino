#include <Servo.h>

Servo s1Servo;

int s1Step = 1;
int timestep = 28;
bool waitToStart = false;


int s1Center = 100;
double s1Range = 60;

int s1Min = s1Center-s1Range/2;
int s1Max = s1Center+s1Range/2;
int s1Pos = s1Min;
int s1Dir = 1;
int s1Next = s1Pos + s1Dir*s1Step;

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
  s1Servo.attach(3);  // attaches the servo on pin __ to the s1Servo object

  s1Servo.write(s1Pos);

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

void loop() {
  // put your main code here, to run repeatedly:

//  if (Serial.available() > 0) {
//    s1Range = Serial.parseInt();
//    s1Min = s1Center-s1Range/2;
//    s1Max = s1Center+s1Range/2;
//    serial_flush();
//  }
  if (Serial.available() > 0) {
    s1Step = Serial.parseInt();
    serial_flush();
  }
//  if (Serial.available() > 0) {
//    timestep = Serial.parseInt();
//    serial_flush();
//  }

  currMilli = millis();

  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
  nextMilli = currMilli + timestep;

  
  Serial.print("currMilli="); Serial.println(currMilli);
  Serial.print("realStepLength="); Serial.println(realStepLength);
//  Serial.print("s1Pos="); Serial.println(s1Pos);
//  Serial.print("s1Step="); Serial.println(s1Step);
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
  s1Servo.write(s1Next);
  s1Pos = s1Next;
  s1Next = s1Pos + s1Dir*s1Step;

  delay(timestep);

}
