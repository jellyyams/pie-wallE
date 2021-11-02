
#include <Servo.h>

// servo setup
Servo leftEyeServo;     // create servo object to control wall-e's left eye and head tilt (in conjunction with rightEyeServo)
Servo rightEyeServo;    // create servo object to control wall-e's right eye and head tilt (in conjunction with leftEyeServo)
Servo headPanServo;     // create servo object to control wall-e's head shaking
Servo topNeckServo;     // create servo object to control wall-e's head nodding
Servo botNeckServo;     // create servo object to control wall-e's head bobbing (in conjunction with topNeckServo)
Servo leftArmServo;     // create servo object to control wall-e's left arm rotation
Servo rightArmServo;    // create servo object to control wall-e's right arm rotation

int leftEyePos;
int rightEyePos;
int headPanPos;
int botNeckPos;
int leftArmPos;
int rightArmPos;

int currBPM = 60;    // initialize music BPM. Update with change in music
unsigned long stepLength = 600;      // in milliseconds, calculated from bpm
unsigned long beatStart = 0;
unsigned long oldMilli = 0;
unsigned long currMilli = 0;
unsigned long nextMilli = stepLength;        
unsigned long allowedError = 50; // milliseconds we can be off by
int minDelay = 110;
unsigned long realStepLength = stepLength;

int topNeckMin = 10;
int topNeckMax = 110;
int topNeckPos = topNeckMin;
int topNeckStep = 99;
int nextTopNeck = topNeckPos + topNeckStep;
unsigned long topNeckLastSwitch = 0;
double topNeckRealBPM = 0;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(230400); // open the serial port at 9600 bps:
//  leftEyeServo.attach(9);  // attaches the servo on pin __ to the leftEyeServo object
//  rightEyeServo.attach(10);  // attaches the servo on pin __ to the rightEyeServo object
//  headPanServo.attach(11);  // attaches the servo on pin __ to the headPanServo object
  topNeckServo.attach(9);  // attaches the servo on pin __ to the topNeckServo object
//  botNeckServo.attach(13);  // attaches the servo on pin __ to the botNeckServo object

}
void serial_flush(void) {
  // manually flushes the serial input so that user input values aren't misread
  while (Serial.available()) Serial.read();
}

void calcStepLength() {
  
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

  Serial.print("currMilli="); Serial.println(currMilli);
//  Serial.print("stepLength="); Serial.println(stepLength);
  Serial.print("realStepLength="); Serial.println(realStepLength);
//  Serial.print("topNeckPos="); Serial.println(topNeckPos);
  Serial.print("topNeckRealBPM="); Serial.println(topNeckRealBPM);

  
  // run or continue these dance moves
  topNeckServo.write(nextTopNeck);
  topNeckPos = nextTopNeck;
  nextTopNeck = topNeckPos + topNeckStep;

  if (nextTopNeck > topNeckMax || nextTopNeck < topNeckMin) {
    topNeckStep = -1*topNeckStep;
    nextTopNeck = nextTopNeck + 2*topNeckStep;
    topNeckRealBPM = (1/double(currMilli-topNeckLastSwitch))*1000*60/2;
    topNeckLastSwitch = currMilli;
  }

  
  
  

  
  


}
