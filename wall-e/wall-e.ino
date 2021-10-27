
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
int topNeckPos;
int botNeckPos;
int leftArmPos;
int rightArmPos;

int currBPM = 60;    // initialize music BPM. Update with change in music
int stepLength;      // calculated from bpm
int beatStart = 0;
int currMili;
int nextMilli = beatStart;        
int allowedError = 50; // milliseconds we can be off by

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // open the serial port at 9600 bps:
  leftEyeServo.attach(9);  // attaches the servo on pin __ to the leftEyeServo object
  rightEyeServo.attach(10);  // attaches the servo on pin __ to the rightEyeServo object
  headPanServo.attach(11);  // attaches the servo on pin __ to the headPanServo object
  topNeckServo.attach(12);  // attaches the servo on pin __ to the topNeckServo object
  botNeckServo.attach(13);  // attaches the servo on pin __ to the botNeckServo object

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
  
}

void swingArms() {
  
}

void tiltHead() {
  
}



void loop() {
  // put your main code here, to run repeatedly:

  // check the current time and wait until nextMili to start the next step
  currMilli = millis();
  while (currMilli() < nextMilli) {
    // wait for stepLength before starting the next step
  }
  if (abs(currMilli()-nextMilli) > allowedError) {
    Serial.print("Error: running "); Serial.print(currMilli()-nextMilli)); Serial.println(" ms behind");
  }
  nextMilli = currMilli + stepLength;

  // run or continue these dance moves

  
  


}
