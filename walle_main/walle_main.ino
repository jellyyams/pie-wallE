/*
 * Note to self, Nov 30:
 * mpv is to play metronome for wallE for 10 seconds. 
 * Determine bpm then. Start dancing to song of known bpm (good start would be "the less i know the better")
 * Next step for bpm detection would be figuring out freq of most snares
 * Node32 board works with frequency analysis and speaker, need to test out motor driver still 
 * 
 * Dec3: 
 * Tested out frequency parameters, 128 samples, 2000 hz, and cut off at 20 hz seems to work best. first order filter 
 * 
 * 
 * analogWrite statements may be commented out!!!!!
 */

#include <arduinoFFT.h>
#include "filters.h";
#include "SoundData.h";
#include "XT_DAC_Audio.h";
#include <Adafruit_PWMServoDriver.h>

int buttonmode1 = 0; 
int buttonmode2 = 0; 
bool setUpBPM_bool = true;
bool setUpSpeaker_bool = true; 
bool setUpServos_bool = true; 
bool setUpLED_bool = true; 
bool setUpMotor_bool = true; 
int current_state1; 
int last_state1 = HIGH; 
int current_state2; 
int last_state2 = HIGH; 

int motor_mode = HIGH; 

// Servo movement setup

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
int SERVOMIN  = 130; // This is the 'minimum' pulse length count (out of 4096)
int SERVOMAX  = 430; // This is the 'maximum' pulse length count (out of 4096)

double allBPM = 60; // bpm to set all of the servos to (so that it can be adjusted between servos so that some parts can move at half speed of others)
unsigned long baseTime = 20; // the ideal length of each loop (ms)
const bool waitToStart = false; // whether to wait for something to send in the Serial input before starting (in order to manually start on the beat)
const bool dispCalc = false; // whether to display movement parameter calculations in the matchBPM() function
const bool dispServoInfo = false; 
const bool adjBPM = false; // whether to run matchBPM with a corrected value (aka the adjustBPM() function) every time realBPM is calculated
const int servosRunning = 7; // only try to run and find movement parameters for servos that have goalBPM, centerPos, maxRange, and maxVel defined
const bool useBPM = false; // whether to use BPM to calculate range, stepDist, and timeMult
const int greetingLen = 10000; // time to spend on greeting sequence (ms)

double goalBPM[7] =     {allBPM/2,  allBPM/2,   allBPM,   allBPM,   allBPM,   allBPM,   allBPM};
double centerPos[7] =   {95,        95,         107,      95,       105,       95,       95}; // (deg)
double maxRange[7] =    {30,        30,         106,      130,      110,       30,       30}; // maximum total angular travel defined by mechanical limits (deg)
double maxVel[7] =      {20,        20,         60,       60,       60,       60,       60}; // (deg/s)
double maxError = 1.0; // max difference between newBPM or closestBPM and goalBPM
double minStep = 0.1; // minimum step distance (deg)
double maxTimeMult = 5; // maximum baseTime multiplier to check


int timeMult[7] = {1,1,1,1,1,1,1}; // initalize with default values
double stepDist[7] = {0.5,0.5,0.5,0.5,0.5,0.5,0.5}; // initalize with default values (deg)
double range[7]; // calculated ideal travel of servo (deg)
double closestBPM[7]; // closest bpm to goal bpm that can be calculated with contraints
double minPos[7]; // bottom of travel defined by range and centerPos - NOT necessarily the mechanical limits of the servo (deg)
double maxPos[7]; // top of travel defined by range and centerPos    -                      "                             (deg)
double curPos[7]; // (deg)
double nextPos[7]; // (deg)
int dir[7] = {-1,1,1,1,-1,1,1}; // direction servo is currently moving, + for towards maxPos, - for towards minPos
unsigned long lastSwitch[7]; // timestamp of last change of direction (ms)
double realBPM[7] = {allBPM,allBPM,allBPM,allBPM,allBPM,allBPM,allBPM}; // calculated from time of last direction change
double realVel[7]; // calculated from time of last direction change and range travelled (deg/s)
int currMult[7] = {1,1,1,1,1,1,1}; // counter to only update servo position after timeMult baseTime intervals
int danceNum = 0;


unsigned long oldMilli = 0; // (ms timestamp)
unsigned long currMilli = 0; // (ms timestamp)
unsigned long nextMilli = baseTime; // (ms timestamp)       
unsigned long realStepLength = baseTime; // real time between the oldMilli and currMilli to check if adjustedDelay is working (ms)
unsigned long adjustedDelay = baseTime - currMilli; // time to wait at the end of each loop in order to make timesteps the right length (ms)
unsigned long greetingStart = 0;

// Freqeuncy analysis setup

#define SAMPLES 128
int SAMPLING_FREQUENCY = 2000; //Hz, must be less than 10000 due to ADC
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
double adc; 
uint8_t sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long prev_time; 
unsigned long curr_time; 


// Low-pass filter

const float cutoff_freq  = 20;  //Cutoff frequency in Hz
const float sampling_time = 0.002; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD1; // Order (OD1 to OD4)
Filter f(cutoff_freq, sampling_time, order);


//Finding most common time between beats

uint8_t count = 0;
const int time_listening = 5; 
const int freq_thresh = 300;  
unsigned int last_beat_detected = 0; 
const int num_samples = 10; 
int index_beats = 0; 
int beats[num_samples]; 
long bpm_period = 0; 
int LED1Mode = HIGH; 


//Pins

const uint8_t buttonPin1 = 34;
const uint8_t buttonPin2 = 39;
const uint8_t LEDPin1 = 26;
const uint8_t LEDPin2 = 27;
const uint8_t LEDPin3 = 13;
const uint8_t speakerPin = 25; 
const uint8_t microphonePin = 36;
const uint8_t motor1pin1 = 18;
const uint8_t motor1pin2 = 5;
const uint8_t motor2pin1 = 16;
const uint8_t motor2pin2 = 17;
const uint8_t motor1speed = 15; 
const uint8_t motor2speed = 19; 

const uint8_t eyeRPin =  0;
const uint8_t eyeLPin =  1;
const uint8_t headPin =  2;
const uint8_t neckTPin = 3;
const uint8_t neckBPin = 4;
const uint8_t armLPin =  5;
const uint8_t armRPin =  6;


//Playing audio

XT_DAC_Audio_Class DacAudio(speakerPin,0);    // Create the main player class object. Use GPIO 25, one of the 2 DAC pins and timer 0                                     
XT_Wav_Class StarWars(Force); 


void nodHead() {
  /* Send servo commands for one step of the nodHead dance move */
  updatePos(neckTPin);
}

void shakeHead() {
  /* Sends servo commands for one step of the shakeHead dance move */
  updatePos(headPin);
}

void bobHead() {
  /* Send servo commands for one step of the bobHead dance move */
  updatePos(neckTPin);
  updatePos(neckBPin);
}

void swingArms() {
  /* Send servo commands for one step of the swingArms dance move */
  updatePos(armRPin);
  updatePos(armLPin);
}

void moveEyes() {
  /* Send servo commands for one step of the moveEyes dance move
   * Does one step of head tilting if setEyeModeTilt() was last to run
   * Does one step of eye lifting if setEyeModeLift() was last run
   */
  updatePos(eyeRPin);
  updatePos(eyeLPin);
}

void setEyeModeLift() {
  /* Set right eye to opposite side of range from left eye so that eyes raise and lower together */
  dir[eyeRPin] = -1;
  curPos[eyeRPin] = maxPos[eyeRPin];
  nextPos[eyeRPin] = maxPos[eyeRPin];
  dir[eyeLPin] = 1;
  curPos[eyeLPin] = minPos[eyeLPin];
  nextPos[eyeLPin] = minPos[eyeLPin];
}

void setEyeModeTilt() {
  /* Set right eye to same side of range as left eye so that eyes tilt together from side to side */
  dir[eyeRPin] = 1;
  curPos[eyeRPin] = minPos[eyeRPin];
  nextPos[eyeRPin] = minPos[eyeRPin];
  dir[eyeLPin] = 1;
  curPos[eyeLPin] = minPos[eyeLPin];
  nextPos[eyeLPin] = minPos[eyeLPin];
}

void runAllServos() {
  /* Run through all servos and update their positions */
  
  for (uint8_t i = 0; i < servosRunning; i++) { 
    updatePos(i); 
  }
}

void updatePos(uint8_t s) {
  /* Move servo if the servo should be moved in this loop, and update currPos and nextPos accordingly 
   * Run this every loop for each servo to move them at the right speed and range 
   */
   
  if (currMult[s] == timeMult[s]) { // check if it's been the right number of baseTime steps to move the servo again
    // if yes, reset the counter and get next position
    currMult[s] = 1;
    calcNextPos(s);
    if (dispServoInfo) { printVals(1); } // print values for this servo if user wants them displayed (for debugging)
    setServo(s); 
  } else {
    // if no, just add to counter and don't move servo
    currMult[s] = currMult[s] + 1;
  }
}

void setServo(uint8_t s) {
  /* Set servo s to the next position, update current position to match nextPos, and calculate pulselen from deg (of nextPos) */
  
  int pulselen = map(nextPos[s], 30, 160, SERVOMIN, SERVOMAX);
  pwm.setPWM(s, 0, pulselen);
  curPos[s] = nextPos[s];
}

void setServoPos(uint8_t s, double pos) {
  /* Directly set servo s to pos, update curPos and nextPos to pos */
  
  // calculate pulselen from deg (of pos)
  nextPos[s] = pos;
  int pulselen = map(nextPos[s], 30, 160, SERVOMIN, SERVOMAX);
  // set PWM to pulselen
  pwm.setPWM(s, 0, pulselen);
  // update curPos
  curPos[s] = nextPos[s];
}

void calcNextPos(uint8_t s) {  
  /* Calculate the next position of servo s */
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
  }
}

void matchBPM(uint8_t s, double BPM) {
  /* changes a servo's timeMult, stepDist, and range to hit a given BPM */
  
  int newTimeMult;
  double newStep;
  double newRange;
  double newBPM;
  double newError;
  double closestError=100;
  
  newTimeMult = 1; // the multiple of the baseTime to update servo position on
  while (newTimeMult <= maxTimeMult) { // prepare to test all timeMults 1->max
    
    if (abs(closestError) < maxError) { // if close enough, leave function
      return;
    }
    
    newStep = stepFromTimeMult(newTimeMult,maxVel[s]); // calculate step distance from timeMult and maxVel
    newRange = calcRange(BPM, newTimeMult, newStep);  // calculate range to achive bpm with defined baseTime multiple and step distance
    
    if (newRange > maxRange[s]) { // if the ideal range is outside of maxRange, then use maxRange and move servo slower (smaller step size)
      newRange = maxRange[s];
      newStep = calcStep(BPM, newTimeMult, newRange);
      Serial.println("Range too large");
    }
    
    newBPM = calcBPM(newTimeMult, newStep, newRange); // calculate the bpm with parameters determined above to check our work
    newError = newBPM-BPM;
    
    if (dispCalc) {
      Serial.println(newTimeMult);
      Serial.print("newStep=");
      Serial.println(newStep);
      Serial.print("newRange="); 
      Serial.println(newRange);
      Serial.print("newBPM="); 
      Serial.println(newBPM);
      Serial.print("newError="); 
      Serial.println(newError);
    }
      
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

double stepFromTimeMult(double newTimeMult, double vel) {
  /* Calculate step distance using time between steps (defined by newTimeMult*baseTime) and an angular speed of the servo (vel, in deg/s)
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
double calcStep(double bpm,int newTimeMult, double newRange) {
  /* Calculate range of movement based on a bpm, the time between steps (defined by newTimeMult*baseTime), and the step distance (deg)
  range (deg)   (min)           60 (s)   1000 (ms)      stepDist (deg)
  ----------- = --------   x    ----  x  -------    x   ----------------------
  (beat)        bpm (beat)      (min)    (s)            timeMult*baseTime (ms)
  */
  return newRange*bpm*double(newTimeMult*baseTime)/(60.0*1000.0);
}
double calcRange(double bpm,int newTimeMult, double newStep) {
  /* Calculate step distance based on a bpm, the time between steps (defined by newTimeMult*baseTime), and the range of movement (deg)
                    range (deg)     bpm (beat)    (min)       (s)
  stepDist (deg) =  -----------  x  ----------  x ------    x ---------  x  timeMult*baseTime (ms)
                    (beat)          (min)         60 (s)      1000 (ms)
  */
  return 60.0*1000.0*newStep/(bpm*double(newTimeMult*baseTime));
}
double calcBPM(int newTimeMult, double newStep, double newRange) {
  /* Calculates bpm based on the time between steps (defined by newTimeMult*baseTime), the step distance (deg), and the range of movement (deg)
  bpm (beat)    (beat)          stepDist (deg)                1000 (ms)     60 (s)
  ---------- =  -----------  x  ----------------------     x  ---------   x ------
  (min)         range (deg)     timeMult*baseTime (ms)        (s)           (min)
  */
  return newStep*1000.0*60.0/(newRange*double(newTimeMult*baseTime));
}

void printVals(uint8_t s) {
  // WARNING: Uncomment only currently relevant lines. Printing too many statements per loop can overflow Arduino/COM buffer and cause failure
//  Serial.print("s"); Serial.print(s); Serial.print(" stepDist="); Serial.println(stepDist[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" curPos="); Serial.println(curPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" range="); Serial.println(range[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" minPos="); Serial.println(minPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" maxPos="); Serial.println(maxPos[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" realBPM="); Serial.println(realBPM[s]);
//  Serial.print("s"); Serial.print(s); Serial.print(" realVel="); Serial.println(realVel[s]);
}

void moveToBPM(){
  if(setUpServos_bool){
      setUpServos(); 
   }
    
  currMilli = millis();
  // calculate how long the last loop actually took and record milli for next loop
  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
  
  runAllServos();
  
  
  // dynamically adjust length of delay based on how much time has already been spent in this loop and when the next one should start
  nextMilli = currMilli + baseTime; // determine next step based on "current time" recorded at beginning of loop
  adjustedDelay = int(nextMilli-millis()); // calculate how many more ms to wait before starting next loop
  delay(adjustedDelay);

}

void setUpServos(){
  digitalWrite(LEDPin1, LOW);
  digitalWrite(LEDPin2, LOW);  
  Serial.println("three pressed");
  setUpServos_bool = false; 
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
}
  


void testServoDriver(){
  for (int pulseLength = SERVOMIN ; pulseLength <= SERVOMAX ; pulseLength++)    //Move each servo from servoMin to servoMax
  {
    pwm.setPWM(0, 0, pulseLength);  //Set the current PWM pulse length on board 1, servo i
    delay(10);
    Serial.println(pulseLength); 
  }
  delay(500);
  for (int pulseLength = SERVOMAX ; pulseLength >= SERVOMIN ; pulseLength--)    ////Move each servo from servoMax to servoMin
  {
    pwm.setPWM(0, 0, pulseLength);           //Set the current PWM pulse length on board 1, servo i
    delay(10);
    Serial.println(pulseLength); 
  }
  delay(500);
  
}

void playAudio(){
  if(setUpSpeaker_bool){
    setUpSpeaker(); 
  } 
  
  DacAudio.FillBuffer();  
}

void setUpSpeaker(){
  Serial.println("Playing audio");
  StarWars.RepeatForever=false;        // Keep on playing sample forever!!!
  DacAudio.Play(&StarWars); 
  setUpSpeaker_bool = false; 
}

void moveMotorsToBPM(){

  if (setUpMotor_bool){
    setUpMotors(); 
  } else {
    if (millis() - prev_time >= (4*bpm_period)){
      Serial.println("changing direction"); 
      prev_time = millis(); 
      moveMotors(); 
    }
  } 
}

void setUpMotors(){
  Serial.println("set up motors"); 
  digitalWrite(LEDPin1, HIGH); 
  digitalWrite(LEDPin2, HIGH); 
  digitalWrite(LEDPin3, LOW); 
  setUpMotor_bool = false; 
  prev_time = millis(); 
  
}

void moveMotors(){
//  analogWrite(motor1speed, 10);
//  analogWrite(motor2speed, 10);
  

  if (motor_mode == HIGH){
    Serial.println("Set to forward"); 
    
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);

    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    motor_mode = LOW; 
  } else {
    Serial.println("set to back"); 
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    motor_mode = HIGH; 
  }
  
}

void stopMotors(){
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  
}

void testMicrophone(){
  adc = analogRead(microphonePin); 
  Serial.println(adc); 
 
  if(adc > 450){
    digitalWrite(LEDPin1, HIGH);
    delay(1000);
    digitalWrite(LEDPin1, LOW);
  }  
}
void blinkToBPM(){
  if (setUpLED_bool){
    Serial.println("set up leds"); 
    setUpLED_bool = false; 
    prev_time = millis(); 
  } else{
    if (millis() - prev_time >= bpm_period){
      Serial.println("blink"); 
      prev_time = millis(); 
      blinkLED(); 
    }
  }
}

void blinkLED(){
  
  if(LED1Mode == HIGH){
    digitalWrite(LEDPin1, LOW); 
    LED1Mode = LOW; 
    Serial.println("Set to low"); 
    
  } else{
    LED1Mode = HIGH; 
    Serial.println("set to high"); 
    digitalWrite(LEDPin1, HIGH);    
  }
}

double runFFT(){
  double sum = getFrequencySamples(); 
  averageFrequencySamples(sum); 

  // windows the data and computes the FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  return peak;
}

double getFrequencySamples(){
  double sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long newTime= micros();  
    adc=f.filterIn(analogRead(microphonePin));
    vReal[i] = (double)adc; 
    sum += adc;
    vImag[i] = 0;
    while(micros() < (microseconds + sampling_period_us)){}
  } 
  return sum; 
}

void averageFrequencySamples(double sum){
  double average = sum / SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = vReal[i] - average;
  }
}

int sortDesc(const void *cmp1, const void *cmp2){
  // Need to cast the void * to int *
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return b - a; 
}

void findMedianBPM(int beats[], int num_samples){
  qsort(beats, num_samples, sizeof(beats[0]), sortDesc);
  for(int i = 0; i < num_samples; i++)
  {
    Serial.println(beats[i]);
  }
  int mid_index = num_samples / 2; 
  Serial.println("median bpm"); 
  Serial.println(beats[mid_index]); 
  allBPM = beats[mid_index];  
  bpm_period = 1000 * (60 / allBPM); 
}

void listenForBPM(){
  if(setUpBPM_bool){
    setUpBPM(); 
    digitalWrite(LEDPin3, LOW);
    digitalWrite(LEDPin2, LOW);
  }
    
  int dom_freq = runFFT();
   
  Serial.println(dom_freq); 
  if(dom_freq > freq_thresh){
    storeAndFindBeats(); 
    
    if(index_beats >= num_samples){
      findMedianBPM(beats, num_samples); 
      index_beats = 0; 
    }
  }
}

void setUpBPM(){
  Serial.println("Starting BPM detection"); 
  last_beat_detected = millis();
  setUpBPM_bool = false;
  
}

void storeAndFindBeats(){
  float current_time = millis();
  float period = current_time - last_beat_detected; 
  
  float period_seconds = (period / 1000);
  float bpm = (60 / period_seconds);

  beats[index_beats] = (int)bpm; 
  index_beats += 1; 
  last_beat_detected = current_time;
  
}

void runWalle(){

  current_state1 = digitalRead(buttonPin1); 
    
  if (last_state1 == LOW && current_state1 == HIGH){
    buttonmode1 = buttonmode1%6 + 1; 
  }

  last_state1 = current_state1; 

  if (buttonmode1 == 1) {
    digitalWrite(LEDPin1, HIGH); 
    digitalWrite(LEDPin2, HIGH);
    digitalWrite(LEDPin3, LOW);
    //greeting
   
  } else if (buttonmode1 == 2) {
    listenForBPM();
    blinkToBPM(); 
     
  } else if (buttonmode1 == 3) {
    moveToBPM();
    
  } else if (buttonmode1 == 4){
    moveMotorsToBPM(); 
    
  } else if (buttonmode1 == 5){
    stopMotors(); 
    Serial.println("five pressed");
    digitalWrite(LEDPin1, LOW);
    digitalWrite(LEDPin2, LOW); 
//    playAudio(); 
  
  } else if (buttonmode1 == 6){
    digitalWrite(LEDPin1, LOW); 
    digitalWrite(LEDPin2, LOW);
    digitalWrite(LEDPin3, HIGH);

    setUpMotor_bool = true; 
    setUpBPM_bool = true; 
    setUpServos_bool = true;
    setUpSpeaker_bool = true;  
    setUpLED_bool = true; 
    Serial.println("Reset"); 
    
  }
}

void runDanceSeq() {
  
}

void runGreeting() {
  /* Do one step of greeting sequence of dance moves for the first greetingLen milliseconds, after greetingStart
   */
  if(setUpServos_bool){
     setUpServos(); 
  }
  
  if (currMilli < greetingLen/5) {
    moveEyes();
  } else if (currMilli < greetingStart + greetingLen*2/5) {
    shakeHead();
  } else if (currMilli < greetingStart + greetingLen*3/5) {
    nodHead();
  } else if (currMilli < greetingStart + greetingLen*4/5) {
    bobHead();
  } else if (currMilli < greetingStart + greetingLen) {
    swingArms();
  }
  
  keepTime();
}

void keepTime() {
  // dynamically adjust length of delay based on how much time has already been spent in this loop and when the next one should start
  nextMilli = currMilli + baseTime; // determine next step based on "current time" recorded at beginning of loop
  adjustedDelay = int(nextMilli-millis()); // calculate how many more ms to wait before starting next loop
  if (adjustedDelay < 0) {adjustedDelay = 0;}
  delay(adjustedDelay);

  currMilli = millis();
  // calculate how long the last loop actually took and record milli for next loop
  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;
}

void setup(){
  //set up servo driver communication
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  const float sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(motor1speed, OUTPUT); 
  pinMode(motor2speed, OUTPUT); 
  pinMode(LEDPin1, OUTPUT); 
  pinMode(LEDPin2, OUTPUT); 
  pinMode(LEDPin3, OUTPUT); 
  pinMode(buttonPin1, INPUT); 
  pinMode(buttonPin2, INPUT);
  pinMode(microphonePin, INPUT); 
  prev_time = millis();  

  digitalWrite(motor1pin1, LOW); 
  digitalWrite(motor1pin2, LOW); 
  digitalWrite(motor2pin1, LOW); 
  digitalWrite(motor2pin2, LOW); 
  
  Serial.begin(115200); 
  Serial.println("began"); 
}


void loop(){ 
//when button is clicked, switch mode
  current_state2 = digitalRead(buttonPin2); 

  
  if (last_state2 == LOW && current_state2 == HIGH){
    buttonmode2 = buttonmode2%2 + 1; 
    greetingStart = millis();
  }

  last_state2 = current_state2; 
  listenForBPM(); 

  if(buttonmode2 == 1){
    digitalWrite(LEDPin3, HIGH); 
  } else if (buttonmode2 == 2) {
    runGreeting();
  } else if (buttonmode2 == 3){
    digitalWrite(LEDPin3, LOW); 
    runWalle(); 
  }
  
}
