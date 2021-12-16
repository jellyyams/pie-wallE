#include <arduinoFFT.h>
#include <Wire.h>
#include "filters.h";
#include "SoundData.h";
#include "XT_DAC_Audio.h";
#include <Adafruit_PWMServoDriver.h>


// Button mode setup

int buttonmode1 = 0; 
int buttonmode2 = 0; 
bool setUpBPM_bool = true;
bool setUpSpeaker_bool = true; 
bool setUpServos_bool = true; 
bool setUpLED_bool = true; 
bool setUpMotor_bool = true; 
bool runGreeting_bool = true; 
bool setUpDance_bool = true; 
int current_state1; 
int last_state1 = HIGH; 
int current_state2; 
int last_state2 = HIGH; 
int motor_mode = HIGH; 

// Dance setup

unsigned long prev_time_dance; 
int danceNum = 0;
int dance_period = 12; 

// Servo movement setup

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
int SERVO_FREQ = 50; // Analog servos run at ~50 Hz updates
int SERVOMIN  = 130; // This is the 'minimum' pulse length count (out of 4096)
int SERVOMAX  = 430; // This is the 'maximum' pulse length count (out of 4096)

double allBPM = 60; // bpm to set all of the servos to (so that it can be adjusted between servos so that some parts can move at half speed of others)
unsigned long baseTime = 20; // the ideal length of each loop (ms)
const int servosRunning = 7; // only try to run and find movement parameters for servos that have goalBPM, centerPos, maxRange, and maxVel defined

double centerPos[7] =   {95,        95,         107,      95,       105,       95,       95}; // (deg)
double maxRange[7] =    {30,        30,         106,      130,      110,       30,       30}; // maximum total angular travel defined by mechanical limits (deg)
double maxVel[7] =      {20,        20,         60,       60,       60,       30,       30}; // (deg/s)
double maxError = 1.0; // max difference between newBPM or closestBPM and goalBPM
double minStep = 0.1; // minimum step distance (deg)
double maxTimeMult = 5; // maximum baseTime multiplier to check

int numBeats[7] = {2,2,2,2,2,1,1}; 
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
int currMult[7] = {1,1,1,1,1,1,1}; // counter to only update servo position after timeMult baseTime intervals

unsigned long oldMilli = 0; // (ms timestamp)
unsigned long currMilli = 0; // (ms timestamp)
unsigned long nextMilli = baseTime; // (ms timestamp)       
unsigned long realStepLength = baseTime; // real time between the oldMilli and currMilli to check if adjustedDelay is working (ms)
unsigned long adjustedDelay = baseTime - currMilli; // time to wait at the end of each loop in order to make timesteps the right length (ms)

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
const int freq_min = 240;  
const int freq_max = 700; 
const int bpm_min = 20; 
const int bpm_max = 200; 
unsigned int last_beat_detected = 0; 
const int num_samples = 10; 
int index_beats = 0; 
int beats[num_samples]; 
long bpm_period = 1000; 
int LED1Mode = HIGH; 


//Pins

const uint8_t buttonPin1 = 34;
const uint8_t buttonPin2 = 39;
const uint8_t LEDPin1 = 26;
const uint8_t LEDPin2 = 32;
const uint8_t LEDPin3 = 15;
const uint8_t speakerPin = 25; 
const uint8_t microphonePin = 35;
const uint8_t motor1pin1 = 27;
const uint8_t motor1pin2 = 14;
const uint8_t motor2pin1 = 12;
const uint8_t motor2pin2 = 13;

//Servo pins
const uint8_t eyeRPin =  0;
const uint8_t eyeLPin =  1;
const uint8_t headPin =  2;
const uint8_t neckTPin = 3;
const uint8_t neckBPin = 4;
const uint8_t armLPin =  5;
const uint8_t armRPin =  6;


//Audio output setup

XT_DAC_Audio_Class DacAudio(speakerPin,0);    // Create the main player class object. Use GPIO 25, one of the 2 DAC pins and timer 0                                     
XT_Wav_Class WalleName(walle_name); 
XT_Wav_Class Tada(tada); 

void dance(){
   /* loops through a dance sequence */
   
  if(setUpDance_bool){
    //runs once when function is first called 
    prev_time_dance = millis(); 
    danceNum = 0; 
    setUpDance_bool = false; 
  }
  
  if(millis() - prev_time_dance > (bpm_period * dance_period)){
    //if the time elapsed exceeds a given period, move to next step in dance sequence 
    danceNum = danceNum %6 + 1; 
    prev_time_dance = millis(); 
  }

  if(danceNum < 5){
    //the first four time periods, walle dances with servos 
    moveServosToBPM(); 
    
  } else if (danceNum == 5){
    //the fifth time period, walle dances with dc motors 
    moveMotorsToBPM(); 
  } else if (danceNum == 6){
    //the sixth time period, walle says "tada", indicating the completion of his dance 
    stopMotors(); 
    playTada(); 
  }  
}

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
  /* set right eye to opposite side of range from left eye so that eyes raise and lower together */
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

void updatePos(uint8_t s) {
  /* Move servo if the servo should be moved in this loop, and update currPos and nextPos accordingly 
   * Run this every loop for each servo to move them at the right speed and range 
   */
   
  if (currMult[s] == timeMult[s]) { // check if it's been the right number of baseTime steps to move the servo again
    // if yes, reset the counter and get next position
    currMult[s] = 1;
    calcNextPos(s);
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

void calcNextPos(uint8_t s) {  
  /* Calculate the next position of servo s */
   if (nextPos[s] > maxPos[s]) {
    dir[s] = -1;
    double realBPM = (1/double(currMilli-lastSwitch[s]))*1000*60; // calculate the actual time it took to complete the whole range
    lastSwitch[s] = currMilli;
    Serial.println(realBPM); 
  } else if (nextPos[s] < minPos[s]) {
    dir[s] = 1;
    double realBPM = (1/double(currMilli-lastSwitch[s]))*1000*60; // calculate the actual time it took to complete the whole range
    lastSwitch[s] = currMilli;
    Serial.println(realBPM); 
  }
  nextPos[s] = curPos[s] + dir[s]*stepDist[s]; 
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

void moveServosToBPM(){
  /* Walle servos to match the bpm */
  if(setUpServos_bool){
    //runs once when function is first called 
    setUpServos(); 
   }
    
  currMilli = millis();
  // calculate how long the last loop actually took and record milli for next loop
  realStepLength = currMilli - oldMilli;
  oldMilli = currMilli;

  //Move a different set of servos depending on which dance number we're on 
  if(danceNum == 1){
    swingArms(); 
  } else if(danceNum ==2){
    moveEyes(); 
  } else if(danceNum == 3){
    bobHead(); 
  } else if(danceNum == 4){
    updatePos(neckTPin); 
    updatePos(armLPin);  
  }
  
  // Delay loop a certain amount to hit the given BPM
  nextMilli = currMilli + baseTime; // determine next step based on "current time" recorded at beginning of loop
  adjustedDelay = int(nextMilli-millis()); // calculate how many more ms to wait before starting next loop
  delay(adjustedDelay);

}

void setUpServos(){
  /* move all servos to starting position and calculate each servo's motion parameters based on bpm*/
  
  digitalWrite(LEDPin1, LOW);
  digitalWrite(LEDPin2, LOW);  

  setUpServos_bool = false; 
 
  for (uint8_t i = 0; i < servosRunning; i++) {
    range[i] = maxRange[i];
    minPos[i] = centerPos[i]-range[i]/2;
    maxPos[i] = centerPos[i]+range[i]/2;
    curPos[i] = minPos[i];
    nextPos[i] = minPos[i];
    
    double adjustedBPM = allBPM/numBeats[i];
    //modifies motion parameters to match BPM
    matchBPM(i,adjustedBPM);
    setServo(i); 
   }
}

void playWalleName() {
  /* play "walle" sound from speaker*/
  DacAudio.FillBuffer();  
  if(WalleName.Playing==false && Tada.Playing==false)
    DacAudio.Play(&WalleName);
}

void playTada() {
  /* play "tada" sound from speaker*/
  
  DacAudio.FillBuffer();  
  if(WalleName.Playing==false && Tada.Playing==false)    
    DacAudio.Play(&Tada);
}


void moveMotorsToBPM(){
   /* Move DC motors to BPM*/

  if (setUpMotor_bool){
    //runs once when this function is first called
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
  /*set up DC motors for movement*/
  
  Serial.println("set up motors"); 
  digitalWrite(LEDPin1, HIGH); 
  digitalWrite(LEDPin2, HIGH); 
  digitalWrite(LEDPin3, LOW); 
  setUpMotor_bool = false; 
  prev_time = millis(); 
  
}

void moveMotors(){
  /*move DC motors, swtich directioin they were preiously running*/

  if (motor_mode == HIGH){
    Serial.println("Set to forward"); 
    
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);

    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
    motor_mode = LOW; 
  } else {
    Serial.println("set to back"); 
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
    motor_mode = HIGH; 
  }
  
}

void stopMotors(){
  /*Stop moving all DC motors */
  
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  
}

void blinkToBPM(){
  /*Blink LEDs to BPM for use when calibrating walle's dancing */
  
  if (setUpLED_bool){
    //runs once when this function is first called
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
  /*Blink LED*/
  
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
  /*Runs a frequency analysis on audio input*/
  
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
  /*sums all of the frequency samples*/
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
  /*averages all the frequency samples*/
  double average = sum / SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = vReal[i] - average;
  }
}


int sortDesc(const void *cmp1, const void *cmp2){
  /*sorting function used when ordering the median bpm */
  int a = *((int *)cmp1);
  int b = *((int *)cmp2);
  // The comparison
  return b - a; 
}

void findMedianBPM(int beats[], int num_samples){
  /*Finds the median bpm out of a set amount of samples*/
  
  qsort(beats, num_samples, sizeof(beats[0]), sortDesc);
  for(int i = 0; i < num_samples; i++)
  {
    Serial.println(beats[i]);
  }
  int mid_index = num_samples / 2; 

  allBPM = beats[mid_index];  
  bpm_period = 1000 * (60 / allBPM); 
}

void listenForBPM(){
  /*Detects the frequency of metrenome beat and calculates time elapsed between each detection*/
  
  if(setUpBPM_bool){
    //runs once when this function is first called
    setUpBPM(); 
  }
  //Runs frequency analysis on audio input
  int dom_freq = runFFT();

  //If the dominant frequency is greater than and less than a certain threshold, calculate the time elapsed since last beat detected 
  if(dom_freq > freq_min && dom_freq < freq_max){
    Serial.println(dom_freq);
    storeAndFindBeats(); 

    //if enough beats have been detected, selected the median beat and set that as our detected bpm
    if(index_beats >= num_samples){
      findMedianBPM(beats, num_samples); 
      index_beats = 0; 
    }
  }
}

void setUpBPM(){
  /*Sets up BPM detection*/
  Serial.println("Starting BPM detection"); 
  digitalWrite(LEDPin3, LOW);
  digitalWrite(LEDPin2, LOW);
  last_beat_detected = millis();
  setUpBPM_bool = false;
  
}

void storeAndFindBeats(){
  /*stores the bpm detected in beats array*/
  
  float current_time = millis();
  float period = current_time - last_beat_detected; 
  float period_seconds = (period / 1000);
  float bpm = (60 / period_seconds);

  int bpm_int = int(bpm);

  //If bpm is less than certain threshold, we count that as a legitimate sample and store the detected bpm
  if(bpm_int < bpm_max){
    beats[index_beats] = (int)bpm; 
    index_beats += 1; 
    last_beat_detected = current_time;
  }
  
}

void runWalle(){
  /*Controls which mode walle is running*/

  //walle modes are controlled by button 1
  current_state1 = digitalRead(buttonPin1); 
    
  if (last_state1 == LOW && current_state1 == HIGH){
    buttonmode1 = buttonmode1%5 + 1; 
  }

  last_state1 = current_state1; 

  if (buttonmode1 == 1) {
    //Greeting mode
    digitalWrite(LEDPin1, HIGH); 
    digitalWrite(LEDPin2, HIGH);
    digitalWrite(LEDPin3, LOW);
    playWalleName(); 
   
  } else if (buttonmode1 == 2) {
    //BPM detection mode
    listenForBPM();
    blinkToBPM(); 
     
  } else if (buttonmode1 == 3) {
    //waiting mode, gives user time to set up walle for dancing 
    digitalWrite(LEDPin1, LOW); 
    digitalWrite(LEDPin2, HIGH);
    digitalWrite(LEDPin3, LOW); 

  } else if (buttonmode1 == 4){
    //dancing mode
    digitalWrite(LEDPin1, HIGH); 
    digitalWrite(LEDPin2, HIGH);
    digitalWrite(LEDPin3, LOW); 
    dance(); 
    
  }else if (buttonmode1 == 5){
    //Rests walle to be ready for another run through of modes
    stopMotors(); 
    digitalWrite(LEDPin1, LOW); 
    digitalWrite(LEDPin2, LOW);
    digitalWrite(LEDPin3, HIGH);
    runGreeting_bool = true; 
    setUpMotor_bool = true; 
    setUpBPM_bool = true; 
    setUpServos_bool = true;
    setUpSpeaker_bool = true;  
    setUpLED_bool = true; 
    setUpDance_bool = true; 
    
  }
}

void setup(){
  //set up servo driver communication
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  

  //set up frequency analysis 
  const float sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  //set up input and output pins 
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(LEDPin1, OUTPUT); 
  pinMode(LEDPin2, OUTPUT); 
  pinMode(LEDPin3, OUTPUT); 
  pinMode(buttonPin1, INPUT); 
  pinMode(buttonPin2, INPUT);
  pinMode(microphonePin, INPUT); 
  prev_time = millis();  

  //Makes sure both DC motors aren't running 
  digitalWrite(motor1pin1, LOW); 
  digitalWrite(motor1pin2, LOW); 
  digitalWrite(motor2pin1, LOW); 
  digitalWrite(motor2pin2, LOW); 
  
  Serial.begin(115200); 
  Serial.println("began"); 

  setUpServos(); 
}


void loop(){ 
  /*Main loop toggles between an "on" and "off" state using button 2*/
  
  current_state2 = digitalRead(buttonPin2); 
  if (last_state2 == LOW && current_state2 == HIGH){
    buttonmode2 = buttonmode2%2 + 1; 
  }

  last_state2 = current_state2; 

  if(buttonmode2 == 1){
    //off state. Walle won't move or take any other user input
    digitalWrite(LEDPin3, HIGH); 
  } else if (buttonmode2 == 2){
    //on state. Walle will run through modes and takes button input
    digitalWrite(LEDPin3, LOW); 
    runWalle(); 
  }
  
}
