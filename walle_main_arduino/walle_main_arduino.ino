#include <arduinoFFT.h>
#include <Servo.h>

#define SAMPLES 64
#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC

//FFT variables
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */


const uint16_t num_pixels = 60;
const uint16_t sound_thresh = 100;
uint8_t loop_num = 1;
uint16_t running_sum = 0; 
 
unsigned int sampling_period_us;
unsigned long microseconds;


double vReal[SAMPLES];
double vImag[SAMPLES];

boolean runningFFT = true; 

//Microphone variables
const uint8_t LEDPin = 8;
const uint8_t microphonePin = A0;

int sensorValue = 0; 

//Servo variables
Servo myservo1; 
int servo1Pin = 9; 
int servo_pos1 = 0; 

int previousMillis = 0; 

//DC motor variables
int motor1pin1 = 5;
int motor1pin2 = 4;
int motor2pin1 = 3;
int motor2pin2 = 2;

//void moveServo(){
//  for (servo_pos1 = 0; servo_pos1 <= 60; servo_pos1 += 1) { // goes from 0 degrees to 180 degrees
//    myservo1.write(servo_pos1);      
//    delay(30); // tell servo to go to position in variable 'pos'
//  }
//  
//  for (servo_pos1 = 60; servo_pos1 >= 0; servo_pos1 -= 1) { // goes from 0 degrees to 180 degrees
//    myservo1.write(servo_pos1);      
//    delay(30); // tell servo to go to position in variable 'pos'
//  }
//      
//}

void testDCMotor(){
  // put your main code here, to run repeatedly:   
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(1000);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(1000);

}

void testMicrophone(){
  sensorValue = analogRead(microphonePin); 
  Serial.println(sensorValue); 
 
  if(sensorValue > 450){
    digitalWrite(LEDPin, HIGH);
    delay(1000);
    digitalWrite(LEDPin, LOW);
  }  
}


double runFFT(){

  double average = 0;
  for (int i = 0; i < SAMPLES; i++) {
    while (!(ADCSRA & 0x10)); // wait on ADIF bit
    ADCSRA = 0b11110101; // clear ADIF bit
    vReal[i] = (double)ADC; 
    average += ADC;
    vImag[i] = 0;
    while(micros() < (microseconds + sampling_period_us)){
      }
  }

  average = average / SAMPLES;
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = vReal[i] - average;
  }


  // windows the data and computes the FFT
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  // finds largest frequency component, and computes overall volume
  double max_val = 0;
  uint16_t frequency = 0;
  for (int i = 0; i < SAMPLES/2; i++) {
    if (vReal[i] > max_val) {
      max_val = vReal[i];
      frequency = (20000 / (SAMPLES/2)) * i;
    }
  }
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  return peak;
}

void setup(){
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(LEDPin, OUTPUT); 
  pinMode(microphonePin, INPUT); 

  if (runningFFT){
    ADCSRA = 0xe5; // set the adc to free running mode
    ADMUX = 0x40; // use adc0
    DIDR0 = 0x01; // turn off the digital input for adc0
    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
    
  }

  myservo1.attach(servo1Pin); 
  Serial.begin(9600); 

}

void loop(){ 
//  running_sum += runFFT(); 
//  
//  if (loop_num > 9){
//    Serial.println(running_sum / loop_num);
//    running_sum = 0;
//    loop_num = 0;
//  }
//  loop_num += 1; 

    

  Serial.println(runFFT());

  
}
