/*
 * Note to self, Nov 30:
 * mpv is to play metronome for wallE for 10 seconds. 
 * Determine bpm then. Start dancing to song of known bpm (good start would be "the less i know the better")
 * Next step for bpm detection would be figuring out freq of most snares
 * Node32 board works with frequency analysis and speaker, need to test out motor driver still 
 */

#include <arduinoFFT.h>
#include "filters.h"



#define SAMPLES 64
int SAMPLING_FREQUENCY = 1000; //Hz, must be less than 10000 due to ADC

const float cutoff_freq  = 20;  //Cutoff frequency in Hz

const float sampling_time = 0.005; //Sampling time in seconds.
IIR::ORDER  order  = IIR::ORDER::OD3; // Order (OD1 to OD4)

// Low-pass filter
Filter f(cutoff_freq, sampling_time, order);

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */


double adc; 

const uint16_t num_pixels = 60;
const uint16_t sound_thresh = 100;
uint8_t loop_num = 1;
uint16_t running_sum = 0; 
 
unsigned int sampling_period_us;
unsigned long microseconds;


double vReal[SAMPLES];
double vImag[SAMPLES];

boolean runningFFT = true; 

//Pins
const uint8_t LEDPin = 8;
const uint8_t microphonePin = 36;

//DC motor variables
int motor1pin1 = 5;
int motor1pin2 = 4;
int motor2pin1 = 3;
int motor2pin2 = 2;

int previousMillis = 0; 

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
  adc = analogRead(microphonePin); 
  Serial.println(adc); 
 
  if(adc > 450){
    digitalWrite(LEDPin, HIGH);
    delay(1000);
    digitalWrite(LEDPin, LOW);
  }  
}


double runFFT(){


  double average = 0;
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long newTime= micros();  
    adc=f.filterIn(analogRead(microphonePin));
    vReal[i] = (double)adc; 
    average += adc;
    vImag[i] = 0;
    while(micros() < (microseconds + sampling_period_us)){}
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
  const float sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(LEDPin, OUTPUT); 
  pinMode(microphonePin, INPUT); 
  
  Serial.begin(115200); 

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
  //Serial.println(analogRead(microphonePin));

  
}
