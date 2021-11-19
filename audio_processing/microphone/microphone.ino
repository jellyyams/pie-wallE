#include <arduinoFFT.h>
#include <Servo.h>

//FFT variables
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

const uint16_t samples = 64;
const uint16_t num_pixels = 60;
const uint8_t loop_num = 1;
const uint16_t sound_thresh = 100;

double vReal[samples];
double vImag[samples];

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

void moveServo(){
  for (servo_pos1 = 0; servo_pos1 <= 60; servo_pos1 += 1) { // goes from 0 degrees to 180 degrees
    myservo1.write(servo_pos1);      
    delay(30); // tell servo to go to position in variable 'pos'
  }
  
  for (servo_pos1 = 60; servo_pos1 >= 0; servo_pos1 -= 1) { // goes from 0 degrees to 180 degrees
    myservo1.write(servo_pos1);      
    delay(30); // tell servo to go to position in variable 'pos'
  }
      
}

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


void runFFT(){
  // collects the samples from the ADC and computes the average
  int freq_avg = 0;
  int max_avg = 0;

  // we sample and compute the FFT a few times to reduce noise
  for (int loop_count = 0; loop_count < loop_num; loop_count++) {
    double average = 0;
    for (int i = 0; i < samples; i++) {
      while (!(ADCSRA & 0x10)); // wait on ADIF bit
      ADCSRA = 0b11110101; // clear ADIF bit
      vReal[i] = (double)ADC; 
      average += ADC;
      vImag[i] = 0;
    }

    average = average / samples;
    for (int i = 0; i < samples; i++) {
      vReal[i] = vReal[i] - average;
    }

    // windows the data and computes the FFT
    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, samples);

    // finds largest frequency component, and computes overall volume
    double max_val = 0;
    uint16_t frequency = 0;
    for (int i = 0; i < samples/2; i++) {
      if (vReal[i] > max_val) {
        max_val = vReal[i];
        frequency = (20000 / (samples/2)) * i;
      }
    }

    max_avg += max_val;
    freq_avg += frequency;
  }

  max_avg /= loop_num;
  freq_avg /= loop_num;
 
  
  Serial.println(freq_avg);
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
    
  }

  myservo1.attach(servo1Pin); 
  Serial.begin(115200); 

}

void loop(){ 
  runFFT(); 

  
  
}
