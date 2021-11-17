#include <arduinoFFT.h>
#include <Servo.h>

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 64; //This value MUST ALWAYS be a power of 2
const double signalFrequency = 1000;
const double samplingFrequency = 5000;
const uint8_t amplitude = 100;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

int LEDPin = 8; 

int sensorPin = A0;
int sensorValue = 0; 

Servo myservo1; 
int servo1Pin = 9; 
int servo_pos1 = 0; 

int previousMillis = 0; 


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


void runFFT(){
   /* Build raw data */
  double cycles = (((samples-1) * signalFrequency) / samplingFrequency); //Number of signal cycles that the sampling will read
  for (uint16_t i = 0; i < samples; i++)
  {
    vReal[i] = int8_t((amplitude * (sin((i * (twoPi * cycles)) / samples))) / 2.0);/* Build data with positive and negative values*/
    //vReal[i] = uint8_t((amplitude * (sin((i * (twoPi * cycles)) / samples) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }
  /* Print the results of the simulated sampling according to time */
  //Serial.println("Data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */

  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */

  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */

  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
//  Serial.println("dominant frequency"); 
//  Serial.println(x,6);
}

void setup(){
  pinMode(LEDPin, OUTPUT); 
  pinMode(sensorPin, INPUT); 
//  ADCSRA = 0xe5; // set the adc to free running mode
//  ADMUX = 0x40; // use adc0
//  DIDR0 = 0x01; // turn off the digital input for adc0
  myservo1.attach(servo1Pin); 
  Serial.begin(9600); 

}

void loop(){
  sensorValue = analogRead(sensorPin); 
  Serial.println(sensorValue); 
//  if(sensorValue > 450){
//    digitalWrite(LEDPin, HIGH);
//    delay(1000);
//    digitalWrite(LEDPin, LOW);
//  } 

  runFFT(); 
//  delay(10000); 

  
  
}
