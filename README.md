# Dancing Wall-E

## Overview 

This repo is part of larger, integrated engineering project where we built a Wall-E replica with dancing abilities. You can find more details for this project [here](https://olincollege.github.io/pie-2021-03/Dancing-Wall-e/). Wall-E runs entirely on an esp32, and has seperate chips that allow it to take audio input, control two DC motors, control 7 servos, and output audio to a speaker. 

The primary file lives in the "walle_main" and is called "walle_main.ino". Run this file from the Arduino IDE when connected to an esp32 or any other microcontroller with I2C, DAC, and ADC pins. 

## Dependencies 

This project relies on the [arduinoFFT library](https://github.com/kosme/arduinoFFT), [Xtronical DAC audio library](https://www.xtronical.com/the-dacaudio-library-download-and-installation/), [Adafruit PWM servo driver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library) library, and the [arduino filter](https://tttapa.github.io/Arduino-Filters/Doxygen/d9/de0/Butterworth_8ino-example.html) library. The arduino FFT and Adafruit libraries need to be installed via the Arduino IDE library manager. 

