/* This code calculates the temperature using a PT100, 
 * Written by Roboro
 * Github: https://github.com/RonanB96/Read-Temp-From-PT100-With-Arduino
 * Circuit: https://circuits.io/circuits/2962051-reading-temperature-from-pt100/
 * Blog: https://roboroblog.wordpress.com
 * Instrustable Post:http://www.instructables.com/id/Reading-Temperature-From-PT100-Using-Arduino/ 
  */

// You'll need to download this timer library from here
// http://www.doctormonk.com/search?q=timer 
#include <Servo.h> 
#include "Timer.h"
#include <Wire.h> 
int servoPin = 3;
// Define Variables
float V;
float temp;
float Rx;
Servo Servo1;

// Variables to convert voltage to resistance
float C = 100;
float slope = 10.756;

// Variables to convert resistance to temp
float R0 = 100.0;
float alpha = 0.00385;

int Vin = A0; // Vin is Analog Pin A0

Timer t; // Define Timer object



int temp_read_Delay = 500;
int real_temperature = 0;
int setpoint = 80;


float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int PID_value = 0;
//PID constants
int kp = 203;   int ki= 7.2;   int kd = 1.04;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

void setup() {
  Serial.begin(9600); // Set Baudrate at 9600
  pinMode(Vin,INPUT); // Make Vin Input
  t.every(100,takeReading); // Take Reading Every 100ms
   Servo1.attach(servoPin);
}

void loop() {
  t.update(); // Update Timer

  PID_error = setpoint - temp;        //Calculate the pid ERROR
    
    if(PID_error > 30)                              //integral constant will only affect errors below 30ºC             
    {PID_i = 0;}
    
    PID_p = kp * PID_error;                         //Calculate the P value
    PID_i = PID_i + (ki * PID_error);               //Calculate the I value
    timePrev = Time;                    // the previous time is stored before the actual time read
    Time = millis();                    // actual time read
    elapsedTime = (Time - timePrev) / 1000;   
    PID_d = kd*((PID_error - previous_error)/elapsedTime);  //Calculate the D value
    PID_value = PID_p + PID_i + PID_d;  //Calculate total PID value

if(PID_value < 0)
    {      PID_value = 0;       }
    if(PID_value > 7400)
    {      PID_value = 7400;    }

   Servo1.write(PID_value*0.02432);
  delay(1000);
}

void takeReading(){
  // Bits to Voltage
  V = (analogRead(Vin)/1023.0)*5.0; // (bits/2^n-1)*Vmax 
  // Voltage to resistance
  Rx = V*slope+C; //y=mx+c
  // Resistance to Temperature
  temp= (Rx/R0-1.0)/alpha; // from Rx = R0(1+alpha*X) 
  Serial.println(temp);
}
