extern "C" {
  #include <stdlib.h>
  #include <stdio.h>
  #include <string.h>
  #include <inttypes.h>
}
#include "Adafruit_VL6180X.h"

// Motor
#define pinI1     8  //define IN1 interface
#define pinI2     11 //define IN2 interface 
#define speedpinA 9  //enable motor A
#define pinI3     12 //define IN3 interface 
#define pinI4     13 //define IN4 interface 
#define speedpinB 10 //enable motor B
#define _speed1   250 // define the speed of motor A
#define _speed2   210 // define the speed of motor B

// Ultrasonic sensor
#define trigger 4
#define echo 5

// Laser Range sensors
Adafruit_VL6180X lasers;
byte sensor1_pin = 3; // SHutDowN pin
byte sensor2_pin = 6;

const int distanceThreshold = 50;
bool stopped = false;

// Motor control routines
void forward() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,_speed1);
  analogWrite(speedpinB,_speed2);

  //turn DC Motor B move clockwise
  digitalWrite(pinI4,HIGH);
  digitalWrite(pinI3,LOW);

  //turn DC Motor A move anticlockwise
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI1,HIGH);
}
 
void backward() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,_speed1);
  analogWrite(speedpinB,_speed2);
  
  //turn DC Motor B move anticlockwise
  digitalWrite(pinI4,LOW);
  digitalWrite(pinI3,HIGH);
  
  //turn DC Motor A move clockwise
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI1,LOW);
}
 
void left() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,_speed1);
  analogWrite(speedpinB,_speed2);
  
  //turn DC Motor B move clockwise
  digitalWrite(pinI4,HIGH);
  digitalWrite(pinI3,LOW);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI1,LOW);
}
 
void right() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,_speed1);
  analogWrite(speedpinB,_speed2);

  //turn DC Motor B move anticlockwise
  digitalWrite(pinI4,LOW);
  digitalWrite(pinI3,HIGH);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI1,HIGH);
}
 
void stop() {
  digitalWrite(speedpinA,LOW);
  //Unenble the pin, to stop the motor. This should be done to avid damaging the motor. 
  digitalWrite(speedpinB,LOW);
  delay(2000);
}

int getRight() {
  digitalWrite(sensor2_pin, LOW); 
  digitalWrite(sensor1_pin, HIGH); 
  lasers.begin(); 
  delay(1);
  return lasers.readRange();
}

int getLeft() {
  digitalWrite(sensor1_pin, LOW); 
  digitalWrite(sensor2_pin, HIGH); 
  lasers.begin();
  delay(1);
  return lasers.readRange();
}

int getUltrasonic() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  short duration = pulseIn(echo, HIGH);
  return (5 * duration) / 29.1;
}

// Car control routine
void setup() {
  Serial.begin(9600);

  // Ultrasonic sensor
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  // Motor
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speedpinB,OUTPUT);

  // Laser Range sensor
  pinMode(sensor1_pin, OUTPUT);
  pinMode(sensor2_pin, OUTPUT);
  digitalWrite(sensor1_pin, LOW); // Disable laser1
  digitalWrite(sensor2_pin, LOW); // Disable laser2
  Wire.begin();

  // Start car
  forward();
}

void loop() {
  // Laser Range sensor
  Serial.println("Left: " + String(getLeft()));
  delay(50);
  Serial.println("Right: " + String(getRight()));
  delay(50);
}
