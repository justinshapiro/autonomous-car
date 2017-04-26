extern "C" {
  #include <stdlib.h>
  #include <stdio.h>
  #include <string.h>
  #include <inttypes.h>
}

#include "Adafruit_VL6180X.h"

// Motor
#define pinI1     8  //define IN1 interface - motor A
#define pinI2     11 //define IN2 interface - motor A
#define speedpinA 9  //enable motor A
#define pinI3     12 //define IN3 interface - motor B
#define pinI4     13 //define IN4 interface - motor B
#define speedpinB 10 //enable motor B
#define _speed1   245 // define the speed of motor A
#define _speed2   200 // define the speed of motor B

// Ultrasonic sensor
//#define trigger 4
//#define echo 5

// Laser Range sensors
Adafruit_VL6180X lasers;
byte sensor1_pin = 3; 
byte sensor2_pin = 6;
byte sensor3_pin = 5;

const int distanceThreshold = 70;
const int wallDistance = 50;
const int maxSensorDistance = 255;

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

void slightlyLeft(){
  //input a simulation value to set the speed
  analogWrite(speedpinA,200);
  analogWrite(speedpinB,200);
  
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

void slightlyRight(){
    //input a simulation value to set the speed
  analogWrite(speedpinA,240);
  analogWrite(speedpinB,250);

  //turn DC Motor B move anticlockwise
  digitalWrite(pinI4,LOW);
  digitalWrite(pinI3,HIGH);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI1,HIGH);
}
 
void stop() {
  digitalWrite(speedpinA,LOW); 
  digitalWrite(speedpinB,LOW);
  delay(2000);
}

int getRight() {
  delay(50);
  digitalWrite(sensor2_pin, LOW); 
  digitalWrite(sensor3_pin, LOW);
  digitalWrite(sensor1_pin, HIGH); 
  lasers.begin(); 
  delay(1);
  Serial.println("Right: " + String(lasers.readRange()));
  return lasers.readRange();
}

int getLeft() {
  delay(50);
  digitalWrite(sensor1_pin, LOW); 
  digitalWrite(sensor3_pin, LOW);
  digitalWrite(sensor2_pin, HIGH); 
  lasers.begin();
  delay(1);
  Serial.println("Left: " + String(lasers.readRange()));
  return lasers.readRange();
}

int getFront() {
  delay(50);
  digitalWrite(sensor1_pin, LOW); 
  digitalWrite(sensor2_pin, LOW);
  digitalWrite(sensor3_pin, HIGH); 
  lasers.begin();
  delay(1);
  Serial.println("Front: " + String(lasers.readRange()));
  return lasers.readRange();
}

// Car control routine
void setup() {
  Serial.begin(9600);

  // Ultrasonic sensor
 // pinMode(trigger, OUTPUT);
  //pinMode(echo, INPUT);

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
  pinMode(sensor3_pin, OUTPUT);
  digitalWrite(sensor1_pin, LOW);
  digitalWrite(sensor2_pin, LOW);
  digitalWrite(sensor3_pin, LOW); 
  Wire.begin();

  // Start car
  forward();
}

void loop() {
  if (wallDistance > getRight()){
    slightlyLeft();
  }
  if (getLeft() < wallDistance){
    slightlyRight();
  }
  
  while (getFront() <= distanceThreshold) {
    backward();
    if (getRight() > maxSensorDistance){
      right();
    }
    if (getLeft() > maxSensorDistance){
      left();
    }
    while (getRight() <= distanceThreshold) {
      left();
    }
    while (getLeft() <= distanceThreshold){
      right();
    }
  }
  forward();
  
  //while (getLeft() <= distanceThreshold){
  //  right();
  //}
  //forward();

  //while (getRight() <= distanceThreshold) {
  //  left();
  //}
  //forward();
}
