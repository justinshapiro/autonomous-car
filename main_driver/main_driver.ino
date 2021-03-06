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
#define _speed2   250 // define the speed of motor B

// Ultrasonic sensor
#define trigger 4
#define echo 5

// Laser Range sensors
Adafruit_VL6180X lasers;
byte sensor1_pin = 3; 
byte sensor2_pin = 6;
byte sensor3_pin = 5;

const int distanceThreshold = 70;

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

/********** Navigation **********/


const unsigned char NAVIGATION_FRONT = 0x01;//   0001
const unsigned char NAVIGATION_LEFT =  0x02;//   0010
const unsigned char NAVIGATION_RIGHT = 0x04;//   0100
//Reserved for future additions:
const unsigned char NAVIGATION_TURNING = 0x08;// 1000

const int NAVIGATION_THRESHOLD = 10;

unsigned char getState()
{
    unsigned char state = 0;
    
    if(getFront() < NAVIGATION_THRESHOLD)
        state |= NAVIGATION_FRONT;
    if(getLeft() < NAVIGATION_THRESHOLD)
        state |= NAVIGATION_LEFT;
    if(getRight() < NAVIGATION_THRESHOLD)
        state |= NAVIGATION_RIGHT;
    
    return state;
}

void navigation_mainLoop()
{
    switch(getState())
    {
    case NAVIGATION_FRONT:
        go90Right();
        break;
    case NAVIGATION_LEFT:
        forward();
        break;
    case NAVIGATION_RIGHT:
        forward();
        break;
    case NAVIGATION_FRONT | NAVIGATION_LEFT:
        go90Right();
        break;
    case NAVIGATION_FRONT | NAVIGATION_RIGHT:
        go90Left();
        break;
    case NAVIGATION_LEFT | NAVIGATION_RIGHT:
        forward();
        break;
    case NAVIGATION_FRONT | NAVIGATION_LEFT | NAVIGATION_RIGHT:
        go90Right();
        go90Right();
        break;
    }
    delay(400);
    stop();
}

/********** End navigation **********/

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
  pinMode(sensor3_pin, OUTPUT);
  digitalWrite(sensor1_pin, LOW);
  digitalWrite(sensor2_pin, LOW);
  digitalWrite(sensor3_pin, LOW); 
  Wire.begin();

  // Start car
  forward();
}

void loop() {
  /*while (getFront() <= distanceThreshold) {
    backward();
  }
  forward();

  while (getLeft() <= distanceThreshold){
    right();
  }
  forward();

  while (getRight() <= distanceThreshold) {
    left();
  }
  forward();*/
  //Does arduino call "loop" repetedly on its own,
  //or does this need to be surrounded by a "while(true)"?
  navigation_mainLoop();
}
