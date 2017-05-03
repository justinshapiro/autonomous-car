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
#define _speed1   120 // define the speed of motor A (left wheel)
#define _speed2   120 // define the speed of motor B (right wheel)

// Ultrasonic sensor
#define trigger 6
#define echo 5

// Laser Range sensors
Adafruit_VL6180X lasers;
byte sensor1_pin = 2; // right
byte sensor2_pin = 6; // left

const int distanceThreshold = 255; 
const int wallDistance = 50;
int _front;
int _left;
int _right;
bool front_free = false;
bool left_free = false;
bool right_free = false;

// Motor control routines
void forward() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,_speed1);
  analogWrite(speedpinB,_speed2);

  //turn DC Motor B move clockwise
  digitalWrite(pinI4,LOW);
  digitalWrite(pinI3,HIGH);

  //turn DC Motor A move anticlockwise
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI1,LOW);

  if (getLeft() < wallDistance) {
    slightlyRight();
  }

  if (getRight() < wallDistance) {
    slightlyLeft();
  }
}
 
void backward() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,_speed1);
  analogWrite(speedpinB,_speed2);
  
  //turn DC Motor B move anticlockwise
  digitalWrite(pinI4,HIGH);
  digitalWrite(pinI3,LOW);
  
  //turn DC Motor A move clockwise
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI1,HIGH);
}
 
void left() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,_speed1);       // need to make these speed values smaller if we 
  analogWrite(speedpinB,_speed2);       // want smaller turns (or really small and add a delay as necessary)
  
  //turn DC Motor B move clockwise
  digitalWrite(pinI4,HIGH);
  digitalWrite(pinI3,LOW);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI1,LOW);
}
 
void right() {
  //input a simulation value to set the speed
  analogWrite(speedpinA, _speed1);
  analogWrite(speedpinB, _speed2);

  //turn DC Motor B move anticlockwise
  digitalWrite(pinI4,LOW);
  digitalWrite(pinI3,HIGH);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI1,HIGH);
}

void slightlyRight(){
  //input a simulation value to set the speed
  analogWrite(speedpinA, _speed1);
  analogWrite(speedpinB, _speed2);

  //turn DC Motor B move anticlockwise
  digitalWrite(pinI4,LOW);
  digitalWrite(pinI3,HIGH);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI1,HIGH);

  delay(200);
}

void slightlyLeft(){
  //input a simulation value to set the speed
  analogWrite(speedpinA,_speed1);
  analogWrite(speedpinB,_speed2);
  
  //turn DC Motor B move clockwise
  digitalWrite(pinI4,HIGH);
  digitalWrite(pinI3,LOW);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI1,LOW);

  delay(200);
}
 
void stop() {
  digitalWrite(speedpinA,LOW); 
  digitalWrite(speedpinB,LOW);
  delay(2000);
}

int getRight() {
  delay(50);
  digitalWrite(sensor2_pin, LOW); 
  digitalWrite(sensor1_pin, HIGH); 
  lasers.begin(); 
  delay(1);
  Serial.println("Right: " + String(lasers.readRange()));
  return lasers.readRange();
}

int getLeft() {
  delay(50);
  digitalWrite(sensor1_pin, LOW); 
  digitalWrite(sensor2_pin, HIGH); 
  lasers.begin();
  delay(1);
  Serial.println("Left: " + String(lasers.readRange()));
  return lasers.readRange();
}

int getFront() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  short duration = pulseIn(echo, HIGH);
  int result = abs((5 * duration) / 29.1);
  if (result > 255 || result == 0) {
    result = 255;
  }
  Serial.println("Front: " + String(result));
  return result;
}

void go90Left() {
  left();
  delay(400);  
}

void go90Right() {
  right();
  delay(400);  
}

int getState() {
  _front = getFront();
  _right = getRight();
  _left = getLeft();

  if (_front > wallDistance) {
    front_free = true;
  } else {
    front_free = false;
  }

  if (_left > distanceThreshold) {
    left_free = true;
  } else {
    left_free = false;
  }

  if (_right > distanceThreshold) {
    right_free = true;
  } else {
    right_free = false;
  }

  if (front_free && !left_free && !right_free) {
    return 0;
  } else if (front_free && left_free && !right_free) {
    return 0;
  } else if (front_free && left_free && right_free) {
    return 0;
  } else if (!front_free && left_free && right_free) {
    return 1;
  } else if (!front_free && !left_free && right_free) {
    return 1;
  } else if (!front_free && !left_free && !right_free) {
    return 2;
  } else if (!front_free && left_free && !right_free) {
    return 3;
  } 
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
  digitalWrite(sensor1_pin, LOW);
  digitalWrite(sensor2_pin, LOW);
  Wire.begin();

  delay(5000);
  // Start car
  forward();
}

void loop() {
  int state = getState();
  switch (state) {
    case 0: {
      // Move forward
      forward();
    } break;
    case 1: {
      // Move right, then forward
      go90Right();
      forward();
    } break;
    case 2: {
      // Turn around, then forward
      go90Right();
      go90Right();
      forward();
    } break;
    case 3: {
      // Move left, then forward
      go90Left();
      forward();
    } break;
    default: break;
  }

  /*
  // Handle the event that something is in the front
  if (_front <= distanceThreshold) {
    if (_left > wallDistance) {
      go90Left();
    } else if (_right > wallDistance) {
      go90Right();
    }
  }

  forward();
  if (getLeft() <= wallDistance) {
    slightlyRight();
  }
  if (getRight() <= wallDistance) {
    slightlyLeft();
  }*/
}
