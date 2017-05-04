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
#define _speed1   150 // define the speed of motor A (left wheel)
#define _speed2   150 // define the speed of motor B (right wheel)

// Ultrasonic sensor
#define trigger 6
#define echo 5

// Laser Range sensors
Adafruit_VL6180X lasers;
byte sensor1_pin = 2; // right
byte sensor2_pin = 6; // left

const int distanceThreshold = 255; 
const int wallDistance = 40;
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

  if (getLeft() <= wallDistance) {
    slightlyRight();
  }

  if (getRight() <= wallDistance) {
    slightlyLeft();
  }
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
  stop();
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
  stop();
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
 
void stop() {
  digitalWrite(speedpinA,LOW); 
  digitalWrite(speedpinB,LOW);
  delay(500);
}

void stop(int stop_time) {
  digitalWrite(speedpinA,LOW); 
  digitalWrite(speedpinB,LOW);
  delay(stop_time);
}

int getRight() {
  delay(50);
  digitalWrite(sensor2_pin, LOW); 
  digitalWrite(sensor1_pin, HIGH); 
  lasers.begin(); 
  delay(1);
  // Serial.println("Right: " + String(lasers.readRange()));
  return lasers.readRange();
}

int getLeft() {
  delay(50);
  digitalWrite(sensor1_pin, LOW); 
  digitalWrite(sensor2_pin, HIGH); 
  lasers.begin();
  delay(1);
  // Serial.println("Left: " + String(lasers.readRange()));
  return lasers.readRange();
}

int getFront() {
  int result = -1;
  while (result <= 0 || result > 255) {
    digitalWrite(trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    short duration = pulseIn(echo, HIGH);
    result = ((5 * duration) / 29.1) - 50;
  }
  return result;
}

void go90Left() {
  left();
  delay(750);  
}

void go90Right() {
  right();
  delay(750);  
}

int getState() {
  _front = getFront();
  Serial.println("Front: " + String(_front));
  _right = getRight();
  _left = getLeft();
  
  if (_front >= wallDistance) {
    front_free = true;
  } else {
    front_free = false;
  }

  if (_left == distanceThreshold) {
    left_free = true;
  } else {
    left_free = false;
  }

  if (_right == distanceThreshold) {
    right_free = true;
  } else {
    right_free = false;
  }

  /*
   *          F L R
   * State 0: 0 0 0 - Go Straight
   * State 1: 0 0 1 - Go 90 Left
   * State 2: 0 1 0 - Go 90 Right
   * State 3: 0 1 1 - Go Straight
   * State 4: 1 0 0 - Go 90 Left
   * State 5: 1 0 1 - Go 90 Left
   * State 6: 1 1 0 - Go 90 Right
   * State 7: 1 1 1 - Turn Around
   * 
   * 0 = Nothing in front of it
   * 1 = Something in front of it but not at wallDistance
   */

  if (front_free && left_free && right_free) {
    return 0;
  } else if (front_free && left_free && !right_free) {
    return 1;
  } else if (front_free && !left_free && right_free) {
    return 2;
  } else if (front_free && !left_free && !right_free) {
    return 3;
  } else if (!front_free && left_free && right_free) {
    return 4;
  } else if (!front_free && left_free && !right_free) {
    return 5;
  } else if (!front_free && !left_free && right_free) {
    return 6;
  } else if (!front_free && !left_free && !right_free) {
    return 7;
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

  delay(2500);
  // Start car
  forward();
}

void loop() {
  // State Filtering
  int state = getState();
  stop(0);
  int curr_state = state;
  for (int i = 0; i < 10; i++) {
    int state = getState();
    if (state != curr_state) {
      i = 0;
      curr_state = state;
    }
    delay(20);
  }
 
  switch (state) {
    case 0: 
    case 3: {
      // Move forward
      forward();
      delay(2000);
    } break;
    case 2: 
    case 6: {
      // Move right, then forward
      go90Right();
      stop();
      forward();
      delay(2000);
    } break;
    case 1: 
    case 4: 
    case 5: {
      // Move left, then forward
      go90Left();
      stop();
      forward();
      delay(2000);
    } break;
    case 7: {
        // Turn around, then forward
        go90Right();
        stop();
        go90Right();
        stop();
        forward();
        delay(2000);
      } break;
    default: forward(); break;
  }
}
