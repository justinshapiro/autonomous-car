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
#define _speed1   175 // define the speed of motor A (left wheel)
#define _speed2   175 // define the speed of motor B (right wheel)

// Ultrasonic sensor
#define trigger 6
#define echo 5

// Laser Range sensors
Adafruit_VL6180X lasers;
byte sensor1_pin = 2; // right
byte sensor2_pin = 7; // left

// Control variables
const byte distanceThreshold = 255; 
const byte wallDistance = 25;
const byte statesToFilter = 5;
byte _front;
byte _left;
byte _right;
bool front_free = false;
bool left_free = false;
bool right_free = false;
byte last_state;
byte front255 = 0;

// Change these based on battery power and floor type
const int frontOffset = -50;
const short forwardLength = 300;
const short go90Length = 500;
const byte slightlyLength = 100;

// Motor control routines
void forward(bool correction) {
  Serial.println("Going forward");
  short delay_count = 0;
  while (delay_count < forwardLength) {
    //input a simulation value to set the speed
    analogWrite(speedpinA,_speed1);
    analogWrite(speedpinB,_speed2);
  
    //turn DC Motor B move clockwise
    digitalWrite(pinI4,LOW);
    digitalWrite(pinI3,HIGH);
  
    //turn DC Motor A move anticlockwise
    digitalWrite(pinI2,HIGH);
    digitalWrite(pinI1,LOW);

    if (correction) {
      if (getLeft() <= wallDistance) {
        slightlyRight();
      }
    
      if (getRight() <= wallDistance) {
        slightlyLeft();
      }
    }

    delay(50);
    delay_count += 50;
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

  delay(slightlyLength); 
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

  delay(slightlyLength);
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
  analogWrite(speedpinA,_speed1 + 30);       // need to make these speed values smaller if we 
  analogWrite(speedpinB,_speed2 + 30);       // want smaller turns (or really small and add a delay as necessary)
  
  //turn DC Motor B move clockwise
  digitalWrite(pinI4,HIGH);
  digitalWrite(pinI3,LOW);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI1,LOW);
}
 
void right() {
  //input a simulation value to set the speed
  analogWrite(speedpinA, _speed1 + 30);
  analogWrite(speedpinB, _speed2 + 30);

  //turn DC Motor B move anticlockwise
  digitalWrite(pinI4,LOW);
  digitalWrite(pinI3,HIGH);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI1,HIGH);
}
 
void stop(short stop_time) {
  digitalWrite(speedpinA,LOW); 
  digitalWrite(speedpinB,LOW);
  delay(stop_time);
}

byte getRight() {
  delay(10);
  digitalWrite(sensor2_pin, LOW); 
  digitalWrite(sensor1_pin, HIGH); 
  lasers.begin(); 
  delay(1);
  return lasers.readRange();
}

byte getLeft() {
  delay(10);
  digitalWrite(sensor1_pin, LOW); 
  digitalWrite(sensor2_pin, HIGH); 
  lasers.begin();
  delay(1);
  return lasers.readRange();
}

short getFront() {
  int result = 0;
  for (byte i = 0; i < 10; i++) {
    digitalWrite(trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    
    long duration = pulseIn(echo, HIGH);
    short this_result = ((5 * duration) / 29.1) + frontOffset;
    //Serial.println("Raw front data: " + String(this_result));
    if (this_result > 255) {
      this_result = 255;
    }

    result += this_result;
  }
  
  return result / 10;
}

void turnAround() {
  right();
  delay(2 * go90Length);  
}

void go90Left(bool _forward) {
  if (_forward) {
    forward(false);
    forward(false);
  }
  left();
  delay(go90Length);  
  stop(100);
  forward(true);
}

void go90Right(bool _forward) {
  if (_forward) {
    forward(false);
    forward(false);
  }
  right();
  delay(go90Length);  
  stop(100);
  forward(true);
}

byte getState() {
  _front = getFront();
  _right = getRight();
  _left = getLeft();

  Serial.println("Front: " + String(_front));
  Serial.println("Left: " + String(_left));
  Serial.println("Right: " + String(_right));
  
  if (_front > wallDistance && _front <= distanceThreshold) {
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
   *          -----
   * State 0: 0 0 0 - Go Straight
   * State 1: 0 0 1 - Go 90 Left
   * State 2: 0 1 0 - Go 90 Right
   * State 3: 0 1 1 - Go Straight
   * State 4: 1 0 0 - Go 90 Right
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

  // Pause before starting the system
  delay(2500);
}

void loop() {
  // State Filtering
  stop(0);
  int state = getState();
  int curr_state = state;
  for (int i = 0; i < statesToFilter; i++) {
    state = getState();
    if (state != curr_state) {
      i = 0;
      curr_state = state;
    }
    delay(5);
  }

  byte _front = getFront();

  if (_front == 255 || _front < 5) {
    front255++;
  } else {
    front255 = 0;
  }

  Serial.println("In state:" + String(state));

  if ((state == 2 || state == 3) && (last_state == 1 || last_state == 5)) {
    state = 6;
  }
 
  switch (state) {
    case 0: 
    case 2: 
    case 3: { forward(true); } break;
    case 4: 
    case 6: { go90Right(true); } break;
    case 1: 
    case 5: { go90Left(true); } break;
    case 7: {
        if (last_state != 2 || last_state != 4 || last_state != 6) {
          turnAround();
          forward(true);
        } else {
          exit(0);
        }
      } break;
    default: forward(true); break;
  }

  if (last_state == 4 && state == 4) {
    backward();
    delay(500);
    go90Left(false);
  }

  last_state = state;

  Serial.println("Front255 = " + String(front255));
  if (front255 == 3) {
    backward();
    delay(500);
    slightlyLeft();
    front255 = 0;
  }
}
