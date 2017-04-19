#include <Wire.h>
#include <VL6180X.h>

// Laser Range sensors
VL6180X sensor;

// Motor
#define pinI1     8  //define IN1 interface
#define pinI2     11 //define IN2 interface 
#define speedpinA 9  //enable motor A
#define pinI3     12 //define IN3 interface 
#define pinI4     13 //define IN4 interface 
#define speedpinB 10 //enable motor B
#define _speed1   250 // define the speed of motor A
#define _speed2   233 // define the speed of motor B

// Ultrasonic sensor
#define trigger 4
#define echo 5

const int distanceThreshold = 100;
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

int getLaserRange() {
  return sensor.readRangeSingleMillimeters();
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
  Wire.begin();
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);

  // Start car
  forward();
}

void loop() {
  // Ultrasonic sensor
  unsigned int ultrasonicData = abs(getUltrasonic());
  Serial.println(ultrasonicData);
  if (ultrasonicData <= distanceThreshold && !stopped) {
      Serial.println("Distance <= 50, stopping & backing up");
      stop();
      stopped = true;
      backward();
  } else if (ultrasonicData > distanceThreshold && stopped) {
    if (stopped) {
      stopped = false;
    }
    Serial.println("Distance >= 50, stopping & going forward");
    stop();
    forward();
  }
  
  // Serial.println("Ultrasonic: " + String(getUltrasonic()));


  // Laser Range sensor
  //Serial.println("Laser Range: " + String(getLaserRange()));

  delay(100);
}
