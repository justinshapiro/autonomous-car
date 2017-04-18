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
#define spead     200 //define the spead of motor

// Ultrasonic sensor
#define trigger 1
#define echo 2

// Motor control routines
void forward() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,spead);
  analogWrite(speedpinB,spead);

  //turn DC Motor B move clockwise
  digitalWrite(pinI4,HIGH);
  digitalWrite(pinI3,LOW);

  //turn DC Motor A move anticlockwise
  digitalWrite(pinI2,LOW);
  digitalWrite(pinI1,HIGH);
}
 
void backward() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,spead);
  analogWrite(speedpinB,spead);
  
  //turn DC Motor B move anticlockwise
  digitalWrite(pinI4,LOW);
  digitalWrite(pinI3,HIGH);
  
  //turn DC Motor A move clockwise
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI1,LOW);
}
 
void left() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,spead);
  analogWrite(speedpinB,spead);
  
  //turn DC Motor B move clockwise
  digitalWrite(pinI4,HIGH);
  digitalWrite(pinI3,LOW);

  //turn DC Motor A move clockwise
  digitalWrite(pinI2,HIGH);
  digitalWrite(pinI1,LOW);
}
 
void right() {
  //input a simulation value to set the speed
  analogWrite(speedpinA,spead);
  analogWrite(speedpinB,spead);

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
}

short getUltrasonic() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  short duration = pulseIn(echo, HIGH);
  return (5 * duration) / 29.1;
}

void loop() {
  // Ultrasonic sensor
  Serial.println(getUltrasonic());

  // Motor
  forward(); // options: left(), right(), backward()

  // Laser Range sensor
  Serial.print(sensor.readRangeSingleMillimeters());
}
