// Motor & Distance sensor test
#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>

// Motor A & Motor B (watch out with RX/TX pins! - do not override them)
const int pwmA = 9, in1A = 8, in2A = 7;
const int pwmB = 6, in1B = 5, in2B = 4;

const int freq = 20000;          // 20kHz
const int res = 8;               // 0..255

// Motor Speed Values - const
const uint8_t MotorSpeed1 = 128;
const uint8_t MotorSpeed2 = 128;
 
// Servo pinout
const int SERVO_PIN = 1;   //this pin probably doesn't work!/ doesn't work with PWM
Servo servo1;

// Distance sensor pinout
const int ECHO = 2;
const int TRIG = 3;
 
// basic setup
void setup() 
{
  pinMode(in1A, OUTPUT), pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT), pinMode(in2B, OUTPUT);

  ledcSetup(0, freq, res); // Channel 0 for Motor A
  ledcSetup(1, freq, res); // Channel 1 for Motor B
  ledcAttachPin(pwmA, 0);  // Attach Motor A PWM pin to channel 0
  ledcAttachPin(pwmB, 1);  // Attach Motor B PWM pin to channel 1

  // Motor A forward
  digitalWrite(in1A, LOW);  
  digitalWrite(in2A, HIGH);
 
 // Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  
  ledcWrite(0, MotorSpeed1); // Write speed to channel 0 (Motor A)
  ledcWrite(1, MotorSpeed2); // Write speed to channel 1 (Motor B)
  ledcWrite(pwmA, MotorSpeed1);
  ledcWrite(pwmB, MotorSpeed2);
  
  // Distance sensor data
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Servo initialization
  //servo1.attach(SERVO_PIN);
}

int measure_distance() {
  long duration, distance;

  // trigger mechanism
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);
  distance = duration / 58;

  return distance;
}


void loop() { 
  Serial.print(measure_distance());
  Serial.println(" cm");
  delay(500);

  // for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
  //       servo1.write(posDegrees);
  //       Serial.println(posDegrees);
  //       delay(20);
  //   }

  //   for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
  //       servo1.write(posDegrees);
  //       Serial.println(posDegrees);
  //       delay(20);
  //   }
  
}