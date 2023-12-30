#include <Arduino.h>
#include <L298N.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <NewPing.h>
#define trig_Pin 2 
#define echo_Pin 12 

NewPing sonar (trig_Pin, echo_Pin);

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Motor A connections
int enA = 3;   //  left 1 & 2
int in1 = A1;  //1 & 4
int in2 = A2;  //2 & 3
// Motor B connections
int enB = 11;  //  right 3 & 4

const int obstacleThreshold = 20; // Distance threshold for obstacle detection in centimeters


void stop(int i) {
  // Stop the motors
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(i);
}

void foward(int i, int speed) {

  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(i);
}

void reverse(int i, int speed) {

  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(i);
}

void L(int i, int speed) {
  //LEFT
  analogWrite(enA, 0);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(i);
}

void L1(int i, int speed) {

  analogWrite(enA, 0);
  analogWrite(enB, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(i);
  analogWrite(enA, speed);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(i);
}

void R(int i, int speed) {
  //RIGHT
  analogWrite(enB, 0);
  analogWrite(enA, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(i);
}

void R1(int i, int speed) {

  analogWrite(enA, 0);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(i);
  analogWrite(enB, 0);
  analogWrite(enA, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(i);
}

void rotate(int i, int speed) {

  analogWrite(enA, 0);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(i);
  analogWrite(enB, 0);
  analogWrite(enA, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(i);
}

void setup(){

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  stop(0);

  lcd.begin(16, 2);
  Serial.begin(115200);

  pinMode(trig_Pin, OUTPUT);
  pinMode(echo_Pin, INPUT);


}

void loop() {
  // Measure distance using ultrasonic sensor
  long duration, distance;

  digitalWrite(trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_Pin, LOW);
  duration = pulseIn(echo_Pin, HIGH);
  distance = duration * 0.034 / 2;

  // Print distance for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check for obstacles and move the car accordingly
  if (distance > obstacleThreshold) {
    // No obstacle detected, move forward
    foward(100,100);
  } else {
    // Obstacle detected, stop and turn
    stop(100);
    delay(500); // Stop for a short duration
    reverse(50,230);
    //L (100,250); // You can modify this to turn left or perform a different maneuver
    L1 (170,200);
  }
  
   // Print distance for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  lcd.setCursor(0,0);
  lcd.print("d = ");
  lcd.print(distance);
  lcd.println(" cm");

}