#include <Arduino.h>
#include <L298N.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// Motor A connections
int enA = 3;   //  left 1 & 2
int in1 = A1;  //1 & 4
int in2 = A2;  //2 & 3

// Motor B connections
int enB = 11;  //  right 3 & 4

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

char command; 

void setup() {       
  Serial.begin(9600);  //Set the baud rate to your Bluetooth module.
  lcd.begin(16, 2);
}

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

void loop(){

  if(Serial.available() > 0){ 
    command = Serial.read(); 
    stop(0); //initialize with motors stoped
    // Change pin mode only if new command is different from previous.   
    // Serial.println(command);
    lcd.setCursor(0, 0);
    lcd.print("Movement :");

    switch(command){
    case 'F':  
      foward(100,250);
      lcd.println("Forward");
      break;
    case 'B':  
      reverse(100,250);
      lcd.println("Backward");
      break;
    case 'L':  
      L1(100,250);
      lcd.println("Left");
      break;
    case 'R':
      R1(100,250);
      lcd.println("Right");
      break;
    }
  } 
}

