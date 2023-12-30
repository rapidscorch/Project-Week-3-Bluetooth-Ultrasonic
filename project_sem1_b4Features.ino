//ctrl k c to comment all, and ctrl / to uncommentn all

#include <Arduino.h>
#include <L298N.h>
#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Motor A connections
int enA = 3;   //  left 1 & 2
int in1 = A1;  //1 & 4
// Motor B connections
int enB = 11;  //  right 3 & 4
int in2 = A2;  //2 & 3

//rotary encoder
int inDisB = 2;
volatile int totalB = 0;       // Declare total as volatile for interrupt use
volatile int previousB = LOW;  // Declare and initialize previous for countdisB

//infared sensor
int irl = A3;
int irr = 13;

void stop(int i) {
  // Stop the motors
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  // digitalWrite(in3, LOW);
  // digitalWrite(in4, LOW);
  delay(i);
}

void foward(int i, int speed) {

  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // digitalWrite(in3, LOW);
  // digitalWrite(in4, HIGH);
  delay(i);
}

void reverse(int i, int speed) {

  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // digitalWrite(in3, HIGH);
  // digitalWrite(in4, LOW);
  delay(i);
}

void L1(int i, int speed) {  // i = delay in between

  analogWrite(enA, 0);  // left motor turn off
  analogWrite(enB, speed);  // right motor go
  digitalWrite(in1, HIGH);  // all motor backwards
  digitalWrite(in2, LOW);
  delay(i);  // in between delay
  analogWrite(enA, speed);  // left motor go
  analogWrite(enB, 0);  // right motor turn off
  digitalWrite(in1, LOW);  // all motor foward
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

void countdisInterrupt() {
  int encoder = digitalRead(inDisB);

  if (encoder != previousB) {
    if (encoder == HIGH) {
      totalB++;
    }
  }

  previousB = encoder;
}

void setup() {

  attachInterrupt(digitalPinToInterrupt(inDisB), countdisInterrupt, CHANGE);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(12, OUTPUT);
  //pinMode(in3, OUTPUT);
  //pinMode(in4, OUTPUT);

  stop(0);

  lcd.begin(16, 2);
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

bool logicRotate = true;  //!= rotation code on the ramp turn off
bool logicOffIR = true;  // != after going down the slope (used to turn of ir sensor when going up/down slope)
bool logicOffstop2s = true;  // != start counting after going down slope (stop 2 seconds for 200)

void loop() {

/*--------------------------initialisations-------------------------------*/

  int left = digitalRead(irl);  // left ir sensor
  int right = digitalRead(irr);  // right ir sensor
  sensors_event_t a, g, temp;  // Get new sensor events with the readings
  // equation to get the slope angle
  float angle = (atan2(a.acceleration.x, a.acceleration.y) * RAD_TO_DEG) - 90;
  // equation to get the rotated angle
  float dt = 0.05;  // time interval in seconds (adjust as needed)
  static float spin = 0.0;
  if (g.gyro.x > 0.2 || g.gyro.x < -0.2) spin += g.gyro.x * dt;

  mpu.getEvent(&a, &g, &temp);

/*--------------------------lcd-------------------------------*/
  
  // time
  unsigned long currentTime = millis();
  static unsigned long lastUpdateTime = 0;
  unsigned long elapsedTime = currentTime - lastUpdateTime;
  lcd.setCursor(0, 0);  // 1st line 1st data
  if (elapsedTime >= 1000) {  // count the time
    unsigned long seconds = currentTime / 1000;
    lcd.print(seconds);
    lcd.print("s");
    lastUpdateTime = currentTime;
  }

  // distance
  // after going down slope, start counting distance to reach 200cm
  float dis = ((float)(totalB) / 20 * 21);
  if (angle < -15) {  
    totalB = 0;
    logicOffstop2s = false;
  }
  dis = ((float)(totalB) / 20 * 21);

  lcd.setCursor(7,0);  // 1st line 2nd data
  lcd.print(dis);
  lcd.print("cm");

  // angle of slope
  lcd.setCursor(0, 1);  // 2nd line 1st data
  static float slope = 0.0;  // slope angle
  if (slope < angle) {  // find max angle of slope
    slope = angle;
  }
  lcd.print((float)slope);
  lcd.print("'");

  // rotated angle "spin"
  lcd.setCursor(7,1);  // 2nd line 2nd data
  lcd.print(spin);
  lcd.print("*");


/*----------------------------after slope-------------------------------*/
  
  // turn left
  if (!logicOffIR && right == 0 && left == 1) {
    // Only the left sensor detect the line
    reverse(40, 200);
    stop(200);
    L1(200, 170);
  }

  // turn right
  if (!logicOffIR && left == 0 && right == 1) {
    // Only the right sensor detect the line
    reverse(40,200);
    stop(200);
    R1(200, 170);
  }

  // go foward
  if (left == 0 && right == 0) {
    if(!logicOffIR) foward(0, 85);  // normal foward
    else if(logicOffIR) foward(0,100);  // go up slope foward
  }

/*----------------------------during slope-------------------------------*/
  
  //go up ramp
  if (logicOffIR && angle >= 20) {  
    foward(210 , 240);
    logicRotate = false;
  }

  //on a ramp //after crossing the slope, angle turn 0, move more to turn 360 degree
  if (!logicRotate && angle <= 1.5) {  
    stop(4000);
    int j =100; // turn delay
    int s = 175;  // speed
    float spin= 0;

    for (int count = 0; count< 57; count++)  // rotate time needed
      {R1(j, s);}  // rotation delay , speed

    foward(1000,80);  // go foward to place the robot into the line

    logicRotate = true;  // close the rotation code on the ramp
    logicOffIR = false;   // close the go up ramp code (when car crash down -> mpu acceleration jump)
  }

  //go down ramp
  if (left == 1 && right == 1) {  // when going down, both sensor is mid air, wont move, this makes it move
    foward(500,85);
  }

  // stop 3s after 200cm
  if (!logicOffstop2s && !logicOffIR && dis >= 200) {
    stop(3000);  // stop (task 2 finnish)
    logicOffstop2s = true;
  }
}

/*-------------------refrence code if use gyroscope in MPU----------------*/

// if (!logicRotate && angle <= 1.5) {  
//     stop(4000);
//     int j =100; // turn delay
//     int s = 175;  // speed
//     float spin= 0;

//     float spin= 0;
//     while(spin <= 5 && spin >= -5){  // spin until 360 degrees
//       R1(j, s);
//       if (g.gyro.x > 0.25) spin += g.gyro.x * 0.25;  // avoid vibrations counted as rotated angle
//     }
//     foward(1000,80);  // go foward to place the robot into the line

//     logicRotate = true;  // close the rotation code on the ramp
//     logicOffIR = false;   // close the go up ramp code (when car crash down -> mpu acceleration jump)
//   }