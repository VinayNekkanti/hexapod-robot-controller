/*
 * Basic Walking Example
 * 
 * Simple demonstration of tripod gait pattern without sensors.
 * Use this to test basic walking functionality.
 * 
 * Author: Vinay Nekkanti
 * Date: December 2024
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define NUM_SERVOS 18
#define SERVO_FREQ 50
#define SERVOMIN 150
#define SERVOMAX 600

int neutralPos[NUM_SERVOS] = {
  90, 90, 90,  // Leg 0
  90, 90, 90,  // Leg 1
  90, 90, 90,  // Leg 2
  90, 90, 90,  // Leg 3
  90, 90, 90,  // Leg 4
  90, 90, 90   // Leg 5
};

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Walking Test");
  
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  // Initialize to neutral
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, neutralPos[i]);
  }
  
  delay(1000);
  Serial.println("Starting walk cycle...");
}

void loop() {
  tripodGait();
}

void setServoAngle(int servoNum, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulse);
}

void tripodGait() {
  // Lift legs 0, 2, 4
  setServoAngle(2, 45);
  setServoAngle(8, 45);
  setServoAngle(14, 45);
  delay(100);
  
  // Move forward
  setServoAngle(0, neutralPos[0] + 20);
  setServoAngle(6, neutralPos[6] + 20);
  setServoAngle(12, neutralPos[12] + 20);
  delay(100);
  
  // Lower legs 0, 2, 4
  setServoAngle(2, neutralPos[2]);
  setServoAngle(8, neutralPos[8]);
  setServoAngle(14, neutralPos[14]);
  delay(100);
  
  // Lift legs 1, 3, 5
  setServoAngle(5, 45);
  setServoAngle(11, 45);
  setServoAngle(17, 45);
  delay(100);
  
  // Move forward
  setServoAngle(3, neutralPos[3] - 20);
  setServoAngle(9, neutralPos[9] - 20);
  setServoAngle(15, neutralPos[15] - 20);
  delay(100);
  
  // Lower legs 1, 3, 5
  setServoAngle(5, neutralPos[5]);
  setServoAngle(11, neutralPos[11]);
  setServoAngle(17, neutralPos[17]);
  delay(100);
}
