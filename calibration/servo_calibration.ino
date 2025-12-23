/*
 * Servo Calibration Tool
 * 
 * Use this sketch to find the neutral positions for each servo.
 * Each servo has slight manufacturing variations, so calibration
 * ensures all legs are properly aligned in the standing position.
 * 
 * Instructions:
 * 1. Upload this sketch to Arduino
 * 2. Open Serial Monitor (115200 baud)
 * 3. Each servo will move to 90 degrees
 * 4. Observe if the leg joint is straight/neutral
 * 5. Adjust the angle until neutral position is found
 * 6. Record the calibrated values
 * 7. Update neutralPos[] array in main sketch
 * 
 * Author: Vinay Nekkanti
 * Date: December 2024
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PWM Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
#define NUM_SERVOS 18
#define SERVO_FREQ 50
#define SERVOMIN 150
#define SERVOMAX 600

// Starting angle for calibration
int testAngle = 90;

void setup() {
  Serial.begin(115200);
  
  Serial.println(F("========================================"));
  Serial.println(F("  Servo Calibration Tool"));
  Serial.println(F("========================================"));
  Serial.println();
  
  // Initialize PWM driver
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  Serial.println(F("PWM Driver initialized"));
  Serial.println();
  Serial.println(F("Starting calibration sequence..."));
  Serial.println(F("Observe each servo and note the angle needed for neutral position."));
  Serial.println();
  
  delay(2000);
  
  // Test each servo individually
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.println(F("----------------------------------------"));
    Serial.print(F("Testing Servo #"));
    Serial.print(i);
    Serial.print(F(" ("));
    printServoName(i);
    Serial.println(F(")"));
    
    // Move to test angle
    setServoAngle(i, testAngle);
    
    Serial.print(F("Current angle: "));
    Serial.print(testAngle);
    Serial.println(F("°"));
    Serial.println(F("Observe the servo position."));
    Serial.println(F("Is it in neutral/straight position?"));
    Serial.println();
    
    delay(3000);  // Wait 3 seconds for observation
  }
  
  Serial.println(F("========================================"));
  Serial.println(F("  Calibration Complete!"));
  Serial.println(F("========================================"));
  Serial.println();
  Serial.println(F("Record your calibrated values:"));
  Serial.println();
  Serial.println(F("int neutralPos[NUM_SERVOS] = {"));
  Serial.println(F("  90, 90, 90,  // Leg 0: Coxa, Femur, Tibia"));
  Serial.println(F("  90, 90, 90,  // Leg 1"));
  Serial.println(F("  90, 90, 90,  // Leg 2"));
  Serial.println(F("  90, 90, 90,  // Leg 3"));
  Serial.println(F("  90, 90, 90,  // Leg 4"));
  Serial.println(F("  90, 90, 90   // Leg 5"));
  Serial.println(F("};"));
  Serial.println();
  Serial.println(F("Adjust values as needed based on observations."));
  Serial.println(F("Save to calibration_values.txt for reference."));
}

void loop() {
  // Interactive calibration mode
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Parse servo number and angle
    // Format: "servo_number angle" (e.g., "5 85")
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex > 0) {
      int servoNum = input.substring(0, spaceIndex).toInt();
      int angle = input.substring(spaceIndex + 1).toInt();
      
      if (servoNum >= 0 && servoNum < NUM_SERVOS && angle >= 0 && angle <= 180) {
        Serial.print(F("Setting Servo #"));
        Serial.print(servoNum);
        Serial.print(F(" to "));
        Serial.print(angle);
        Serial.println(F("°"));
        
        setServoAngle(servoNum, angle);
      } else {
        Serial.println(F("Invalid input. Use: servo_number angle"));
        Serial.println(F("Example: 5 85"));
      }
    }
  }
}

void setServoAngle(int servoNum, int angle) {
  angle = constrain(angle, 0, 180);
  int pulseWidth = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulseWidth);
}

void printServoName(int servoNum) {
  int leg = servoNum / 3;
  int joint = servoNum % 3;
  
  Serial.print(F("Leg "));
  Serial.print(leg);
  Serial.print(F(" - "));
  
  switch (joint) {
    case 0:
      Serial.print(F("Coxa (Hip)"));
      break;
    case 1:
      Serial.print(F("Femur (Thigh)"));
      break;
    case 2:
      Serial.print(F("Tibia (Shin)"));
      break;
  }
}
