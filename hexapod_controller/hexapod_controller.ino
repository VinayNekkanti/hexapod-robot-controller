/*
 * Hexapod Robot Controller
 * 
 * 18-DOF Hexapod Robot with Autonomous Obstacle Avoidance
 * 
 * Hardware:
 * - Arduino Uno (ATmega328P)
 * - PCA9685 16-Channel PWM Servo Driver
 * - 18x MG90S Micro Servos
 * - HC-SR04 Ultrasonic Distance Sensor
 * - 7.4V 2S LiPo Battery
 * 
 * Author: Vinay Nekkanti
 * Date: December 2025
 * GitHub: https://github.com/VinayNekkanti/hexapod-robot-controller
 * 
 * Pin Configuration:
 * - A4 (SDA): I2C Data to PCA9685
 * - A5 (SCL): I2C Clock to PCA9685
 * - D2: Ultrasonic Trigger
 * - D3: Ultrasonic Echo
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== HARDWARE CONFIGURATION ====================

// PCA9685 PWM Driver Configuration
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration constants
#define NUM_SERVOS 18           // Total number of servos (6 legs × 3 joints)
#define SERVO_FREQ 50           // PWM frequency in Hz (standard for servos)
#define SERVOMIN 150            // Minimum pulse length count (out of 4096)
#define SERVOMAX 600            // Maximum pulse length count (out of 4096)

// Ultrasonic sensor pins
#define TRIG_PIN 2              // Trigger pin for HC-SR04
#define ECHO_PIN 3              // Echo pin for HC-SR04
#define OBSTACLE_DISTANCE 20    // Distance threshold in cm to trigger avoidance

// Timing constants (milliseconds)
#define GAIT_STEP_DELAY 100     // Delay between gait phases
#define TURN_DURATION 500       // Duration of turn maneuver
#define SENSOR_POLL_INTERVAL 50 // How often to check distance sensor

// ==================== SERVO MAPPING ====================
/*
 * Servo Layout:
 * Leg numbering (top view):
 *     0 ---- 1
 *     |      |
 *     2 ---- 3
 *     |      |
 *     4 ---- 5
 * 
 * Right side: 0 (front), 2 (middle), 4 (rear)
 * Left side:  1 (front), 3 (middle), 5 (rear)
 * 
 * Each leg has 3 servos:
 * - Coxa (hip): rotates leg horizontally
 * - Femur (thigh): lifts leg up/down
 * - Tibia (shin): extends/retracts foot
 */

// Servo channel assignments on PCA9685
enum ServoChannels {
  // Right Front Leg (Leg 0)
  LEG0_COXA = 0,
  LEG0_FEMUR = 1,
  LEG0_TIBIA = 2,
  
  // Left Front Leg (Leg 1)
  LEG1_COXA = 3,
  LEG1_FEMUR = 4,
  LEG1_TIBIA = 5,
  
  // Right Middle Leg (Leg 2)
  LEG2_COXA = 6,
  LEG2_FEMUR = 7,
  LEG2_TIBIA = 8,
  
  // Left Middle Leg (Leg 3)
  LEG3_COXA = 9,
  LEG3_FEMUR = 10,
  LEG3_TIBIA = 11,
  
  // Right Rear Leg (Leg 4)
  LEG4_COXA = 12,
  LEG4_FEMUR = 13,
  LEG4_TIBIA = 14,
  
  // Left Rear Leg (Leg 5)
  LEG5_COXA = 15,
  LEG5_FEMUR = 16,
  LEG5_TIBIA = 17
};

// ==================== GLOBAL VARIABLES ====================

// Neutral positions for all servos (adjust during calibration)
// These are the angles when the robot is in standing position
int neutralPos[NUM_SERVOS] = {
  90, 90, 90,  // Leg 0: Coxa, Femur, Tibia
  90, 90, 90,  // Leg 1
  90, 90, 90,  // Leg 2
  90, 90, 90,  // Leg 3
  90, 90, 90,  // Leg 4
  90, 90, 90   // Leg 5
};

// Current positions of all servos
int currentPos[NUM_SERVOS];

// Timing variables
unsigned long lastSensorRead = 0;

// ==================== SETUP ====================

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println(F("========================================"));
  Serial.println(F("  Hexapod Robot Controller v1.0"));
  Serial.println(F("  18-DOF with Obstacle Avoidance"));
  Serial.println(F("========================================"));
  Serial.println();
  Serial.println(F("Initializing..."));
  
  // Initialize I2C bus
  Wire.begin();
  
  // Initialize PCA9685 PWM driver
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Set to 50Hz for servos
  delay(10);  // Allow oscillator to stabilize
  
  Serial.println(F("✓ PWM Driver initialized (50Hz)"));
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println(F("✓ Ultrasonic sensor configured"));
  
  // Set all servos to neutral position
  Serial.println(F("Setting servos to neutral position..."));
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, neutralPos[i]);
    currentPos[i] = neutralPos[i];
    delay(20);  // Small delay to avoid current surge
  }
  
  Serial.println(F("✓ All servos initialized"));
  
  // Wait for servos to reach position
  delay(1000);
  
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F("  Robot Ready!"));
  Serial.println(F("  Starting autonomous navigation..."));
  Serial.println(F("========================================"));
  Serial.println();
}

// ==================== MAIN LOOP ====================

void loop() {
  unsigned long currentTime = millis();
  
  // Read distance sensor periodically
  if (currentTime - lastSensorRead >= SENSOR_POLL_INTERVAL) {
    float distance = getDistance();
    lastSensorRead = currentTime;
    
    // Decision logic based on obstacle proximity
    if (distance > OBSTACLE_DISTANCE) {
      // Path is clear - continue walking forward
      Serial.println(F("Walking forward..."));
      tripodGait();
      
    } else if (distance > 0) {
      // Obstacle detected - perform avoidance maneuver
      Serial.print(F("⚠ Obstacle detected at "));
      Serial.print(distance, 1);
      Serial.println(F(" cm - Turning right"));
      
      turnRight();
      delay(TURN_DURATION);
      
      // Return to neutral after turn
      returnToNeutral();
    }
  }
}

// ==================== SERVO CONTROL FUNCTIONS ====================

/**
 * Set servo to specific angle (0-180 degrees)
 * Converts angle to PWM pulse width and sends via I2C
 * 
 * @param servoNum: Servo channel (0-17)
 * @param angle: Desired angle in degrees (0-180)
 */
void setServoAngle(int servoNum, int angle) {
  // Constrain angle to valid range
  angle = constrain(angle, 0, 180);
  
  // Map angle (0-180°) to pulse width (SERVOMIN-SERVOMAX)
  // PCA9685 uses 12-bit resolution (0-4095)
  int pulseWidth = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  
  // Send PWM signal via I2C
  pwm.setPWM(servoNum, 0, pulseWidth);
  
  // Update position tracking
  currentPos[servoNum] = angle;
}

/**
 * Smoothly move servo from current position to target angle
 * Uses linear interpolation for smooth motion
 * 
 * @param servoNum: Servo channel
 * @param targetAngle: Desired final angle
 * @param stepDelay: Delay between each step (ms)
 */
void smoothMove(int servoNum, int targetAngle, int stepDelay) {
  int current = currentPos[servoNum];
  int steps = abs(targetAngle - current);
  int direction = (targetAngle > current) ? 1 : -1;
  
  for (int i = 0; i < steps; i++) {
    current += direction;
    setServoAngle(servoNum, current);
    delay(stepDelay);
  }
}

/**
 * Return all servos to neutral standing position
 */
void returnToNeutral() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, neutralPos[i]);
  }
  delay(200);
}

// ==================== GAIT PATTERNS ====================

/**
 * Tripod Gait Pattern
 * 
 * Most stable gait for hexapod robots. Three legs move while three 
 * stay planted, forming two alternating triangular support patterns.
 * 
 * Group 1: Legs 0, 2, 4 (right front, right rear, left middle)
 * Group 2: Legs 1, 3, 5 (left front, left rear, right middle)
 * 
 * Sequence:
 * 1. Lift Group 1
 * 2. Move Group 1 forward (while Group 2 supports)
 * 3. Lower Group 1
 * 4. Lift Group 2
 * 5. Move Group 2 forward (while Group 1 supports)
 * 6. Lower Group 2
 * 7. Repeat
 */
void tripodGait() {
  // Phase 1: Lift first tripod (legs 0, 2, 4)
  liftLegs(0, 2, 4);
  delay(GAIT_STEP_DELAY);
  
  // Phase 2: Move lifted legs forward
  moveLegsForward(0, 2, 4);
  delay(GAIT_STEP_DELAY);
  
  // Phase 3: Lower first tripod
  lowerLegs(0, 2, 4);
  delay(GAIT_STEP_DELAY);
  
  // Phase 4: Lift second tripod (legs 1, 3, 5)
  liftLegs(1, 3, 5);
  delay(GAIT_STEP_DELAY);
  
  // Phase 5: Move lifted legs forward
  moveLegsForward(1, 3, 5);
  delay(GAIT_STEP_DELAY);
  
  // Phase 6: Lower second tripod
  lowerLegs(1, 3, 5);
  delay(GAIT_STEP_DELAY);
}

/**
 * Lift specified legs by raising tibia servos
 * 
 * @param leg1, leg2, leg3: Leg numbers (0-5)
 */
void liftLegs(int leg1, int leg2, int leg3) {
  int liftAngle = 45;  // Degrees to lift (adjust as needed)
  
  // Tibia is the 3rd servo of each leg (index: leg*3 + 2)
  setServoAngle(leg1 * 3 + 2, liftAngle);
  setServoAngle(leg2 * 3 + 2, liftAngle);
  setServoAngle(leg3 * 3 + 2, liftAngle);
}

/**
 * Lower specified legs by returning tibia servos to neutral
 * 
 * @param leg1, leg2, leg3: Leg numbers (0-5)
 */
void lowerLegs(int leg1, int leg2, int leg3) {
  // Return to neutral position
  setServoAngle(leg1 * 3 + 2, neutralPos[leg1 * 3 + 2]);
  setServoAngle(leg2 * 3 + 2, neutralPos[leg2 * 3 + 2]);
  setServoAngle(leg3 * 3 + 2, neutralPos[leg3 * 3 + 2]);
}

/**
 * Move specified legs forward by rotating coxa servos
 * 
 * @param leg1, leg2, leg3: Leg numbers (0-5)
 */
void moveLegsForward(int leg1, int leg2, int leg3) {
  int moveAngle = 20;  // Degrees to move forward (adjust for step size)
  
  // Coxa is the 1st servo of each leg (index: leg*3)
  // Add angle for right-side legs, subtract for left-side legs
  setServoAngle(leg1 * 3, neutralPos[leg1 * 3] + (leg1 % 2 == 0 ? moveAngle : -moveAngle));
  setServoAngle(leg2 * 3, neutralPos[leg2 * 3] + (leg2 % 2 == 0 ? moveAngle : -moveAngle));
  setServoAngle(leg3 * 3, neutralPos[leg3 * 3] + (leg3 % 2 == 0 ? moveAngle : -moveAngle));
}

// ==================== TURNING MANEUVERS ====================

/**
 * Turn robot right by rotating all coxa servos
 * Creates a pivot turn in place
 */
void turnRight() {
  int turnAngle = 30;  // Degrees to turn (adjust for turn radius)
  
  // Rotate all coxa servos
  for (int leg = 0; leg < 6; leg++) {
    int coxaServo = leg * 3;
    setServoAngle(coxaServo, neutralPos[coxaServo] + turnAngle);
  }
  
  delay(300);
}

/**
 * Turn robot left by rotating all coxa servos
 */
void turnLeft() {
  int turnAngle = 30;
  
  for (int leg = 0; leg < 6; leg++) {
    int coxaServo = leg * 3;
    setServoAngle(coxaServo, neutralPos[coxaServo] - turnAngle);
  }
  
  delay(300);
}

// ==================== ULTRASONIC SENSOR ====================

/**
 * Measure distance using HC-SR04 ultrasonic sensor
 * 
 * Working principle:
 * 1. Send 10µs trigger pulse
 * 2. Sensor emits 8 ultrasonic pulses at 40kHz
 * 3. Measure echo pulse duration
 * 4. Calculate distance: distance = (duration × speed_of_sound) / 2
 * 
 * @return Distance in centimeters (0 if no echo or timeout)
 */
float getDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse duration (timeout after 30ms = ~5m max range)
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Calculate distance
  // Speed of sound = 343 m/s = 0.0343 cm/µs
  // Distance = (duration × 0.0343) / 2  (divide by 2 for round trip)
  float distance = duration * 0.0343 / 2;
  
  // Return 0 if no echo received (timeout)
  if (duration == 0) {
    return 0;
  }
  
  return distance;
}

// ==================== END OF CODE ====================
