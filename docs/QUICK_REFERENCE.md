# Hexapod Quick Reference

## Pin Connections

```
Arduino Uno → PCA9685 PWM Driver
├─ A4 (SDA) → SDA
├─ A5 (SCL) → SCL  
├─ 5V → VCC
└─ GND → GND

Arduino Uno → HC-SR04 Ultrasonic
├─ D2 → Trigger
├─ D3 → Echo
├─ 5V → VCC
└─ GND → GND

PCA9685 → Servos
└─ Channels 0-17 → 18 servos
   (Power from battery, not Arduino!)
```

## Servo Mapping

| Leg | Position | Coxa | Femur | Tibia |
|-----|----------|------|-------|-------|
| 0 | Right Front | 0 | 1 | 2 |
| 1 | Left Front | 3 | 4 | 5 |
| 2 | Right Middle | 6 | 7 | 8 |
| 3 | Left Middle | 9 | 10 | 11 |
| 4 | Right Rear | 12 | 13 | 14 |
| 5 | Left Rear | 15 | 16 | 17 |

## Key Functions

```cpp
// Set servo angle (0-180°)
setServoAngle(servoNum, angle);

// Get distance from ultrasonic sensor
float distance = getDistance();

// Execute one walking cycle
tripodGait();

// Turn in place
turnRight();
turnLeft();
```

## Configuration Constants

```cpp
#define SERVO_FREQ 50          // PWM frequency (Hz)
#define OBSTACLE_DISTANCE 20   // Avoidance threshold (cm)
#define GAIT_STEP_DELAY 100   // Gait phase delay (ms)
```

## Calibration Process

1. Upload `calibration/servo_calibration.ino`
2. Open Serial Monitor (115200 baud)
3. Observe each servo position at 90°
4. Record needed adjustments
5. Update `neutralPos[]` array in main code

## Common Adjustments

**Walking too fast/slow:**
```cpp
#define GAIT_STEP_DELAY 100  // Increase = slower
```

**Turns too sharp/gentle:**
```cpp
int turnAngle = 30;  // In turnRight()/turnLeft()
```

**Obstacle detection range:**
```cpp
#define OBSTACLE_DISTANCE 20  // Change this value
```

**Step size:**
```cpp
int moveAngle = 20;  // In moveLegsForward()
```

## Serial Monitor Commands

**Baud Rate:** 115200

**Expected Output:**
```
Hexapod Robot Controller v1.0
Initializing...
✓ PWM Driver initialized
✓ Ultrasonic sensor configured
✓ All servos initialized
Robot Ready!
Walking forward...
⚠ Obstacle at 15.2 cm - Turning right
```

## Troubleshooting Quick Fixes

| Problem | Quick Fix |
|---------|-----------|
| Servos don't move | Check battery voltage (>7V) |
| Servo jitter | Add 1000µF capacitor across power |
| No obstacle detection | Verify D2/D3 connections |
| I2C errors | Check SDA→A4, SCL→A5 |
| Upload fails | Close Serial Monitor first |

## Power Requirements

- **Battery:** 7.4V 2S LiPo, 2200mAh minimum
- **Current:** 3-5A typical, 10A peak
- **Runtime:** 30-45 minutes
- **Low voltage:** <6.8V (stop using!)

## Important Safety

⚠️ **Never discharge LiPo below 3.0V per cell**  
⚠️ **Always have emergency stop ready**  
⚠️ **Monitor servo temperatures**  
⚠️ **Use proper LiPo storage voltage (3.8V/cell)**
