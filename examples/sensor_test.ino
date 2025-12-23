/*
 * Ultrasonic Sensor Test
 * 
 * Simple test to verify HC-SR04 ultrasonic sensor is working correctly.
 * Displays distance measurements on Serial Monitor.
 * 
 * Author: Vinay Nekkanti
 * Date: December 2024
 */

#define TRIG_PIN 2
#define ECHO_PIN 3

void setup() {
  Serial.begin(115200);
  Serial.println("HC-SR04 Ultrasonic Sensor Test");
  Serial.println("================================");
  Serial.println();
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("Sensor initialized. Reading distance...");
  Serial.println();
}

void loop() {
  float distance = getDistance();
  
  Serial.print("Distance: ");
  
  if (distance > 0 && distance < 400) {
    Serial.print(distance, 1);
    Serial.println(" cm");
    
    // Visual bar graph
    int bars = map(distance, 0, 100, 0, 50);
    Serial.print("[");
    for (int i = 0; i < bars; i++) {
      Serial.print("=");
    }
    Serial.println("]");
  } else {
    Serial.println("Out of range");
  }
  
  Serial.println();
  delay(500);
}

float getDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  // Calculate distance
  float distance = duration * 0.0343 / 2;
  
  return distance;
}
