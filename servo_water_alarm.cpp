#include <ESP32Servo.h>

// Define pins for Distance Sensor
#define TRIG_PIN 19 // Define the TRIG pin
#define ECHO_PIN 21 // Define the ECHO pin
#define IN1 5       // Define the motor IN1 pin
#define IN2 18      // Define the motor IN2 pin

// Define pins for LDR, Buzzer, and Servo
#define LDR_PIN 34   // Analog pin (ADC1) connected to the LDR
#define BUZZER_PIN 22 // Digital pin connected to the buzzer
#define SERVO_PIN 25 // PWM pin for the servo motor

Servo myServo; // Create servo object

unsigned long distanceAbove12StartTime = 0; // Track time when distance > 12cm
bool isDistanceBuzzerOn = false;            // Track if the buzzer is ON for distance logic
bool isLightSensorBuzzerOn = false;         // Track if the buzzer is ON for light sensor logic
int ldrCount = 0;                           // Counter for LDR low light condition
unsigned long servoTimer = 0;               // Timer for servo control
const unsigned long servoInterval = 60000;  // 1 minute interval

// Servo control variables
unsigned long servoMoveStartTime = 0;
bool isServoMovingTo90 = false;
bool isServoMovingTo0 = false;

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Configure pins for Distance Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Configure pins for LDR, Buzzer, and Servo
  pinMode(LDR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize the servo
  myServo.attach(SERVO_PIN, 500, 2400); // Attach servo to the pin with min and max pulse width
  myServo.write(0); // Set the servo to initial position

  // Ensure the motor, buzzer, and servo power are initially off
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(BUZZER_PIN, HIGH); // Buzzer OFF

  Serial.println("System Initialized");
}

void loop() {
  // Distance Sensor Logic
  long duration;  // Variable to store duration of the pulse
  float distance; // Variable to store calculated distance

  // Ensure TRIG is LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond HIGH pulse to TRIG
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the pulse on the ECHO pin
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (in cm)
  distance = (duration * 0.0343) / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance > 12) {
    // Turn the motor ON if distance > 12 cm
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    Serial.println("Motor ON");

    // Start tracking time for buzzer
    if (distanceAbove12StartTime == 0) {
      distanceAbove12StartTime = millis();
    }

    // Check if the distance has been above 12 cm for more than 10 seconds
    if (millis() - distanceAbove12StartTime >= 10000 && !isDistanceBuzzerOn) {
      digitalWrite(BUZZER_PIN, LOW); // Turn the buzzer ON
      isDistanceBuzzerOn = true;
      Serial.println("Buzzer ON: Distance > 12 cm for 10 seconds");
    }
  } else {
    // Turn the motor OFF
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    Serial.println("Motor OFF");

    // Reset distance tracking if the distance is less than or equal to 12 cm
    distanceAbove12StartTime = 0;
  }

  // LDR Light Sensor Logic
  int ldrValue = analogRead(LDR_PIN);
  float ldrPercentage = (ldrValue / 4095.0) * 100;

  Serial.print("LDR Value: ");
  Serial.print(ldrValue);
  Serial.print(" | Light Intensity: ");
  Serial.print(ldrPercentage);
  Serial.println("%");

  if (ldrValue == 0) {
    ldrCount++;
    if (ldrCount >= 10 && !isLightSensorBuzzerOn) {
      digitalWrite(BUZZER_PIN, LOW); // Turn the buzzer ON
      isLightSensorBuzzerOn = true;
      Serial.println("Buzzer ON: LDR Value is 0 for 10 seconds");
    }
  } else {
    ldrCount = 0;
  }

  // Turn off the buzzer only when both conditions are satisfied
  if (distance <= 12 && ldrValue > 0) {
    if (isDistanceBuzzerOn || isLightSensorBuzzerOn) {
      digitalWrite(BUZZER_PIN, HIGH); // Turn the buzzer OFF
      isDistanceBuzzerOn = false;
      isLightSensorBuzzerOn = false;
      Serial.println("Buzzer OFF: Distance <= 12 cm and Light Intensity > 0");
    }
  }

  // Servo Motor Logic
  if (millis() - servoTimer >= servoInterval) {
    if (!isServoMovingTo90 && !isServoMovingTo0) {
      servoTimer = millis();
      isServoMovingTo90 = true;
      servoMoveStartTime = millis();
      Serial.println("Servo Moving to 90 degrees");
      myServo.write(90); // Rotate servo to 90 degrees
    }
  }

  if (isServoMovingTo90 && millis() - servoMoveStartTime >= 5000) {
    isServoMovingTo90 = false;
    isServoMovingTo0 = true;
    servoMoveStartTime = millis();
    Serial.println("Servo Returning to 0 degrees");
    myServo.write(0); // Return to 0 degrees
  }

  if (isServoMovingTo0 && millis() - servoMoveStartTime >= 5000) {
    isServoMovingTo0 = false;
  }

  // Small delay to avoid overwhelming the loop
  delay(1000);
}
