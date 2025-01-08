// Include the ESP32Servo library
#include <ESP32Servo.h>

// Define pins for Distance Sensor
#define TRIG_PIN 19 // Define the TRIG pin
#define ECHO_PIN 21 // Define the ECHO pin
#define IN1 5       // Define the motor IN1 pin
#define IN2 18      // Define the motor IN2 pin
#define IN3 23      // Define the motor IN3 pin
#define IN4 25      // Define the motor IN4 pin

// Define pins for LDR and Buzzer
#define LDR_PIN 34  // Analog pin (ADC1) connected to the LDR
#define BUZZER_PIN 22 // Digital pin connected to the buzzer

// Define pin for Servo Motor
#define SERVO_PIN 13 // GPIO pin connected to the servo motor

unsigned long distanceAbove12StartTime = 0; // Track time when distance > 12cm
unsigned long lastServoActionTime = 0;      // Track time for servo action
unsigned long lastMotorActionTime = 0;      // Track time for motor action
bool isDistanceBuzzerOn = false;            // Track if the buzzer is ON for distance logic
bool isLightSensorBuzzerOn = false;         // Track if the buzzer is ON for light sensor logic
int ldrCount = 0;                           // Counter for LDR low light condition

Servo servo;                                // Create a servo object

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Configure pins for Distance Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configure pins for LDR and Buzzer
  pinMode(LDR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Attach servo to pin SERVO_PIN
  servo.attach(SERVO_PIN); // Attach servo to GPIO 13
  servo.write(0); // Initialize the servo at 0 degrees

  // Ensure the motor and buzzer are initially off
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
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
  if (millis() - lastServoActionTime >= 60000) { // Check if 1 minute has passed
    Serial.println("Servo Action: Turning to 180 degrees");
    servo.write(180); // Move servo to 180 degrees
    delay(2000);      // Wait for 2 seconds
    Serial.println("Servo Action: Returning to 0 degrees");
    servo.write(0);   // Return servo to 0 degrees
    lastServoActionTime = millis(); // Reset the timer
  }

  // Motor Driver Logic for IN3 and IN4
  if (millis() - lastMotorActionTime >= 120000) { // Check if 2 minutes have passed
    Serial.println("Motor IN3-IN4 Action: Turning ON for 5 seconds");
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW); // Turn ON motor connected to IN3 and IN4
    delay(5000);            // Keep motor ON for 5 seconds
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW); // Turn OFF motor connected to IN3 and IN4
    Serial.println("Motor IN3-IN4 Action: Turning OFF");
    lastMotorActionTime = millis(); // Reset the timer
  }

  // Small delay to avoid overwhelming the loop
  delay(1000);
}
