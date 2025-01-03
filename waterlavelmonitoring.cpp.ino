#define TRIG_PIN 19 // Define the TRIG pin
#define ECHO_PIN 21 // Define the ECHO pin
#define IN1 5       // Define the motor IN1 pin
#define IN2 18      // Define the motor IN2 pin

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Configure pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT); // Motor control pin
  pinMode(IN2, OUTPUT); // Motor control pin

  // Ensure the motor is initially off
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void loop() {
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
  // Speed of sound = 343 m/s = 0.0343 cm/us
  distance = (duration * 0.0343) / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Check the distance and control the motor
  if (distance > 12) {
    // If distance is greater than 12 cm, turn the motor ON
    digitalWrite(IN1, HIGH); // Set IN1 to HIGH
    digitalWrite(IN2, LOW);  // Set IN2 to LOW (Forward direction)
    Serial.println("Motor ON");
  } else {
    // If distance is less than or equal to 12 cm, turn the motor OFF
    digitalWrite(IN1, LOW); // Set IN1 to LOW
    digitalWrite(IN2, LOW); // Set IN2 to LOW (Motor OFF)
    Serial.println("Motor OFF");
  }

  // Wait for a short moment before the next measurement
  delay(500);
}

