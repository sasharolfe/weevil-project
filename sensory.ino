// US-100 Ultrasonic Sensor Test
// Reads distance in centimeters using TRIG and ECHO pins

// Pin definitions
const int TRIG_PIN = 14;
const int ECHO_PIN = 12;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.println("US-100 Distance Sensor Starting...");
}

void loop() {

  // Clear trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send 10us pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo time
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in cm
  // Speed of sound = 0.0343 cm/us
  float distance_cm = duration * 0.0343 / 2.0;

  // Print result
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");
  Serial.print("-");

  delay(200);
}