#include <ESP32Servo.h>
// ============================================================
//  SENSOR PINS
// ============================================================
static const int FRONT_TRIG = 26; // A0
static const int FRONT_ECHO = 34; // A2

static const int LEFT_TRIG = 25; // A1
static const int LEFT_ECHO = 39; // A3

static const int RIGHT_TRIG = 4; // A5
static const int RIGHT_ECHO = 36; // A4

// ============================================================
//  SERVO PINS
// ============================================================
static const int LEFT_OUTER_PIN = 15;
static const int LEFT_INNER_PIN = 13;
static const int RIGHT_OUTER_PIN = 27;
static const int RIGHT_INNER_PIN = 33;
static const int LEFT_OUTER_V_PIN = 21;
static const int LEFT_INNER_V_PIN = 12;
static const int RIGHT_OUTER_V_PIN = 32;
static const int RIGHT_INNER_V_PIN = 14;

// ============================================================
//  LOCOMOTION CONSTANTS
// ============================================================
static const float linear_displacement = 3.175; // cm per step cycle
static const float degrees_per_cycle = 6.75; // degrees per turn cycle
static const int leg_lift_height = 20; // servo offset for leg lift (deg from center)
static const int foot_rotation = 40; // servo sweep angle (deg from center)
static const int motor_wait = 150; // ms per servo phase

static const int left_off_displacement = 0;
static const int right_off_displacement = 0;

// ============================================================
//  SENSOR CONSTANTS
// ============================================================
static const int SENSOR_SAMPLES = 5; // samples taken for one reading (get median)
static const int SENSOR_SAMPLE_DELAY_MS = 20; // ms between readings
static const long SENSOR_TIMEOUT_US = 20000; // about 340 cm max range
static const int CONFIRM_READS = 3; // num reads that must agree before acting

// ============================================================
//  WALL THRESHOLDS (cm)
// ============================================================
static const float WALL_PRESENT_CM = 13.0; // <= wall present
static const float WALL_ABSENT_CM = 30.0; // >= wall absent
static const float STOP_CM = 3.0; // front emergency stop, back up since too close

// ============================================================
//  PID CORRIDOR ALIGNMENT
// ============================================================
static const float LEFT_TARGET_CM = 6.0; // desired gap to left wall
static const float PID_KP = 0.4f;
static const float PID_KI = 0.01f;
static const float PID_KD = 0.2f;
static const float PID_MAX_OUTPUT_DEG = 20.0f;
static const float PID_INTEGRAL_MAX = 30.0f;

// ============================================================
//  BEHAVIOUR
// ============================================================
static const int STUCK_THRESHOLD = 6;
static const int EXIT_STREAK = 4; // consecutive everything open readings = exited!
static const bool DEBUG_SERIAL = true;

// ============================================================
//  SERVO OBJECTS
// ============================================================
Servo left_outer, left_inner, right_outer, right_inner;
Servo left_outer_v, left_inner_v, right_outer_v, right_inner_v;

// ============================================================
//  ROBOT STATE
// ============================================================
int stuckCounter = 0;
int openStreak = 0;
bool mazeExited = false;

bool prevWallFront = false;
bool prevWallLeft = false;
bool prevWallRight = false;

float pidIntegral = 0.0f;
float pidLastError = 0.0f;

struct SensorReading {
  float front, left, right;
  bool wallFront, wallLeft, wallRight;
};

// ============================================================
//  WAIT (non-freezing)
// ============================================================
void pause(unsigned int ms) {
  unsigned long target = millis() + ms;
  while (millis() < target) {
    delay(1);
    }
}

// ============================================================
//  SERVO INIT
// ============================================================
void init_servos() {
  left_outer.write(90);    left_inner.write(120);
  right_outer.write(90);   right_inner.write(60);
  left_outer_v.write(90);  left_inner_v.write(90);
  right_outer_v.write(90); right_inner_v.write(90);
  pause(1000);
}

// ============================================================
//  FORWARD (cm), servos centered at 90. (not using this yet, using step once instead though can go back to this to try)
// ============================================================
void forward(float cm) {
  int steps = max(1, (int)(cm / linear_displacement));
  for (int i = 0; i < steps; i++) {
    left_outer_v.write(90 - leg_lift_height);
    pause(motor_wait/2);
    right_outer_v.write(90 + leg_lift_height);
    pause(motor_wait/4);
    left_outer.write(90 + foot_rotation + left_off_displacement);
    pause(motor_wait/2);
    right_outer.write(90 - foot_rotation + right_off_displacement);
    pause(motor_wait/4);
    left_outer_v.write(90);
    pause(motor_wait/2);
    right_outer_v.write(90);
    pause(motor_wait/4);
    left_outer.write(90 - foot_rotation + left_off_displacement);
    pause(motor_wait/2);
    right_outer.write(90 + foot_rotation - right_off_displacement);
    pause(motor_wait/4);
  }
}

// ============================================================
//  TURN (degrees, - left, + right)
// ============================================================
void turn(int degrees) {
  if (degrees == 0) return;
  float scale = abs(degrees) / 90.0f;
  int inside_rotation = (int)(foot_rotation * (1.0f - scale));
  int outside_rotation = foot_rotation;

  int left_rotation, right_rotation;
  if (degrees > 0) { // right turn
    left_rotation  = outside_rotation;
    right_rotation = inside_rotation;
  } else { // left turn
    left_rotation  = inside_rotation;
    right_rotation = outside_rotation;
  }

  int cycles = max(1, (int)(abs(degrees) / degrees_per_cycle + 0.5f));
  for (int i = 0; i < cycles; i++) {
    left_inner_v.write(90 - leg_lift_height);
    right_inner_v.write(90 + leg_lift_height);
    pause(motor_wait);
    left_inner.write(90 - left_rotation);
    right_inner.write(90 + right_rotation);
    pause(motor_wait);
    left_inner_v.write(90);
    right_inner_v.write(90);
    pause(motor_wait);
    left_outer_v.write(90 - leg_lift_height);
    right_outer_v.write(90 + leg_lift_height);
    pause(motor_wait);
    left_inner.write(90 + left_rotation);
    right_inner.write(90 - right_rotation);
    pause(motor_wait);
    left_outer_v.write(90);
    right_outer_v.write(90);
    pause(motor_wait);
  }
}

// ============================================================
//  SINGLE STEP FORWARD
// ============================================================
void stepOnce() {
  if (DEBUG_SERIAL) Serial.println("Stepping once");
  forward(1);
}

void stepBack(int steps) {
  if (DEBUG_SERIAL) Serial.println("Stepping back");

  for (int i = 0; i < steps; i++) {
    left_inner_v.write(90 + leg_lift_height);
    right_inner_v.write(90 - leg_lift_height);
    pause(motor_wait);
    left_inner.write(90 + foot_rotation);
    right_inner.write(90 - foot_rotation);
    pause(motor_wait);
    left_inner_v.write(90);
    right_inner_v.write(90);
    pause(motor_wait);
    left_outer_v.write(90 - leg_lift_height);
    right_outer_v.write(90 + leg_lift_height);
    pause(motor_wait);
    left_inner.write(90 - foot_rotation);
    right_inner.write(90 + foot_rotation);
    pause(motor_wait);
    left_outer_v.write(90);
    right_outer_v.write(90);
    pause(motor_wait);
  }
}

// ============================================================
//  SENSORS
// ============================================================
float readSensorRaw(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT_US);
  if (dur == 0) return 999.0f; // maybe consider having 0 for bad reads instead? idk yet...
  return dur * 0.0343f / 2.0f;
}

float readSensor(int trigPin, int echoPin) {
  float s[SENSOR_SAMPLES];
  for (int i = 0; i < SENSOR_SAMPLES; i++) {
    s[i] = readSensorRaw(trigPin, echoPin);
    pause(SENSOR_SAMPLE_DELAY_MS);
  }
  // insertion sort => median
  for (int i = 1; i < SENSOR_SAMPLES; i++) {
    float key = s[i]; int j = i - 1;
    while (j >= 0 && s[j] > key) { s[j+1] = s[j]; j--; }
    s[j+1] = key;
  }
  return s[SENSOR_SAMPLES / 2];
}

bool hysteresisWall(float cm, bool prev) {
  if (cm <= WALL_PRESENT_CM) return true;
  if (cm >= WALL_ABSENT_CM) return false;
  return prev; // not certain, say it's same as before
}

SensorReading readAllSensors() {
  SensorReading r;
  r.front = readSensor(FRONT_TRIG, FRONT_ECHO);
  r.left = readSensor(LEFT_TRIG,  LEFT_ECHO);
  r.right = readSensor(RIGHT_TRIG, RIGHT_ECHO);
  r.wallFront = hysteresisWall(r.front, prevWallFront);
  r.wallLeft = hysteresisWall(r.left, prevWallLeft);
  r.wallRight = hysteresisWall(r.right, prevWallRight);
  prevWallFront = r.wallFront;
  prevWallLeft = r.wallLeft;
  prevWallRight = r.wallRight;
  if (DEBUG_SERIAL)
    Serial.printf("Sensors | F:%.1fcm(%d) L:%.1fcm(%d) R:%.1fcm(%d)\n",
      r.front, r.wallFront, r.left, r.wallLeft, r.right, r.wallRight);
  return r;
}

SensorReading readSensorsConfirmed() {
  SensorReading s = readAllSensors();
  for (int i = 1; i < CONFIRM_READS; i++) {
    pause(SENSOR_SAMPLE_DELAY_MS * SENSOR_SAMPLES);
    SensorReading s2 = readAllSensors();
    s.wallFront = s.wallFront && s2.wallFront;
    s.wallLeft = s.wallLeft  && s2.wallLeft;
    s.wallRight = s.wallRight && s2.wallRight;
    s.front = min(s.front, s2.front);
    s.left = min(s.left,  s2.left);
    s.right = min(s.right, s2.right);
  }
  if (DEBUG_SERIAL)
    Serial.printf("CONFIRMED | F:%d L:%d R:%d\n",
      s.wallFront, s.wallLeft, s.wallRight);
  return s;
}

// ============================================================
//  PID CORRIDOR ALIGNMENT
// ============================================================
float pidAlign(const SensorReading& s) {
  float error = 0.0f;
  bool  active = false;

  if (s.wallLeft) {
    error  = s.left - LEFT_TARGET_CM;
    active = true;
  } else if (s.wallRight) {
    error  = -(s.right - LEFT_TARGET_CM);
    active = true;
  }

  if (!active) { pidIntegral = 0; pidLastError = 0; return 0; }

  pidIntegral = constrain(pidIntegral + error, -PID_INTEGRAL_MAX, PID_INTEGRAL_MAX);
  float deriv = error - pidLastError;
  pidLastError = error;
  float out = PID_KP * error + PID_KI * pidIntegral + PID_KD * deriv;
  return constrain(out, -PID_MAX_OUTPUT_DEG, PID_MAX_OUTPUT_DEG);
}

// ============================================================
//  LEFT-WALL FOLLOW DECISION
// ============================================================
int leftWallDecide(const SensorReading& s) {
  if (!s.wallLeft)  return 3; // left open, turn left
  if (!s.wallFront) return 0; // front open, straight
  if (!s.wallRight) return 1; // right open, turn right
  return 2; // dead end, turn around
}

void executeTurn(int relDir) {
  switch (relDir) {
    case 1: if (DEBUG_SERIAL) Serial.println(">> TURN RIGHT"); turn(90); break;
    case 2: if (DEBUG_SERIAL) Serial.println(">> TURN AROUND"); turn(90); turn(90); break;
    case 3: if (DEBUG_SERIAL) Serial.println(">> TURN LEFT"); turn(-90); break;
    default: break;
  }
  pidIntegral = 0;
  pidLastError = 0;
}

// ============================================================
//  EXIT DETECTION
// ============================================================
bool checkIfExited(const SensorReading& s) {
  if (!s.wallFront && !s.wallLeft && !s.wallRight) openStreak++;
  else openStreak = 0;
  return openStreak >= EXIT_STREAK;
}

// ============================================================
//  STUCK RECOVERY (consider changing this, perhaps just step back no turn? check if left/right open?)
// ============================================================
void attemptRecovery() {
  if (DEBUG_SERIAL) Serial.println("stuck, trying to recover :(");
  static int recoveryCount = 0;
  stepBack(3);
  pause(200);
  turn((recoveryCount++ % 2 == 0) ? 45 : -45);
  pause(200);
  pidIntegral = 0; pidLastError = 0;
  stuckCounter = 0;
}

// ============================================================
//  MAIN STEP
// ============================================================
void mazeStep() {
  SensorReading s = readSensorsConfirmed();

  if (checkIfExited(s)) {
    mazeExited = true;
    if (DEBUG_SERIAL) Serial.println(">>>>> Maze exited!!");
    return;
  }

  if (s.wallFront && s.front < STOP_CM) {
    if (DEBUG_SERIAL) Serial.printf("EMERGENCY STOP front=%.1fcm\n", s.front);
    stuckCounter++;
    if (stuckCounter >= STUCK_THRESHOLD) attemptRecovery();
    return;
  }

  int relDir = leftWallDecide(s);

  if (relDir != 0) {
    executeTurn(relDir);
    SensorReading s2 = readSensorsConfirmed();
    if (!s2.wallFront) {
      stepOnce();
      stuckCounter = 0;
    } else {
      if (DEBUG_SERIAL) Serial.println("path blocked after turn");
      stuckCounter++;
    }
  } else {
    float correction = pidAlign(s);
    if (abs(correction) > 2.0f) {
      turn((int)correction);
      turn(-(int)correction);
    }
    stepOnce();
    stuckCounter = 0;
  }

  if (stuckCounter >= STUCK_THRESHOLD) attemptRecovery();
}

// ============================================================
//  SETUP & LOOP
// ============================================================
void setup() {
  if (DEBUG_SERIAL) {
    Serial.begin(115200);
    while (!Serial) pause(10);
    Serial.println("Maze solver, initialising");
  }

  pinMode(FRONT_TRIG, OUTPUT); pinMode(FRONT_ECHO, INPUT);
  pinMode(LEFT_TRIG, OUTPUT); pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_TRIG, OUTPUT); pinMode(RIGHT_ECHO, INPUT);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  left_outer.setPeriodHertz(50); left_inner.setPeriodHertz(50);
  right_outer.setPeriodHertz(50); right_inner.setPeriodHertz(50);
  left_outer_v.setPeriodHertz(50); left_inner_v.setPeriodHertz(50);
  right_outer_v.setPeriodHertz(50); right_inner_v.setPeriodHertz(50);

  left_outer.attach(LEFT_OUTER_PIN, 500, 2400);
  left_inner.attach(LEFT_INNER_PIN, 500, 2400);
  right_outer.attach(RIGHT_OUTER_PIN, 500, 2400);
  right_inner.attach(RIGHT_INNER_PIN, 500, 2400);
  left_outer_v.attach(LEFT_OUTER_V_PIN, 500, 2400);
  left_inner_v.attach(LEFT_INNER_V_PIN, 500, 2400);
  right_outer_v.attach(RIGHT_OUTER_V_PIN, 500, 2400);
  right_inner_v.attach(RIGHT_INNER_V_PIN, 500, 2400);

  init_servos();
  pause(5000);

  if (DEBUG_SERIAL) Serial.println("Ready, beginning maze solve");
}

bool move = true;
void loop() {
  // if (mazeExited) { pause(5000); return; }
  // mazeStep();

  if (Serial.available() > 0) {
    char input = Serial.read(); // Read the character typed

    if (input == '1') {
      move = true;
      Serial.println("move is now: TRUE");
    } 
    else if (input == '0') {
      move = false;
      Serial.println("move is now: FALSE");
    }
  }
  if (move) {
    //stepOnce();
    forward(1);
    
  } else {
    init_servos();
  }

  // init_servos();1
  // stepOnce();
  // readAllSensors();
  // pause(800);
}