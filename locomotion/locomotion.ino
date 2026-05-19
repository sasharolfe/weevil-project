#include <ESP32Servo.h>

Servo left_outer;
Servo left_inner;
Servo right_outer;
Servo right_inner;

Servo left_outer_v;
Servo left_inner_v;
Servo right_outer_v;
Servo right_inner_v;

// Define constant displacement
static const int linear_displacement = 1;
static const int anguler_displacement = 1;

// Define constant angles
static const int leg_lift_height = 15;
static const int foot_rotation = 20;

// Define constant time needed for movement
static const int motor_wait = 150;

// Define pins
static const int LEFT_OUTER_PIN  = 15;
static const int LEFT_INNER_PIN  = 13;
static const int RIGHT_OUTER_PIN = 27;
static const int RIGHT_INNER_PIN = 33;
static const int LEFT_OUTER_V_PIN  = 21;
static const int LEFT_INNER_V_PIN  = 12;
static const int RIGHT_OUTER_V_PIN = 32;
static const int RIGHT_INNER_V_PIN = 14;

// Simple servo identifier enum
enum ServoName {
  LEFT_OUTER,
  LEFT_INNER,
  RIGHT_OUTER,
  RIGHT_INNER,
  LEFT_OUTER_V,
  LEFT_INNER_V,
  RIGHT_OUTER_V,
  RIGHT_INNER_V
};

// delay that doesn't freeze the computer
void wait(unsigned int time) {
  unsigned long target_time = millis() + time;
  while (millis() < target_time) {
    delay(1);
  }
}

void setup() {
  // Set all servos to standard 50Hz
  left_outer.setPeriodHertz(50);
  left_inner.setPeriodHertz(50);
  right_outer.setPeriodHertz(50);
  right_inner.setPeriodHertz(50);
  left_outer_v.setPeriodHertz(50);
  left_inner_v.setPeriodHertz(50);
  right_outer_v.setPeriodHertz(50);
  right_inner_v.setPeriodHertz(50);

  // Attach servos
  left_outer.attach(LEFT_OUTER_PIN, 500, 2400);
  left_inner.attach(LEFT_INNER_PIN, 500, 2400);
  right_outer.attach(RIGHT_OUTER_PIN, 500, 2400);
  right_inner.attach(RIGHT_INNER_PIN, 500, 2400);

  left_outer_v.attach(LEFT_OUTER_V_PIN, 500, 2400);
  left_inner_v.attach(LEFT_INNER_V_PIN, 500, 2400);
  right_outer_v.attach(RIGHT_OUTER_V_PIN, 500, 2400);
  right_inner_v.attach(RIGHT_INNER_V_PIN, 500, 2400);

  // Initialize all servo positions
  left_outer.write(90);
  left_inner.write(90);
  right_outer.write(90);
  right_inner.write(90);

  left_outer_v.write(90);
  left_inner_v.write(90);
  right_outer_v.write(90);
  right_inner_v.write(90);
}

// positive is right, negative is left
void turn(int degrees) {
  float scale = abs(degrees) / 90.0;
  int inside_rotation  = foot_rotation * (1.0 - scale);
  int outside_rotation = foot_rotation;

  int left_rotation;
  int right_rotation;

  if (degrees >= 0) {
    left_rotation  = inside_rotation;
    right_rotation = outside_rotation;
  } else {
    left_rotation  = outside_rotation;
    right_rotation = inside_rotation;
  }

  for (int i=0; i < anguler_displacement; i++) {
  // One turning step
    left_inner_v.write(90+leg_lift_height);
    right_inner_v.write(90-leg_lift_height);
    delay(motor_wait);

    left_inner.write(90-left_rotation);
    right_inner.write(90+right_rotation);
    delay(motor_wait);

    left_inner_v.write(90);
    right_inner_v.write(90);
    delay(motor_wait);

    left_outer_v.write(90-leg_lift_height);
    right_outer_v.write(90+leg_lift_height);
    delay(motor_wait);

    left_inner.write(90+left_rotation);
    right_inner.write(90-right_rotation);
    delay(motor_wait);

    left_outer_v.write(90);
    right_outer_v.write(90);
    delay(motor_wait);
  }
}

void init_servos() {
  left_outer.write(90);
  left_inner.write(90);
  right_outer.write(90);
  right_inner.write(90);

  left_outer_v.write(90);
  left_inner_v.write(90);
  right_outer_v.write(90);
  right_inner_v.write(90);
  wait(1000);
}

void forward(float inches) {
  for (int i=0; i < inches/linear_displacement; i++) {
    left_inner_v.write(90+leg_lift_height);
    right_inner_v.write(90-leg_lift_height);
    wait(motor_wait);

    left_inner.write(90-foot_rotation);
    right_inner.write(90+foot_rotation);
    wait(motor_wait);

    left_inner_v.write(90);
    right_inner_v.write(90);
    wait(motor_wait);

    left_outer_v.write(90-leg_lift_height);
    right_outer_v.write(90+leg_lift_height);
    wait(motor_wait);

    left_inner.write(90+foot_rotation);
    right_inner.write(90-foot_rotation);
    wait(motor_wait);

    left_outer_v.write(90);
    right_outer_v.write(90);
    wait(motor_wait);
  }
}

void loop() {
  forward(1);
  wait(1000);
}