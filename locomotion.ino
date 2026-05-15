#include <ESP32Servo.h>

Servo left_outer;
Servo left_inner;
Servo right_outer;
Servo right_inner;

Servo left_outer_v;
Servo left_inner_v;
Servo right_outer_v;
Servo right_inner_v;


// Define constant speeds
static const int linear_displacement = 1;
static const int anguler_displacement = 1;


// Define pins
static const int LEFT_OUTER_PIN  = 27;
static const int LEFT_INNER_PIN  = 13;
static const int RIGHT_OUTER_PIN = 14;
static const int RIGHT_INNER_PIN = 15;
static const int LEFT_OUTER_V_PIN  = 33;
static const int LEFT_INNER_V_PIN  = 12;
static const int RIGHT_OUTER_V_PIN = 21;
static const int RIGHT_INNER_V_PIN = 32;

// Track current positions
int left_outer_pos  = 0;
int left_inner_pos  = 0;
int right_outer_pos = 0;
int right_inner_pos = 0;

int left_outer_pos_V  = 0;
int left_inner_pos_V  = 0;
int right_outer_pos_V = 0;
int right_inner_pos_V = 0;

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

// Move function
void move(ServoName servo_name, int degrees) {

  Servo* servo;
  int* position;

  switch (servo_name) {
    case LEFT_OUTER:
      servo = &left_outer;
      position = &left_outer_pos;
      break;
    case LEFT_INNER:
      servo = &left_inner;
      position = &left_inner_pos;
      break;
    case RIGHT_OUTER:
      servo = &right_outer;
      position = &right_outer_pos;
      break;
    case RIGHT_INNER:
      servo = &right_inner;
      position = &right_inner_pos;
      break;
    
    case LEFT_OUTER_V:
      servo = &left_outer_v;
      position = &left_outer_pos_V;
      break;
    case LEFT_INNER_V:
      servo = &left_inner_v;
      position = &left_inner_pos_V;
      break;
    case RIGHT_OUTER_V:
      servo = &right_outer_v;
      position = &right_outer_pos_V;
      break;
    case RIGHT_INNER_V:
      servo = &right_inner;
      position = &right_inner_pos_V;
      break;
  }
  servo->write(*position);
}

void setup() {

  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ESP32PWM::allocateTimer(4);
  ESP32PWM::allocateTimer(5);
  ESP32PWM::allocateTimer(6);
  ESP32PWM::allocateTimer(7);

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
  left_outer.write(left_outer_pos);
  left_inner.write(left_inner_pos);
  right_outer.write(right_outer_pos);
  right_inner.write(right_inner_pos);

  left_outer_v.write(left_outer_pos);
  left_inner_v.write(left_inner_pos);
  right_outer_v.write(right_outer_pos);
  right_inner_v.write(right_inner_pos);
}

// positive is right, negative is left
void turn(int degrees) {
  if (degrees > 0) {
    //turn left
  } else {
    //turn right
  }
}

void init_servos() {
  left_outer.write(0);
  left_inner.write(0);
  right_outer.write(0);
  right_inner.write(0);

  left_outer_v.write(0);
  left_inner_v.write(0);
  right_outer_v.write(0);
  right_inner_v.write(0);
  delay(1000);
}

void forward(float inches) {
  for (int i=0; i<linear_displacement; i++) {
    LEFT_INNER_V.write(-30);
    RIGHT_INNER_V.write(-30);
    delay(100);

    LEFT_INNER.write(75);
    RIGHT_INNER.write(75);
    delay(100);

    LEFT_INNER_V.write(0);
    RIGHT_INNER_V(0);
    delay(100);

    LEFT_OUTER_V.write(30);
    RIGHT_OUTER_V.write(30);
    delay(100);

    LEFT_INNER.write(-75);
    RIGHT_INNER.write(-75);
    delay(100);

    LEFT_OUTER_V.write(0);
    RIGHT_OUTER_V.write(0);
    delay(100);
  }
}

void loop() {
  delay(500);
  right_inner.write(90);
  delay(500);
  right_inner.write(0);
}