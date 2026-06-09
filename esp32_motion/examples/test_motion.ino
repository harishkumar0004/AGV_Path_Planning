#include "motor_config.h"
#include "step_generator.h"
#include "differential_drive.h"
#include "motion_controller.h"


const uint8_t LEFT_STEP_PIN = 4;
const uint8_t LEFT_DIR_PIN = 13;
const uint8_t LEFT_ENABLE_PIN = 14;

const uint8_t RIGHT_STEP_PIN = 16;
const uint8_t RIGHT_DIR_PIN = 26;
const uint8_t RIGHT_ENABLE_PIN = 25;

MotorConfig motor_config = {
  117.0,    // wheel_diameter_mm
  324.0,    // wheel_base_mm
  40000,    // pulses_per_revolution: change only this for T60 pulse setting
  40000.0,  // max_frequency_hz
  10000.0   // max_acceleration_hz_per_sec
};

StepGenerator left_motor(
  LEFT_STEP_PIN,
  LEFT_DIR_PIN,
  LEFT_ENABLE_PIN,
  0
);

StepGenerator right_motor(
  RIGHT_STEP_PIN,
  RIGHT_DIR_PIN,
  RIGHT_ENABLE_PIN,
  1
);

DifferentialDrive drive(left_motor, right_motor);
MotionController motion_controller(motor_config, drive);

enum TestMotionState {
  TEST_FORWARD,
  TEST_PAUSE_AFTER_FORWARD,
  TEST_BACKWARD,
  TEST_PAUSE_AFTER_BACKWARD,
  TEST_LEFT,
  TEST_PAUSE_AFTER_LEFT,
  TEST_RIGHT,
  TEST_PAUSE_AFTER_RIGHT
};

TestMotionState test_state = TEST_FORWARD;
unsigned long pause_started_ms = 0;


void setup() {
  Serial.begin(115200);
  motion_controller.begin();

  Serial.println("ESP32 hardware-timer motion test started");
  Serial.println("No delay-based pulse generation is used.");
}


void loop() {
  motion_controller.update();

  switch (test_state) {
    case TEST_FORWARD:
      if (!motion_controller.isRunning()) {
        Serial.println("Command: moveForward 500 mm");
        motion_controller.moveForward(500.0);
        test_state = TEST_PAUSE_AFTER_FORWARD;
      }
      break;

    case TEST_PAUSE_AFTER_FORWARD:
      waitForMotionThenPause(TEST_BACKWARD);
      break;

    case TEST_BACKWARD:
      if (!motion_controller.isRunning()) {
        Serial.println("Command: moveBackward 500 mm");
        motion_controller.moveBackward(500.0);
        test_state = TEST_PAUSE_AFTER_BACKWARD;
      }
      break;

    case TEST_PAUSE_AFTER_BACKWARD:
      waitForMotionThenPause(TEST_LEFT);
      break;

    case TEST_LEFT:
      if (!motion_controller.isRunning()) {
        Serial.println("Command: turnLeft 90 deg");
        motion_controller.turnLeft(90.0);
        test_state = TEST_PAUSE_AFTER_LEFT;
      }
      break;

    case TEST_PAUSE_AFTER_LEFT:
      waitForMotionThenPause(TEST_RIGHT);
      break;

    case TEST_RIGHT:
      if (!motion_controller.isRunning()) {
        Serial.println("Command: turnRight 90 deg");
        motion_controller.turnRight(90.0);
        test_state = TEST_PAUSE_AFTER_RIGHT;
      }
      break;

    case TEST_PAUSE_AFTER_RIGHT:
      waitForMotionThenPause(TEST_FORWARD);
      break;
  }
}


void waitForMotionThenPause(TestMotionState next_state) {
  if (motion_controller.isRunning()) {
    pause_started_ms = 0;
    return;
  }

  if (pause_started_ms == 0) {
    pause_started_ms = millis();
  }

  if ((millis() - pause_started_ms) >= 1000) {
    pause_started_ms = 0;
    test_state = next_state;
  }
}
