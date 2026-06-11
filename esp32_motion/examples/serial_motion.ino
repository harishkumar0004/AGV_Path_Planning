#include "motor_config.h"
#include "step_generator.h"
#include "differential_drive.h"
#include "motion_controller.h"
#include "serial_command_handler.h"


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
SerialCommandHandler serial_handler(Serial, motion_controller, 0, &drive);


void setup() {
  serial_handler.begin(115200);
  motion_controller.begin();

  Serial.println("ESP32 serial motion example started");
  Serial.println("Commands: START_FORWARD, START_SLOW_FORWARD, STOP, TURN_LEFT, TURN_RIGHT, STATUS");
  Serial.println("Validation pulses: LEFT_PULSE 100, RIGHT_PULSE 100");
  Serial.println("Calibration: TURN_RIGHT 10, TURN_RIGHT 20, TURN_RIGHT 30, ...");
}


void loop() {
  serial_handler.update();

  if (!serial_handler.isValidationPulseActive()) {
    motion_controller.update();
  }
}
