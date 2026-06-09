#include "../motor_config.h"
#include "../step_generator.h"
#include "../differential_drive.h"
#include "../motion_controller.h"
#include "../serial_command_handler.h"


const uint8_t LEFT_STEP_PIN = 25;
const uint8_t LEFT_DIR_PIN = 26;
const uint8_t LEFT_ENABLE_PIN = 27;

const uint8_t RIGHT_STEP_PIN = 14;
const uint8_t RIGHT_DIR_PIN = 12;
const uint8_t RIGHT_ENABLE_PIN = 13;

MotorConfig motor_config = {
  100.0,    // wheel_diameter_mm
  300.0,    // wheel_base_mm
  10000,    // pulses_per_revolution
  10000.0,  // max_frequency_hz
  5000.0,   // max_acceleration_hz_per_sec
  50.0,     // min_start_frequency_hz
  50        // direction_change_settling_ms
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
SerialCommandHandler serial_handler(Serial, motion_controller, 2000);


void setup() {
  serial_handler.begin(115200);
  motion_controller.begin();

  Serial.println("ESP32 serial motion example started");
  Serial.println("Valid commands: FORWARD, LEFT, RIGHT, STOP");
}


void loop() {
  serial_handler.update();
  motion_controller.update();
}
