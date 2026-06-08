#include "step_generator.h"
#include "motion_controller.h"


// Update these pins to match your wiring from Arduino Mega to the T60 drivers.
const uint8_t LEFT_STEP_PIN = 2;
const uint8_t LEFT_DIR_PIN = 3;
const uint8_t LEFT_ENABLE_PIN = 4;

const uint8_t RIGHT_STEP_PIN = 5;
const uint8_t RIGHT_DIR_PIN = 6;
const uint8_t RIGHT_ENABLE_PIN = 7;

const float MAX_SPEED_STEPS_PER_SEC = 800.0;
const float ACCELERATION_STEPS_PER_SEC2 = 400.0;

StepGenerator leftMotor(
  LEFT_STEP_PIN,
  LEFT_DIR_PIN,
  LEFT_ENABLE_PIN
);

StepGenerator rightMotor(
  RIGHT_STEP_PIN,
  RIGHT_DIR_PIN,
  RIGHT_ENABLE_PIN
);

MotionController motionController(
  leftMotor,
  rightMotor,
  MAX_SPEED_STEPS_PER_SEC,
  ACCELERATION_STEPS_PER_SEC2
);


void setup() {
  Serial.begin(115200);
  motionController.begin();

  Serial.println("Arduino Mega motion controller test started");
  Serial.println("All distances are in steps.");
}


void loop() {
  motionController.moveForward(1000);
  delay(1000);

  motionController.moveBackward(1000);
  delay(1000);

  motionController.turnLeft(500);
  delay(1000);

  motionController.turnRight(500);
  delay(1000);
}
