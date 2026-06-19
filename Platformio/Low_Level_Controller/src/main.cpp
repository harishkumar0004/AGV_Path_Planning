#include <Arduino.h>
#include "RobotConfig.h"
#include "StepperTimerMotor.h"
#include "DifferentialDrive.h"

// =====================================================
// Motor objects
// =====================================================

StepperTimerMotor leftMotor(
    LEFT_PUL_PIN,
    LEFT_DIR_PIN,
    LEFT_ENA_PIN,
    LEFT_DIR_INVERT,
    ENABLE_ACTIVE_LOW
);

StepperTimerMotor rightMotor(
    RIGHT_PUL_PIN,
    RIGHT_DIR_PIN,
    RIGHT_ENA_PIN,
    RIGHT_DIR_INVERT,
    ENABLE_ACTIVE_LOW
);

DifferentialDrive drive(leftMotor, rightMotor);

String inputLine = "";
String lastRobotCommand = "STOP";

// =====================================================
// ESP32 hardware timer
// =====================================================

hw_timer_t* motorTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onMotorTimer() {
    portENTER_CRITICAL_ISR(&timerMux);

    leftMotor.isrUpdate();
    rightMotor.isrUpdate();

    portEXIT_CRITICAL_ISR(&timerMux);
}

void setupMotorTimer() {
    // ESP32 APB clock is usually 80 MHz.
    // Prescaler 80 gives 1 MHz timer clock.
    // So 1 timer count = 1 us.
    motorTimer = timerBegin(0, 80, true);

    timerAttachInterrupt(motorTimer, &onMotorTimer, true);

    // Example: MOTOR_TIMER_TICK_US = 20 gives 50 kHz ISR rate.
    timerAlarmWrite(motorTimer, MOTOR_TIMER_TICK_US, true);
    timerAlarmEnable(motorTimer);
}

// =====================================================
// Debug helper functions
// =====================================================

// This shows the internal software command sign.
// true  means setDirection(true)
// false means setDirection(false)
const char* internalSign(bool directionForward) {
    return directionForward ? "+" : "-";
}

const char* internalName(bool directionForward) {
    return directionForward ? "INTERNAL_POSITIVE" : "INTERNAL_NEGATIVE";
}

// This display is ONLY for human debugging.
// It maps your verified robot behavior into understandable physical effect.
//
// Your verified behavior:
// CW command uses:
//     left internal negative
//     right internal positive
//
// But physically you understand CW as:
//     left forward effect
//     right backward effect
//
// So for display:
//     internal negative -> FORWARD_EFFECT
//     internal positive -> BACKWARD_EFFECT
const char* physicalEffect(bool directionForward) {
    return directionForward ? "BACKWARD_EFFECT" : "FORWARD_EFFECT";
}

const char* robotMotionFromCommand(const String& command) {
    if (command == "FORWARD") return "AGV_FORWARD";
    if (command == "BACKWARD") return "AGV_BACKWARD";
    if (command == "CLOCKWISE") return "AGV_CLOCKWISE";
    if (command == "COUNTER_CLOCKWISE") return "AGV_COUNTER_CLOCKWISE";
    if (command == "W_POSITIVE") return "AGV_CLOCKWISE_BY_W";
    if (command == "W_NEGATIVE") return "AGV_COUNTER_CLOCKWISE_BY_W";
    if (command == "LEFT_ONLY") return "LEFT_WHEEL_ONLY_TEST";
    if (command == "RIGHT_ONLY") return "RIGHT_WHEEL_ONLY_TEST";
    if (command == "STOP") return "AGV_STOPPED";
    return "UNKNOWN";
}

// =====================================================
// Printing
// =====================================================

void printHelp() {
    Serial.println();
    Serial.println("=== AGV FIREBEETLE ESP32-E HARDWARE TIMER MOTOR TEST ===");
    Serial.println();
    Serial.println("Robot commands:");
    Serial.println("  F 1       -> forward 1 RPM");
    Serial.println("  B 1       -> backward 1 RPM");
    Serial.println("  CW 1      -> clockwise 1 RPM");
    Serial.println("  CCW 1     -> counter-clockwise 1 RPM");
    Serial.println();
    Serial.println("Wheel-only commands:");
    Serial.println("  L F 1     -> left wheel internal forward test");
    Serial.println("  L B 1     -> left wheel internal backward test");
    Serial.println("  R F 1     -> right wheel internal forward test");
    Serial.println("  R B 1     -> right wheel internal backward test");
    Serial.println();
    Serial.println("Robot velocity test:");
    Serial.println("  W 1       -> setRobotVelocity(0, +1 rad/s)");
    Serial.println("  W -1      -> setRobotVelocity(0, -1 rad/s)");
    Serial.println();
    Serial.println("Other:");
    Serial.println("  S         -> stop");
    Serial.println("  STATUS    -> print status");
    Serial.println("  HELP      -> print help");
    Serial.println();
}

void printStatus() {
    bool leftDir = leftMotor.getDirectionForward();
    bool rightDir = rightMotor.getDirectionForward();

    Serial.println("----- STATUS -----");

    Serial.print("Robot command: ");
    Serial.println(lastRobotCommand);

    Serial.print("Expected robot motion: ");
    Serial.println(robotMotionFromCommand(lastRobotCommand));

    Serial.println();

    Serial.print("Left Hz: ");
    Serial.println(leftMotor.getTargetHz(), 2);

    Serial.print("Right Hz: ");
    Serial.println(rightMotor.getTargetHz(), 2);

    Serial.println();

    Serial.println("Internal software sign:");
    Serial.print("  Left internal sign: ");
    Serial.print(internalSign(leftDir));
    Serial.print(" / ");
    Serial.println(internalName(leftDir));

    Serial.print("  Right internal sign: ");
    Serial.print(internalSign(rightDir));
    Serial.print(" / ");
    Serial.println(internalName(rightDir));

    Serial.println();

    Serial.println("Displayed physical effect:");
    Serial.print("  Left displayed effect: ");
    Serial.println(physicalEffect(leftDir));

    Serial.print("  Right displayed effect: ");
    Serial.println(physicalEffect(rightDir));

    Serial.println();

    Serial.println("Internal step counters:");
    Serial.print("  Left steps: ");
    Serial.println(leftMotor.getStepCount());

    Serial.print("  Right steps: ");
    Serial.println(rightMotor.getStepCount());

    Serial.println("------------------");
}

// =====================================================
// Limit functions
// =====================================================

float limitRpm(float rpm) {
    rpm = fabs(rpm);

    if (rpm > MAX_TEST_RPM) {
        rpm = MAX_TEST_RPM;
    }

    return rpm;
}

float limitSignedValue(float value, float maxAbsValue) {
    if (value > maxAbsValue) {
        value = maxAbsValue;
    }

    if (value < -maxAbsValue) {
        value = -maxAbsValue;
    }

    return value;
}

// =====================================================
// Command handling
// =====================================================

void handleCommand(String cmd) {
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.length() == 0) {
        return;
    }

    if (cmd == "HELP") {
        printHelp();
        return;
    }

    if (cmd == "S" || cmd == "STOP") {
        drive.stop();
        lastRobotCommand = "STOP";
        Serial.println("STOP");
        printStatus();
        return;
    }

    if (cmd == "STATUS") {
        printStatus();
        return;
    }

    char buffer[60];
    cmd.toCharArray(buffer, sizeof(buffer));

    char p1[12] = {0};
    char p2[12] = {0};
    char p3[12] = {0};

    int count = sscanf(buffer, "%11s %11s %11s", p1, p2, p3);

    String a = String(p1);
    String b = String(p2);
    String c = String(p3);

    // For commands like:
    // F 1
    // B 1
    // CW 1
    // CCW 1
    // W -1
    float value2 = DEFAULT_TEST_RPM;
    if (count >= 2) {
        value2 = atof(p2);
    }

    // For commands like:
    // L F 1
    // R B 1
    float value3 = DEFAULT_TEST_RPM;
    if (count >= 3) {
        value3 = atof(p3);
    }

    // ------------------------------
    // Robot-level commands
    // ------------------------------

    if (a == "F") {
        float rpm = limitRpm(value2);

        drive.driveForwardRPM(rpm);
        lastRobotCommand = "FORWARD";

        Serial.print("FORWARD ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "B") {
        float rpm = limitRpm(value2);

        drive.driveBackwardRPM(rpm);
        lastRobotCommand = "BACKWARD";

        Serial.print("BACKWARD ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "CW") {
        float rpm = limitRpm(value2);

        drive.rotateClockwiseRPM(rpm);
        lastRobotCommand = "CLOCKWISE";

        Serial.print("CLOCKWISE ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "CCW") {
        float rpm = limitRpm(value2);

        drive.rotateCounterClockwiseRPM(rpm);
        lastRobotCommand = "COUNTER_CLOCKWISE";

        Serial.print("COUNTER-CLOCKWISE ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }

    // ------------------------------
    // Individual wheel test commands
    // ------------------------------

    else if (a == "L" && b == "F") {
        float rpm = limitRpm(value3);

        leftMotor.setDirection(true);
        leftMotor.setFrequencyHz(rpmToHz(rpm));
        lastRobotCommand = "LEFT_ONLY";

        Serial.print("LEFT INTERNAL FORWARD TEST ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "L" && b == "B") {
        float rpm = limitRpm(value3);

        leftMotor.setDirection(false);
        leftMotor.setFrequencyHz(rpmToHz(rpm));
        lastRobotCommand = "LEFT_ONLY";

        Serial.print("LEFT INTERNAL BACKWARD TEST ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "R" && b == "F") {
        float rpm = limitRpm(value3);

        rightMotor.setDirection(true);
        rightMotor.setFrequencyHz(rpmToHz(rpm));
        lastRobotCommand = "RIGHT_ONLY";

        Serial.print("RIGHT INTERNAL FORWARD TEST ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "R" && b == "B") {
        float rpm = limitRpm(value3);

        rightMotor.setDirection(false);
        rightMotor.setFrequencyHz(rpmToHz(rpm));
        lastRobotCommand = "RIGHT_ONLY";

        Serial.print("RIGHT INTERNAL BACKWARD TEST ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }

    // ------------------------------
    // Robot angular velocity command
    // ------------------------------
    // W is signed.
    // W 1  means positive w.
    // W -1 means negative w.
    //
    // With your current DifferentialDrive convention:
    // setRobotVelocity(0, +w) should rotate clockwise.
    // setRobotVelocity(0, -w) should rotate counter-clockwise.
    // ------------------------------

    else if (a == "W") {
        float w = limitSignedValue(value2, 2.0f);  // rad/s safety limit

        drive.setRobotVelocity(0.0f, w);

        if (w > 0.0f) {
            lastRobotCommand = "W_POSITIVE";
        } else if (w < 0.0f) {
            lastRobotCommand = "W_NEGATIVE";
        } else {
            lastRobotCommand = "STOP";
        }

        Serial.print("SET ROBOT W RAD/S: ");
        Serial.println(w, 3);
    }

    else {
        Serial.println("Unknown command. Type HELP.");
        return;
    }

    printStatus();
}

// =====================================================
// Setup and loop
// =====================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    drive.begin();
    drive.stop();

    setupMotorTimer();

    printHelp();

    Serial.println("Robot constants:");
    Serial.print("  Wheel circumference m: ");
    Serial.println(WHEEL_CIRCUMFERENCE_M, 6);

    Serial.print("  Pulses per meter: ");
    Serial.println(PULSES_PER_METER, 2);

    Serial.print("  Pulses for 50 cm: ");
    Serial.println(distanceToPulses(0.5f));

    Serial.print("  1 RPM Hz: ");
    Serial.println(rpmToHz(1.0f), 2);

    Serial.print("  30 RPM Hz: ");
    Serial.println(rpmToHz(30.0f), 2);

    Serial.println();
    Serial.println("Verified physical convention:");
    Serial.println("  F command      -> AGV moves forward");
    Serial.println("  B command      -> AGV moves backward");
    Serial.println("  CW command     -> AGV rotates clockwise");
    Serial.println("  CCW command    -> AGV rotates counter-clockwise");
    Serial.println("  W > 0          -> expected clockwise");
    Serial.println("  W < 0          -> expected counter-clockwise");
    Serial.println();
    Serial.println("Ready.");
}

void loop() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            handleCommand(inputLine);
            inputLine = "";
        } else {
            inputLine += c;
        }
    }
}