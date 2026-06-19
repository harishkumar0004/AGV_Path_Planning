#include <Arduino.h>
#include "RobotConfig.h"
#include "StepperTimerMotor.h"
#include "DifferentialDrive.h"

// =====================================================
// Motor objects
// =====================================================

unsigned long lastAlignControlMs = 0;

float lastAlignV = 999.0f;
float lastAlignW = 999.0f;

constexpr unsigned long ALIGN_CONTROL_PERIOD_MS = 50;  // 20 Hz
constexpr float CMD_CHANGE_EPS = 0.0005f;

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

// =====================================================
// AprilTag alignment data
// =====================================================

struct TagData {
    bool visible = false;
    bool nearEdge = false;
    int id = -1;

    float xNorm = 0.0f;
    float yNorm = 0.0f;
    float thetaDeg = 0.0f;

    unsigned long lastUpdateMs = 0;
};

TagData tag;

bool alignEnabled = false;

enum AlignState {
    WAIT_TAG,
    ROUGH_YAW,
    CENTER_Y_SAFE,
    X_STEP_ROTATE,
    X_STEP_CREEP,
    X_STEP_STOP,
    X_STEP_EVALUATE,
    FINAL_Y,
    FINAL_YAW,
    ALIGNED_STOP,
    LOST_TAG
};

AlignState alignState = WAIT_TAG;

float xBeforeStep = 0.0f;
float yBeforeStep = 0.0f;
int xStepDirection = 1;
bool xStepDirectionValid = false;
unsigned long stateStartMs = 0;

String inputLine = "";
String lastRobotCommand = "STOP";

// Print TAG log slowly only
unsigned long lastAlignPrintMs = 0;

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
    motorTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(motorTimer, &onMotorTimer, true);
    timerAlarmWrite(motorTimer, MOTOR_TIMER_TICK_US, true);
    timerAlarmEnable(motorTimer);
}

// =====================================================
// Debug helper functions
// =====================================================

void sendAlignVelocity(float v, float w) {
    bool changed =
        fabs(v - lastAlignV) > CMD_CHANGE_EPS ||
        fabs(w - lastAlignW) > CMD_CHANGE_EPS;

    if (!changed) {
        return;
    }

    drive.setRobotVelocity(v, w);

    lastAlignV = v;
    lastAlignW = w;
}

const char* internalSign(bool directionForward) {
    return directionForward ? "+" : "-";
}

const char* internalName(bool directionForward) {
    return directionForward ? "INTERNAL_POSITIVE" : "INTERNAL_NEGATIVE";
}

// Display only. Does not affect motor control.
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
    if (command == "ROUGH_YAW_CW") return "ROUGH_YAW_CLOCKWISE";
    if (command == "ROUGH_YAW_CCW") return "ROUGH_YAW_COUNTER_CLOCKWISE";
    if (command == "CENTER_Y_SAFE") return "CENTER_Y_SAFE_TRANSLATE";
    if (command == "X_STEP_ROTATE") return "X_STEP_ROTATE_TO_OFFSET";
    if (command == "X_STEP_CREEP") return "X_STEP_CREEP_FORWARD";
    if (command == "X_STEP_STOP") return "X_STEP_STOPPED";
    if (command == "X_STEP_EVALUATE") return "X_STEP_EVALUATE";
    if (command == "FINAL_Y") return "FINAL_Y_TRANSLATE";
    if (command == "FINAL_YAW") return "FINAL_YAW_ZERO";
    if (command == "HARD_EDGE_STOP") return "HARD_EDGE_STOPPED";
    if (command == "ALIGN_STOP") return "ALIGN_STOPPED";
    if (command == "LEFT_ONLY") return "LEFT_WHEEL_ONLY_TEST";
    if (command == "RIGHT_ONLY") return "RIGHT_WHEEL_ONLY_TEST";
    if (command == "STOP") return "AGV_STOPPED";
    return "UNKNOWN";
}

float clampFloat(float value, float minValue, float maxValue) {
    if (value > maxValue) return maxValue;
    if (value < minValue) return minValue;
    return value;
}

const char* alignStateName(AlignState state) {
    switch (state) {
        case WAIT_TAG: return "WAIT_TAG";
        case ROUGH_YAW: return "ROUGH_YAW";
        case CENTER_Y_SAFE: return "CENTER_Y_SAFE";
        case X_STEP_ROTATE: return "X_STEP_ROTATE";
        case X_STEP_CREEP: return "X_STEP_CREEP";
        case X_STEP_STOP: return "X_STEP_STOP";
        case X_STEP_EVALUATE: return "X_STEP_EVALUATE";
        case FINAL_Y: return "FINAL_Y";
        case FINAL_YAW: return "FINAL_YAW";
        case ALIGNED_STOP: return "ALIGNED_STOP";
        case LOST_TAG: return "LOST_TAG";
        default: return "UNKNOWN";
    }
}

bool isTagTimedOut() {
    return tag.visible && (millis() - tag.lastUpdateMs > TAG_TIMEOUT_MS);
}

bool isTagNearEdge() {
    return fabs(tag.xNorm) > TAG_HARD_EDGE_LIMIT || fabs(tag.yNorm) > TAG_HARD_EDGE_LIMIT;
}

void resetXStepState() {
    xBeforeStep = 0.0f;
    yBeforeStep = 0.0f;
    xStepDirection = 1;
    xStepDirectionValid = false;
    stateStartMs = 0;
}

// =====================================================
// Alignment controller
// =====================================================

void printAlignBrief(float v, float w) {
    unsigned long now = millis();

    if (now - lastAlignPrintMs < 300) {
        return;
    }

    lastAlignPrintMs = now;

    Serial.print("ALIGN ");
    Serial.print(alignStateName(alignState));
    Serial.print(" id=");
    Serial.print(tag.id);
    Serial.print(" x=");
    Serial.print(tag.xNorm, 3);
    Serial.print(" y=");
    Serial.print(tag.yNorm, 3);
    Serial.print(" th=");
    Serial.print(tag.thetaDeg, 2);
    Serial.print(" xBefore=");
    Serial.print(xBeforeStep, 3);
    Serial.print(" yBefore=");
    Serial.print(yBeforeStep, 3);
    Serial.print(" xDir=");
    Serial.print(xStepDirection);
    Serial.print(" v=");
    Serial.print(v, 4);
    Serial.print(" w=");
    Serial.println(w, 4);
}

void updateAlignmentController() {
    // Important:
    // Do NOT call drive.stop() continuously when align is OFF.
    // Otherwise manual commands like F 1, CW 1 will immediately stop.
    if (!alignEnabled) {
        return;
    }

    unsigned long now = millis();

    if (now - lastAlignControlMs < ALIGN_CONTROL_PERIOD_MS) {
        return;
    }

    lastAlignControlMs = now;

    if (!tag.visible || isTagTimedOut()) {
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        alignState = LOST_TAG;
        lastRobotCommand = "ALIGN_STOP";
        printAlignBrief(0.0f, 0.0f);
        return;
    }

    tag.nearEdge = isTagNearEdge();

    if (tag.nearEdge) {
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        alignState = LOST_TAG;
        lastRobotCommand = "HARD_EDGE_STOP";
        printAlignBrief(0.0f, 0.0f);
        return;
    }

    float v = 0.0f;
    float w = 0.0f;

    float thetaControl = THETA_SIGN * tag.thetaDeg;
    float xControl = X_SIGN * tag.xNorm;
    float yControl = Y_SIGN * tag.yNorm;

    bool isXStepState =
        alignState == X_STEP_ROTATE ||
        alignState == X_STEP_CREEP ||
        alignState == X_STEP_STOP ||
        alignState == X_STEP_EVALUATE;

    if (isXStepState && fabs(tag.yNorm) > Y_SAFE_LIMIT) {
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        alignState = CENTER_Y_SAFE;
        lastRobotCommand = "CENTER_Y_SAFE";
        printAlignBrief(0.0f, 0.0f);
        return;
    }

    if (alignState == X_STEP_ROTATE) {
        float targetThetaDeg = xStepDirection * X_STEP_THETA_DEG;
        float thetaError = targetThetaDeg - tag.thetaDeg;

        if (fabs(thetaError) > THETA_TOL_DEG) {
            v = 0.0f;
            w = K_X_STEP_YAW * thetaError;
            w = clampFloat(w, -W_X_STEP_MAX, W_X_STEP_MAX);

            lastRobotCommand = "X_STEP_ROTATE";
            sendAlignVelocity(v, w);
            printAlignBrief(v, w);
            return;
        }

        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        stateStartMs = now;
        alignState = X_STEP_CREEP;
        lastRobotCommand = "X_STEP_STOP";
        printAlignBrief(0.0f, 0.0f);
        return;
    }

    if (alignState == X_STEP_CREEP) {
        if (now - stateStartMs < X_STEP_CREEP_MS) {
            v = V_X_STEP;
            w = 0.0f;

            lastRobotCommand = "X_STEP_CREEP";
            sendAlignVelocity(v, w);
            printAlignBrief(v, w);
            return;
        }

        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        stateStartMs = now;
        alignState = X_STEP_STOP;
        lastRobotCommand = "X_STEP_STOP";
        printAlignBrief(0.0f, 0.0f);
        return;
    }

    if (alignState == X_STEP_STOP) {
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        lastRobotCommand = "X_STEP_STOP";

        if (now - stateStartMs >= X_STEP_SETTLE_MS) {
            alignState = X_STEP_EVALUATE;
        }

        printAlignBrief(0.0f, 0.0f);
        return;
    }

    if (alignState == X_STEP_EVALUATE) {
        float xImprovement = fabs(xBeforeStep) - fabs(tag.xNorm);

        if (xImprovement < 0.01f) {
            xStepDirection = -xStepDirection;
        }

        alignState = WAIT_TAG;
        lastRobotCommand = "X_STEP_EVALUATE";
        printAlignBrief(0.0f, 0.0f);
    }

    if (fabs(thetaControl) > ROUGH_THETA_TOL_DEG) {
        alignState = ROUGH_YAW;

        v = 0.0f;
        w = K_THETA * thetaControl;
        w = clampFloat(w, -W_ALIGN_MAX, W_ALIGN_MAX);

        if (w > 0.0f) {
            lastRobotCommand = "ROUGH_YAW_CW";
        } else {
            lastRobotCommand = "ROUGH_YAW_CCW";
        }
        sendAlignVelocity(v, w);
        printAlignBrief(v, w);
        return;
    }

    if (fabs(tag.yNorm) > Y_SAFE_LIMIT) {
        alignState = CENTER_Y_SAFE;

        v = KY * yControl;
        v = clampFloat(v, -V_ALIGN_MAX, V_ALIGN_MAX);
        w = 0.0f;

        lastRobotCommand = "CENTER_Y_SAFE";
        sendAlignVelocity(v, w);
        printAlignBrief(v, w);
        return;
    }

    if (fabs(xControl) > X_TOL_NORM) {
        xBeforeStep = tag.xNorm;
        yBeforeStep = tag.yNorm;

        if (!xStepDirectionValid) {
            if (xControl > 0.0f) {
                xStepDirection = 1;
            } else {
                xStepDirection = -1;
            }
            xStepDirectionValid = true;
        }

        stateStartMs = now;
        alignState = X_STEP_ROTATE;
        lastRobotCommand = "X_STEP_ROTATE";
        printAlignBrief(0.0f, 0.0f);
        return;
    }

    if (fabs(yControl) > Y_TOL_NORM) {
        alignState = FINAL_Y;

        v = KY * yControl;
        v = clampFloat(v, -V_ALIGN_MAX, V_ALIGN_MAX);
        w = 0.0f;

        lastRobotCommand = "FINAL_Y";
        sendAlignVelocity(v, w);
        printAlignBrief(v, w);
        return;
    }

    if (fabs(thetaControl) > THETA_TOL_DEG) {
        alignState = FINAL_YAW;

        v = 0.0f;
        w = K_THETA_FINAL * thetaControl;
        w = clampFloat(w, -W_FINAL_MAX, W_FINAL_MAX);

        lastRobotCommand = "FINAL_YAW";
        sendAlignVelocity(v, w);
        printAlignBrief(v, w);
        return;
    }

    xStepDirectionValid = false;
    alignState = ALIGNED_STOP;
    lastRobotCommand = "ALIGN_STOP";

    drive.stop();
    lastAlignV = 999.0f;
    lastAlignW = 999.0f;

    printAlignBrief(0.0f, 0.0f);
}

// =====================================================
// Printing
// =====================================================

void printHelp() {
    Serial.println();
    Serial.println("=== AGV FIREBEETLE ESP32-E MOTOR + APRILTAG ALIGN TEST ===");
    Serial.println();
    Serial.println("Manual robot commands:");
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
    Serial.println("AprilTag alignment commands from Raspberry Pi:");
    Serial.println("  ALIGN ON");
    Serial.println("  ALIGN OFF");
    Serial.println("  TAG <id> <x_norm> <y_norm> <theta_deg>");
    Serial.println("  TAG LOST");
    Serial.println();
    Serial.println("Other:");
    Serial.println("  S         -> stop and disable alignment");
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

    Serial.print("Align enabled: ");
    Serial.println(alignEnabled ? "YES" : "NO");

    Serial.print("Align state: ");
    Serial.println(alignStateName(alignState));

    Serial.print("Tag visible: ");
    Serial.println(tag.visible ? "YES" : "NO");

    Serial.print("Tag near edge: ");
    Serial.println(tag.nearEdge ? "YES" : "NO");

    Serial.print("Tag id: ");
    Serial.println(tag.id);

    Serial.print("xNorm: ");
    Serial.println(tag.xNorm, 4);

    Serial.print("yNorm: ");
    Serial.println(tag.yNorm, 4);

    Serial.print("thetaDeg: ");
    Serial.println(tag.thetaDeg, 2);

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
// TAG command parser
// =====================================================

bool handleTagCommand(const String& cmd) {
    if (cmd == "TAG LOST") {
        tag.visible = false;
        tag.nearEdge = false;

        if (alignEnabled) {
            drive.stop();
            lastAlignV = 999.0f;
            lastAlignW = 999.0f;
            resetXStepState();
            alignState = LOST_TAG;
            lastRobotCommand = "ALIGN_STOP";
        }

        Serial.println("TAG LOST RECEIVED");
        return true;
    }

    char buffer[90];
    cmd.toCharArray(buffer, sizeof(buffer));

    char word[12] = {0};
    int id = -1;
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    int count = sscanf(buffer, "%11s %d %f %f %f", word, &id, &x, &y, &theta);

    String first = String(word);

    if (first == "TAG" && count == 5) {
        tag.visible = true;
        tag.id = id;
        tag.xNorm = x;
        tag.yNorm = y;
        tag.thetaDeg = theta;
        tag.lastUpdateMs = millis();
        tag.nearEdge = isTagNearEdge();

        // Do not print every TAG here.
        // updateAlignmentController() prints slow brief logs.
        return true;
    }

    return false;
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

    // TAG commands should be handled before normal command parsing.
    if (handleTagCommand(cmd)) {
        return;
    }

    if (cmd == "HELP") {
        printHelp();
        return;
    }

    if (cmd == "S" || cmd == "STOP") {
        alignEnabled = false;
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        alignState = WAIT_TAG;
        lastRobotCommand = "STOP";
        Serial.println("STOP + ALIGN DISABLED");
        printStatus();
        return;
    }

    if (cmd == "STATUS") {
        printStatus();
        return;
    }

    if (cmd == "ALIGN ON") {
        alignEnabled = true;
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        alignState = WAIT_TAG;
        lastRobotCommand = "ALIGN_STOP";
        Serial.println("ALIGN ENABLED");
        return;
    }

    if (cmd == "ALIGN OFF") {
        alignEnabled = false;
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        alignState = WAIT_TAG;
        lastRobotCommand = "STOP";
        Serial.println("ALIGN DISABLED + STOP");
        return;
    }

    // Any manual movement command should disable alignment.
    // This avoids fighting between manual command and auto-align.
    char buffer[60];
    cmd.toCharArray(buffer, sizeof(buffer));

    char p1[12] = {0};
    char p2[12] = {0};
    char p3[12] = {0};

    int count = sscanf(buffer, "%11s %11s %11s", p1, p2, p3);

    String a = String(p1);
    String b = String(p2);

    float value2 = DEFAULT_TEST_RPM;
    if (count >= 2) {
        value2 = atof(p2);
    }

    float value3 = DEFAULT_TEST_RPM;
    if (count >= 3) {
        value3 = atof(p3);
    }

    // ------------------------------
    // Robot-level commands
    // ------------------------------

    if (a == "F") {
        alignEnabled = false;
        float rpm = limitRpm(value2);

        drive.driveForwardRPM(rpm);
        lastRobotCommand = "FORWARD";

        Serial.print("FORWARD ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "B") {
        alignEnabled = false;
        float rpm = limitRpm(value2);

        drive.driveBackwardRPM(rpm);
        lastRobotCommand = "BACKWARD";

        Serial.print("BACKWARD ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "CW") {
        alignEnabled = false;
        float rpm = limitRpm(value2);

        drive.rotateClockwiseRPM(rpm);
        lastRobotCommand = "CLOCKWISE";

        Serial.print("CLOCKWISE ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "CCW") {
        alignEnabled = false;
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
        alignEnabled = false;
        float rpm = limitRpm(value3);

        leftMotor.setDirection(true);
        leftMotor.setFrequencyHz(rpmToHz(rpm));
        lastRobotCommand = "LEFT_ONLY";

        Serial.print("LEFT INTERNAL FORWARD TEST ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "L" && b == "B") {
        alignEnabled = false;
        float rpm = limitRpm(value3);

        leftMotor.setDirection(false);
        leftMotor.setFrequencyHz(rpmToHz(rpm));
        lastRobotCommand = "LEFT_ONLY";

        Serial.print("LEFT INTERNAL BACKWARD TEST ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "R" && b == "F") {
        alignEnabled = false;
        float rpm = limitRpm(value3);

        rightMotor.setDirection(true);
        rightMotor.setFrequencyHz(rpmToHz(rpm));
        lastRobotCommand = "RIGHT_ONLY";

        Serial.print("RIGHT INTERNAL FORWARD TEST ");
        Serial.print(rpm);
        Serial.println(" RPM");
    }
    else if (a == "R" && b == "B") {
        alignEnabled = false;
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

    else if (a == "W") {
        alignEnabled = false;
        float w = limitSignedValue(value2, 2.0f);

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

    Serial.println("AprilTag yaw-only alignment:");
    Serial.println("  Pi sends: TAG <id> <x_norm> <y_norm> <theta_deg>");
    Serial.println("  Enable:  ALIGN ON");
    Serial.println("  Disable: ALIGN OFF");
    Serial.println("  Lost:    TAG LOST");
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

    updateAlignmentController();
}
