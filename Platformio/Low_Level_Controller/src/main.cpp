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

struct PoseData {
    bool visible = false;
    int id = -1;
    float xM = 0.0f;
    float yM = 0.0f;
    float yawDeg = 0.0f;
    unsigned long lastUpdateMs = 0;
};

TagData tag;
PoseData pose;

enum AlignInputMode {
    ALIGN_INPUT_PIXEL,
    ALIGN_INPUT_POSE,
    ALIGN_INPUT_POSE_TRACK,
    ALIGN_INPUT_POSE_GEOMETRIC,
    ALIGN_INPUT_POSE_PRIMITIVE,
    ALIGN_INPUT_POSE_TRIAL
};

AlignInputMode alignInputMode = ALIGN_INPUT_PIXEL;

bool alignEnabled = false;

enum NavState {
    NAV_IDLE,
    NAV_START_ALIGN,
    NAV_CAPTURE_HEADING,
    NAV_CRUISE,
    NAV_DONE,
    NAV_ERROR
};

bool navEnabled = false;
NavState navState = NAV_IDLE;
int currentTagId = 0;
int expectedNextTagId = 1;
float navTargetHeadingDeg = 0.0f;

enum PoseGeoState {
    PG_OBSERVE,
    PG_TURN,
    PG_CREEP,
    PG_SETTLE,
    PG_FINAL_YAW,
    PG_ALIGNED,
    PG_LOST,
    PG_UNSAFE
};

PoseGeoState pgState = PG_OBSERVE;
unsigned long pgStateStartMs = 0;
float pgTurnDeg = 0.0f;
float pgMoveM = 0.0f;
float pgMoveDir = 1.0f;
unsigned long pgTurnMs = 0;
unsigned long pgCreepMs = 0;
float pgTargetAngleDeg = 0.0f;
unsigned long lastPoseGeoPrintMs = 0;

enum PosePrimitiveState {
    PP_SELECT,
    PP_EXECUTE,
    PP_SETTLE,
    PP_ALIGNED,
    PP_LOST,
    PP_UNSAFE
};

struct MotionPrimitive {
    float v;
    float w;
    const char* name;
};

MotionPrimitive primitives[] = {
    {            0.0f,             0.0f, "STOP" },
    {     POSE_PRIM_V,             0.0f, "FWD" },
    {    -POSE_PRIM_V,             0.0f, "BACK" },
    {     POSE_PRIM_V,      POSE_PRIM_W, "FWD_CW" },
    {     POSE_PRIM_V,     -POSE_PRIM_W, "FWD_CCW" },
    {    -POSE_PRIM_V,      POSE_PRIM_W, "BACK_CW" },
    {    -POSE_PRIM_V,     -POSE_PRIM_W, "BACK_CCW" },
    {            0.0f,      POSE_PRIM_W, "ROT_CW" },
    {            0.0f,     -POSE_PRIM_W, "ROT_CCW" }
};

PosePrimitiveState ppState = PP_SELECT;
unsigned long ppStateStartMs = 0;
float ppSelectedV = 0.0f;
float ppSelectedW = 0.0f;
const char* ppSelectedName = "NONE";
float ppSelectedCost = 0.0f;
unsigned long lastPosePrimitivePrintMs = 0;

enum PoseTrialState {
    PT_OBSERVE,
    PT_EXECUTE,
    PT_SETTLE,
    PT_EVALUATE,
    PT_FINAL_YAW,
    PT_ALIGNED,
    PT_LOST,
    PT_UNSAFE
};

struct TrialPrimitive {
    float v;
    float w;
    const char* name;
};

TrialPrimitive trialPrimitives[] = {
    {     POSE_TRIAL_V,             0.0f, "FWD" },
    {    -POSE_TRIAL_V,             0.0f, "BACK" },
    {     POSE_TRIAL_V,      POSE_TRIAL_W, "FWD_CW" },
    {     POSE_TRIAL_V,     -POSE_TRIAL_W, "FWD_CCW" },
    {    -POSE_TRIAL_V,      POSE_TRIAL_W, "BACK_CW" },
    {    -POSE_TRIAL_V,     -POSE_TRIAL_W, "BACK_CCW" },
    {            0.0f,      POSE_TRIAL_W, "ROT_CW" },
    {            0.0f,     -POSE_TRIAL_W, "ROT_CCW" }
};

PoseTrialState ptState = PT_OBSERVE;
unsigned long ptStateStartMs = 0;

float ptBeforeCost = 0.0f;
float ptBeforeX = 0.0f;
float ptBeforeY = 0.0f;
float ptBeforeYaw = 0.0f;
float ptNewCost = 0.0f;

int ptPrimitiveIndex = 0;
float ptSelectedV = 0.0f;
float ptSelectedW = 0.0f;
const char* ptSelectedName = "NONE";
unsigned long lastPoseTrialPrintMs = 0;

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

float poseXBeforeStepM = 0.0f;
float poseYBeforeStepM = 0.0f;
int poseXStepDirection = 1;
bool poseXStepDirectionValid = false;
unsigned long poseStateStartMs = 0;

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

float signFloat(float value) {
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
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

bool isPoseTimedOut() {
    return pose.visible && (millis() - pose.lastUpdateMs > TAG_TIMEOUT_MS);
}

const char* alignInputModeName() {
    switch (alignInputMode) {
        case ALIGN_INPUT_PIXEL: return "PIXEL";
        case ALIGN_INPUT_POSE: return "POSE";
        case ALIGN_INPUT_POSE_TRACK: return "TRACK";
        case ALIGN_INPUT_POSE_GEOMETRIC: return "GEOMETRIC";
        case ALIGN_INPUT_POSE_PRIMITIVE: return "PRIMITIVE";
        case ALIGN_INPUT_POSE_TRIAL: return "TRIAL";
        default: return "UNKNOWN";
    }
}

const char* navStateName(NavState state) {
    switch (state) {
        case NAV_IDLE: return "NAV_IDLE";
        case NAV_START_ALIGN: return "NAV_START_ALIGN";
        case NAV_CAPTURE_HEADING: return "NAV_CAPTURE_HEADING";
        case NAV_CRUISE: return "NAV_CRUISE";
        case NAV_DONE: return "NAV_DONE";
        case NAV_ERROR: return "NAV_ERROR";
        default: return "UNKNOWN";
    }
}

const char* poseGeoStateName(PoseGeoState state) {
    switch (state) {
        case PG_OBSERVE: return "OBSERVE";
        case PG_TURN: return "TURN";
        case PG_CREEP: return "CREEP";
        case PG_SETTLE: return "SETTLE";
        case PG_FINAL_YAW: return "FINAL_YAW";
        case PG_ALIGNED: return "ALIGNED";
        case PG_LOST: return "LOST";
        case PG_UNSAFE: return "UNSAFE";
        default: return "UNKNOWN";
    }
}

const char* posePrimitiveStateName(PosePrimitiveState state) {
    switch (state) {
        case PP_SELECT: return "SELECT";
        case PP_EXECUTE: return "EXECUTE";
        case PP_SETTLE: return "SETTLE";
        case PP_ALIGNED: return "ALIGNED";
        case PP_LOST: return "LOST";
        case PP_UNSAFE: return "UNSAFE";
        default: return "UNKNOWN";
    }
}

const char* poseTrialStateName(PoseTrialState state) {
    switch (state) {
        case PT_OBSERVE: return "OBSERVE";
        case PT_EXECUTE: return "EXECUTE";
        case PT_SETTLE: return "SETTLE";
        case PT_EVALUATE: return "EVALUATE";
        case PT_FINAL_YAW: return "FINAL_YAW";
        case PT_ALIGNED: return "ALIGNED";
        case PT_LOST: return "LOST";
        case PT_UNSAFE: return "UNSAFE";
        default: return "UNKNOWN";
    }
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

void resetPoseXStepState() {
    poseXBeforeStepM = 0.0f;
    poseYBeforeStepM = 0.0f;
    poseXStepDirection = 1;
    poseXStepDirectionValid = false;
    poseStateStartMs = 0;
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

void updatePixelAlignmentController() {
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


void printPoseAlignBrief(float v, float w) {
    unsigned long now = millis();

    if (now - lastAlignPrintMs < 300) {
        return;
    }

    lastAlignPrintMs = now;

    Serial.print("POSE_ALIGN ");
    Serial.print(alignStateName(alignState));
    Serial.print(" id=");
    Serial.print(pose.id);
    Serial.print(" xM=");
    Serial.print(pose.xM, 4);
    Serial.print(" yM=");
    Serial.print(pose.yM, 4);
    Serial.print(" yaw=");
    Serial.print(pose.yawDeg, 2);
    Serial.print(" xBeforeM=");
    Serial.print(poseXBeforeStepM, 4);
    Serial.print(" yBeforeM=");
    Serial.print(poseYBeforeStepM, 4);
    Serial.print(" xStepDir=");
    Serial.print(poseXStepDirection);
    Serial.print(" v=");
    Serial.print(v, 4);
    Serial.print(" w=");
    Serial.println(w, 4);
}

void stopPoseAlignmentAsLost(const String& command) {
    drive.stop();
    lastAlignV = 999.0f;
    lastAlignW = 999.0f;
    resetPoseXStepState();
    alignState = LOST_TAG;
    lastRobotCommand = command;
    printPoseAlignBrief(0.0f, 0.0f);
}

void updatePoseAlignmentController() {
    if (!alignEnabled) {
        return;
    }

    unsigned long now = millis();

    if (now - lastAlignControlMs < ALIGN_CONTROL_PERIOD_MS) {
        return;
    }

    lastAlignControlMs = now;

    if (!pose.visible || isPoseTimedOut()) {
        stopPoseAlignmentAsLost("ALIGN_STOP");
        return;
    }

    if (fabs(pose.xM) > POSE_LOCAL_UNSAFE_M || fabs(pose.yM) > POSE_LOCAL_UNSAFE_M) {
        stopPoseAlignmentAsLost("HARD_EDGE_STOP");
        return;
    }

    float v = 0.0f;
    float w = 0.0f;

    float yawControl = THETA_SIGN * pose.yawDeg;
    float xControl = X_SIGN * pose.xM;
    float yControl = Y_SIGN * pose.yM;

    bool isXStepState =
        alignState == X_STEP_ROTATE ||
        alignState == X_STEP_CREEP ||
        alignState == X_STEP_STOP ||
        alignState == X_STEP_EVALUATE;

    if (isXStepState && fabs(pose.yM) > POSE_Y_SAFE_M) {
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        alignState = CENTER_Y_SAFE;
        lastRobotCommand = "CENTER_Y_SAFE";
        printPoseAlignBrief(0.0f, 0.0f);
        return;
    }

    if (alignState == X_STEP_ROTATE) {
        float targetThetaDeg = poseXStepDirection * POSE_X_STEP_THETA_DEG;
        float thetaError = targetThetaDeg - pose.yawDeg;

        if (fabs(thetaError) > POSE_YAW_TOL_DEG) {
            v = 0.0f;
            w = K_POSE_X_STEP_YAW * thetaError;
            w = clampFloat(w, -W_POSE_X_STEP_MAX, W_POSE_X_STEP_MAX);

            lastRobotCommand = "X_STEP_ROTATE";
            sendAlignVelocity(v, w);
            printPoseAlignBrief(v, w);
            return;
        }

        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        poseStateStartMs = now;
        alignState = X_STEP_CREEP;
        lastRobotCommand = "X_STEP_STOP";
        printPoseAlignBrief(0.0f, 0.0f);
        return;
    }

    if (alignState == X_STEP_CREEP) {
        if (now - poseStateStartMs < POSE_X_STEP_CREEP_MS) {
            v = V_POSE_X_STEP;
            w = 0.0f;

            lastRobotCommand = "X_STEP_CREEP";
            sendAlignVelocity(v, w);
            printPoseAlignBrief(v, w);
            return;
        }

        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        poseStateStartMs = now;
        alignState = X_STEP_STOP;
        lastRobotCommand = "X_STEP_STOP";
        printPoseAlignBrief(0.0f, 0.0f);
        return;
    }

    if (alignState == X_STEP_STOP) {
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        lastRobotCommand = "X_STEP_STOP";

        if (now - poseStateStartMs >= POSE_X_STEP_SETTLE_MS) {
            alignState = X_STEP_EVALUATE;
        }

        printPoseAlignBrief(0.0f, 0.0f);
        return;
    }

    if (alignState == X_STEP_EVALUATE) {
        bool improved = fabs(pose.xM) < fabs(poseXBeforeStepM) - POSE_X_STEP_IMPROVE_M;

        if (!improved) {
            poseXStepDirection = -poseXStepDirection;
        }

        alignState = WAIT_TAG;
        lastRobotCommand = "X_STEP_EVALUATE";
        printPoseAlignBrief(0.0f, 0.0f);
    }

    if (fabs(yawControl) > POSE_ROUGH_YAW_TOL_DEG) {
        alignState = ROUGH_YAW;

        v = 0.0f;
        w = K_POSE_YAW * yawControl;
        w = clampFloat(w, -W_POSE_MAX, W_POSE_MAX);

        if (w > 0.0f) {
            lastRobotCommand = "ROUGH_YAW_CW";
        } else {
            lastRobotCommand = "ROUGH_YAW_CCW";
        }
        sendAlignVelocity(v, w);
        printPoseAlignBrief(v, w);
        return;
    }

    if (fabs(pose.yM) > POSE_Y_SAFE_M) {
        alignState = CENTER_Y_SAFE;

        v = K_POSE_Y * yControl;
        v = clampFloat(v, -V_POSE_MAX, V_POSE_MAX);
        w = 0.0f;

        lastRobotCommand = "CENTER_Y_SAFE";
        sendAlignVelocity(v, w);
        printPoseAlignBrief(v, w);
        return;
    }

    if (fabs(pose.xM) > POSE_X_TOL_M) {
        poseXBeforeStepM = pose.xM;
        poseYBeforeStepM = pose.yM;

        if (!poseXStepDirectionValid) {
            if (xControl > 0.0f) {
                poseXStepDirection = 1;
            } else {
                poseXStepDirection = -1;
            }
            poseXStepDirectionValid = true;
        }

        poseStateStartMs = now;
        alignState = X_STEP_ROTATE;
        lastRobotCommand = "X_STEP_ROTATE";
        printPoseAlignBrief(0.0f, 0.0f);
        return;
    }

    if (fabs(pose.yM) > POSE_Y_TOL_M) {
        alignState = FINAL_Y;

        v = K_POSE_Y * yControl;
        v = clampFloat(v, -V_POSE_MAX, V_POSE_MAX);
        w = 0.0f;

        lastRobotCommand = "FINAL_Y";
        sendAlignVelocity(v, w);
        printPoseAlignBrief(v, w);
        return;
    }

    if (fabs(yawControl) > POSE_YAW_TOL_DEG) {
        alignState = FINAL_YAW;

        v = 0.0f;
        w = K_POSE_YAW_FINAL * yawControl;
        w = clampFloat(w, -W_POSE_FINAL_MAX, W_POSE_FINAL_MAX);

        lastRobotCommand = "FINAL_YAW";
        sendAlignVelocity(v, w);
        printPoseAlignBrief(v, w);
        return;
    }

    poseXStepDirectionValid = false;
    alignState = ALIGNED_STOP;
    lastRobotCommand = "ALIGN_STOP";

    drive.stop();
    lastAlignV = 999.0f;
    lastAlignW = 999.0f;

    printPoseAlignBrief(0.0f, 0.0f);
}

void updatePoseTrackController() {
    if (!alignEnabled) {
        drive.stop();
        return;
    }

    if (!pose.visible || isPoseTimedOut()) {
        drive.stop();
        Serial.println("POSE_TRACK LOST_OR_TIMEOUT");
        return;
    }

    float x = pose.xM;
    float y = pose.yM;
    float yaw = pose.yawDeg;

    float absX = fabs(x);
    float absY = fabs(y);
    float absYaw = fabs(yaw);

    // Hard safety only. Tracking itself blends all three pose errors.
    if (absX > POSE_TRACK_UNSAFE_X_M ||
        absY > POSE_TRACK_UNSAFE_Y_M ||
        absYaw > POSE_TRACK_UNSAFE_YAW_DEG) {
        drive.stop();
        Serial.println("POSE_TRACK UNSAFE_OUT_OF_RANGE");
        return;
    }

    if (absX < POSE_TRACK_XY_TOL_M &&
        absY < POSE_TRACK_XY_TOL_M &&
        absYaw < POSE_TRACK_YAW_TOL_DEG) {
        drive.stop();
        Serial.print("POSE_TRACK ALIGNED xM=");
        Serial.print(x, 4);
        Serial.print(" yM=");
        Serial.print(y, 4);
        Serial.print(" yaw=");
        Serial.println(yaw, 2);
        return;
    }

    float xToYawDeg = atan2(x, POSE_TRACK_LOOKAHEAD_M) * 180.0f / PI;
    float yawCmdDeg = THETA_SIGN * yaw + xToYawDeg;

    float v = K_POSE_TRACK_Y * Y_SIGN * y;
    float w = K_POSE_TRACK_W * yawCmdDeg;

    v = clampFloat(v, -V_POSE_TRACK_MAX, V_POSE_TRACK_MAX);
    w = clampFloat(w, -W_POSE_TRACK_MAX, W_POSE_TRACK_MAX);

    drive.setRobotVelocity(v, w);

    Serial.print("POSE_TRACK xM=");
    Serial.print(x, 4);
    Serial.print(" yM=");
    Serial.print(y, 4);
    Serial.print(" yaw=");
    Serial.print(yaw, 2);
    Serial.print(" xToYaw=");
    Serial.print(xToYawDeg, 2);
    Serial.print(" yawCmd=");
    Serial.print(yawCmdDeg, 2);
    Serial.print(" v=");
    Serial.print(v, 4);
    Serial.print(" w=");
    Serial.println(w, 4);
}

void printPoseGeoEvent(const char* event) {
    unsigned long now = millis();
    if (now - lastPoseGeoPrintMs < 200) {
        return;
    }
    lastPoseGeoPrintMs = now;

    Serial.print("POSE_GEO ");
    Serial.println(event);
}

void printPoseGeoBrief(
    float x,
    float y,
    float yaw,
    float dist,
    float v,
    float w
) {
    unsigned long now = millis();
    if (now - lastPoseGeoPrintMs < 200) {
        return;
    }
    lastPoseGeoPrintMs = now;

    Serial.print("POSE_GEO state=");
    Serial.print(poseGeoStateName(pgState));
    Serial.print(" xM=");
    Serial.print(x, 4);
    Serial.print(" yM=");
    Serial.print(y, 4);
    Serial.print(" yaw=");
    Serial.print(yaw, 2);
    Serial.print(" dist=");
    Serial.print(dist, 4);
    Serial.print(" targetAngle=");
    Serial.print(pgTargetAngleDeg, 2);
    Serial.print(" turnDeg=");
    Serial.print(pgTurnDeg, 2);
    Serial.print(" moveM=");
    Serial.print(pgMoveM, 4);
    Serial.print(" turnMs=");
    Serial.print(pgTurnMs);
    Serial.print(" creepMs=");
    Serial.print(pgCreepMs);
    Serial.print(" v=");
    Serial.print(v, 4);
    Serial.print(" w=");
    Serial.println(w, 4);
}

void updatePoseGeometricController() {
    if (!alignEnabled) {
        drive.stop();
        pgState = PG_OBSERVE;
        return;
    }

    float x = pose.xM;
    float y = pose.yM;
    float yaw = pose.yawDeg;
    float dist = sqrt(x * x + y * y);

    if (!pose.visible || isPoseTimedOut()) {
        drive.stop();
        pgState = PG_LOST;
        printPoseGeoEvent("LOST_OR_TIMEOUT");
        return;
    }

    float absX = fabs(x);
    float absY = fabs(y);
    float absYaw = fabs(yaw);

    if (absX > POSE_GEO_UNSAFE_X_M ||
        absY > POSE_GEO_UNSAFE_Y_M ||
        absYaw > POSE_GEO_UNSAFE_YAW_DEG) {
        drive.stop();
        pgState = PG_UNSAFE;
        printPoseGeoEvent("UNSAFE_OUT_OF_RANGE");
        return;
    }

    unsigned long now = millis();
    float v = 0.0f;
    float w = 0.0f;

    switch (pgState) {
        case PG_OBSERVE:
            drive.stop();

            if (absX < POSE_GEO_XY_TOL_M && absY < POSE_GEO_XY_TOL_M) {
                pgState = absYaw < POSE_GEO_YAW_TOL_DEG ? PG_ALIGNED : PG_FINAL_YAW;
                pgStateStartMs = now;
                break;
            }

            pgTargetAngleDeg = atan2(x, y) * 180.0f / PI;
            pgTurnDeg = clampFloat(
                pgTargetAngleDeg,
                -POSE_GEO_MAX_TURN_DEG,
                POSE_GEO_MAX_TURN_DEG
            );

            if (fabs(pgTurnDeg) < POSE_GEO_MIN_TURN_DEG &&
                absX > POSE_GEO_XY_TOL_M) {
                float turnSign = signFloat(pgTurnDeg == 0.0f ? x : pgTurnDeg);
                pgTurnDeg = turnSign * POSE_GEO_MIN_TURN_DEG;
            }

            pgMoveM = clampFloat(dist, 0.0f, POSE_GEO_MAX_STEP_DIST_M);
            pgMoveDir = y >= 0.0f ? 1.0f : -1.0f;
            pgTurnMs = constrain(
                (unsigned long)(fabs(pgTurnDeg) * 25.0f),
                60UL,
                220UL
            );
            pgCreepMs = constrain(
                (unsigned long)((pgMoveM / POSE_GEO_CREEP_V) * 1000.0f),
                60UL,
                180UL
            );
            pgState = PG_TURN;
            pgStateStartMs = now;
            break;

        case PG_TURN:
            v = 0.0f;
            w = POSE_GEO_TURN_KW * pgTurnDeg;
            w = clampFloat(w, -POSE_GEO_TURN_W_MAX, POSE_GEO_TURN_W_MAX);
            drive.setRobotVelocity(v, w);

            if (now - pgStateStartMs >= pgTurnMs) {
                drive.stop();
                v = 0.0f;
                w = 0.0f;
                pgState = PG_CREEP;
                pgStateStartMs = now;
            }
            break;

        case PG_CREEP:
            v = pgMoveDir * POSE_GEO_CREEP_V;
            w = 0.0f;
            drive.setRobotVelocity(v, w);

            if (now - pgStateStartMs >= pgCreepMs) {
                drive.stop();
                v = 0.0f;
                pgState = PG_SETTLE;
                pgStateStartMs = now;
            }
            break;

        case PG_SETTLE:
            drive.stop();
            if (now - pgStateStartMs >= POSE_GEO_SETTLE_MS) {
                pgState = PG_OBSERVE;
                pgStateStartMs = now;
            }
            break;

        case PG_FINAL_YAW:
            if (absX > POSE_GEO_XY_TOL_M || absY > POSE_GEO_XY_TOL_M) {
                drive.stop();
                pgState = PG_OBSERVE;
                pgStateStartMs = now;
                break;
            }

            if (absYaw < POSE_GEO_YAW_TOL_DEG) {
                drive.stop();
                pgState = PG_OBSERVE;
                pgStateStartMs = now;
                break;
            }

            v = 0.0f;
            w = POSE_GEO_FINAL_YAW_KW * THETA_SIGN * yaw;
            w = clampFloat(w, -POSE_GEO_FINAL_YAW_W_MAX, POSE_GEO_FINAL_YAW_W_MAX);
            drive.setRobotVelocity(v, w);
            break;

        case PG_ALIGNED:
            drive.stop();
            if (absX >= POSE_GEO_XY_TOL_M ||
                absY >= POSE_GEO_XY_TOL_M ||
                absYaw >= POSE_GEO_YAW_TOL_DEG) {
                pgState = PG_OBSERVE;
                pgStateStartMs = now;
            } else {
                printPoseGeoEvent("ALIGNED");
            }
            break;

        case PG_LOST:
        case PG_UNSAFE:
            drive.stop();
            break;
    }

    printPoseGeoBrief(x, y, yaw, dist, v, w);
}

void predictMotionPrimitive(
    float x,
    float y,
    float yawDeg,
    const MotionPrimitive& primitive,
    float& xPred,
    float& yPred,
    float& yawPred
) {
    yawPred = yawDeg - (primitive.w * POSE_PRIM_DT_SEC * 180.0f / PI);
    yPred = y - (primitive.v * POSE_PRIM_DT_SEC);
    xPred = x - (primitive.w * POSE_PRIM_DT_SEC * 0.08f);
}

float motionPrimitiveCost(float xPred, float yPred, float yawPred) {
    float edgePenalty = 0.0f;

    if (fabs(xPred) > POSE_PRIM_EDGE_X_M) {
        edgePenalty += fabs(xPred) - POSE_PRIM_EDGE_X_M;
    }
    if (fabs(yPred) > POSE_PRIM_EDGE_Y_M) {
        edgePenalty += fabs(yPred) - POSE_PRIM_EDGE_Y_M;
    }

    return
        POSE_PRIM_WX * fabs(xPred) +
        POSE_PRIM_WY * fabs(yPred) +
        POSE_PRIM_WYAW * fabs(yawPred) +
        POSE_PRIM_WEDGE * edgePenalty;
}

void printPosePrimitiveBrief(float x, float y, float yaw) {
    unsigned long now = millis();
    if (now - lastPosePrimitivePrintMs < 200) {
        return;
    }
    lastPosePrimitivePrintMs = now;

    Serial.print("POSE_PRIM state=");
    Serial.print(posePrimitiveStateName(ppState));
    Serial.print(" xM=");
    Serial.print(x, 4);
    Serial.print(" yM=");
    Serial.print(y, 4);
    Serial.print(" yaw=");
    Serial.print(yaw, 2);
    Serial.print(" selected=");
    Serial.print(ppSelectedName);
    Serial.print(" v=");
    Serial.print(ppSelectedV, 4);
    Serial.print(" w=");
    Serial.print(ppSelectedW, 4);
    Serial.print(" cost=");
    Serial.println(ppSelectedCost, 4);
}

void updatePosePrimitiveController() {
    if (!alignEnabled) {
        drive.stop();
        ppState = PP_SELECT;
        return;
    }

    float x = pose.xM;
    float y = pose.yM;
    float yaw = pose.yawDeg;

    if (!pose.visible || isPoseTimedOut()) {
        drive.stop();
        ppState = PP_LOST;
        printPosePrimitiveBrief(x, y, yaw);
        return;
    }

    float absX = fabs(x);
    float absY = fabs(y);
    float absYaw = fabs(yaw);

    if (absX > POSE_PRIM_UNSAFE_X_M ||
        absY > POSE_PRIM_UNSAFE_Y_M ||
        absYaw > POSE_PRIM_UNSAFE_YAW_DEG) {
        drive.stop();
        ppState = PP_UNSAFE;
        printPosePrimitiveBrief(x, y, yaw);
        return;
    }

    unsigned long now = millis();

    switch (ppState) {
        case PP_SELECT: {
            drive.stop();

            if (absX < POSE_PRIM_XY_TOL_M &&
                absY < POSE_PRIM_XY_TOL_M &&
                absYaw < POSE_PRIM_YAW_TOL_DEG) {
                ppState = PP_ALIGNED;
                ppSelectedV = 0.0f;
                ppSelectedW = 0.0f;
                ppSelectedName = "STOP";
                ppSelectedCost = 0.0f;
                break;
            }

            float bestCost = 1.0e9f;
            const size_t primitiveCount = sizeof(primitives) / sizeof(primitives[0]);

            for (size_t i = 0; i < primitiveCount; ++i) {
                float xPred = 0.0f;
                float yPred = 0.0f;
                float yawPred = 0.0f;
                predictMotionPrimitive(
                    x,
                    y,
                    yaw,
                    primitives[i],
                    xPred,
                    yPred,
                    yawPred
                );
                float cost = motionPrimitiveCost(xPred, yPred, yawPred);

                if (cost < bestCost) {
                    bestCost = cost;
                    ppSelectedV = primitives[i].v;
                    ppSelectedW = primitives[i].w;
                    ppSelectedName = primitives[i].name;
                }
            }

            ppSelectedCost = bestCost;
            ppState = PP_EXECUTE;
            ppStateStartMs = now;
            break;
        }

        case PP_EXECUTE:
            drive.setRobotVelocity(ppSelectedV, ppSelectedW);
            if (now - ppStateStartMs >= POSE_PRIM_EXEC_MS) {
                drive.stop();
                ppState = PP_SETTLE;
                ppStateStartMs = now;
            }
            break;

        case PP_SETTLE:
            drive.stop();
            if (now - ppStateStartMs >= POSE_PRIM_SETTLE_MS) {
                ppState = PP_SELECT;
                ppStateStartMs = now;
            }
            break;

        case PP_ALIGNED:
            drive.stop();
            if (absX >= POSE_PRIM_XY_TOL_M ||
                absY >= POSE_PRIM_XY_TOL_M ||
                absYaw >= POSE_PRIM_YAW_TOL_DEG) {
                ppState = PP_SELECT;
                ppStateStartMs = now;
            }
            break;

        case PP_LOST:
        case PP_UNSAFE:
            drive.stop();
            break;
    }

    printPosePrimitiveBrief(x, y, yaw);
}

float positionCost(float x, float y, float yaw) {
    return
        POSE_TRIAL_WX * fabs(x) +
        POSE_TRIAL_WY * fabs(y) +
        POSE_TRIAL_WYAW_POSITION * fabs(yaw);
}

float finalCost(float x, float y, float yaw) {
    return
        POSE_TRIAL_WX * fabs(x) +
        POSE_TRIAL_WY * fabs(y) +
        POSE_TRIAL_WYAW_FINAL * fabs(yaw);
}

void printPoseTrialBrief(float x, float y, float yaw) {
    unsigned long now = millis();
    if (now - lastPoseTrialPrintMs < 200) {
        return;
    }
    lastPoseTrialPrintMs = now;

    Serial.print("POSE_TRIAL state=");
    Serial.print(poseTrialStateName(ptState));
    Serial.print(" xM=");
    Serial.print(x, 4);
    Serial.print(" yM=");
    Serial.print(y, 4);
    Serial.print(" yaw=");
    Serial.print(yaw, 2);
    Serial.print(" beforeCost=");
    Serial.print(ptBeforeCost, 4);
    Serial.print(" newCost=");
    Serial.print(ptNewCost, 4);
    Serial.print(" selected=");
    Serial.print(ptSelectedName);
    Serial.print(" v=");
    Serial.print(ptSelectedV, 4);
    Serial.print(" w=");
    Serial.print(ptSelectedW, 4);
    Serial.print(" primitiveIndex=");
    Serial.println(ptPrimitiveIndex);
}

void updatePoseTrialController() {
    if (!alignEnabled) {
        drive.stop();
        ptState = PT_OBSERVE;
        return;
    }

    float x = pose.xM;
    float y = pose.yM;
    float yaw = pose.yawDeg;

    if (!pose.visible || isPoseTimedOut()) {
        drive.stop();
        ptState = PT_LOST;
        printPoseTrialBrief(x, y, yaw);
        return;
    }

    float absX = fabs(x);
    float absY = fabs(y);
    float absYaw = fabs(yaw);
    float dist = sqrt(x * x + y * y);

    if (absX > POSE_TRIAL_UNSAFE_X_M ||
        absY > POSE_TRIAL_UNSAFE_Y_M ||
        absYaw > POSE_TRIAL_UNSAFE_YAW_DEG) {
        drive.stop();
        ptState = PT_UNSAFE;
        printPoseTrialBrief(x, y, yaw);
        return;
    }

    unsigned long now = millis();

    switch (ptState) {
        case PT_OBSERVE: {
            drive.stop();

            if (absX < POSE_TRIAL_XY_TOL_M &&
                absY < POSE_TRIAL_XY_TOL_M &&
                absYaw < POSE_TRIAL_YAW_TOL_DEG) {
                ptState = PT_ALIGNED;
                ptSelectedV = 0.0f;
                ptSelectedW = 0.0f;
                ptSelectedName = "STOP";
                break;
            }

            if (dist < POSE_TRIAL_POSITION_READY_M &&
                absYaw >= POSE_TRIAL_YAW_TOL_DEG) {
                ptState = PT_FINAL_YAW;
                ptStateStartMs = now;
                break;
            }

            ptBeforeX = x;
            ptBeforeY = y;
            ptBeforeYaw = yaw;
            ptBeforeCost = positionCost(x, y, yaw);
            ptNewCost = ptBeforeCost;

            const int primitiveCount =
                sizeof(trialPrimitives) / sizeof(trialPrimitives[0]);
            if (ptPrimitiveIndex < 0 || ptPrimitiveIndex >= primitiveCount) {
                ptPrimitiveIndex = 0;
            }

            ptSelectedV = trialPrimitives[ptPrimitiveIndex].v;
            ptSelectedW = trialPrimitives[ptPrimitiveIndex].w;
            ptSelectedName = trialPrimitives[ptPrimitiveIndex].name;
            ptState = PT_EXECUTE;
            ptStateStartMs = now;
            break;
        }

        case PT_EXECUTE:
            drive.setRobotVelocity(ptSelectedV, ptSelectedW);
            if (now - ptStateStartMs >= POSE_TRIAL_EXEC_MS) {
                drive.stop();
                ptState = PT_SETTLE;
                ptStateStartMs = now;
            }
            break;

        case PT_SETTLE:
            drive.stop();
            if (now - ptStateStartMs >= POSE_TRIAL_SETTLE_MS) {
                ptState = PT_EVALUATE;
                ptStateStartMs = now;
            }
            break;

        case PT_EVALUATE: {
            drive.stop();
            ptNewCost = positionCost(x, y, yaw);

            if (ptNewCost >= ptBeforeCost - POSE_TRIAL_MIN_IMPROVE) {
                const int primitiveCount =
                    sizeof(trialPrimitives) / sizeof(trialPrimitives[0]);
                ptPrimitiveIndex = (ptPrimitiveIndex + 1) % primitiveCount;
            }

            ptState = PT_OBSERVE;
            ptStateStartMs = now;
            break;
        }

        case PT_FINAL_YAW:
            if (dist > POSE_TRIAL_POSITION_READY_M + 0.002f) {
                drive.stop();
                ptState = PT_OBSERVE;
                ptStateStartMs = now;
                break;
            }

            if (absYaw < POSE_TRIAL_YAW_TOL_DEG) {
                drive.stop();
                ptState = PT_OBSERVE;
                ptStateStartMs = now;
                break;
            }

            ptSelectedV = 0.0f;
            ptSelectedW = 0.004f * THETA_SIGN * yaw;
            ptSelectedW = clampFloat(ptSelectedW, -0.025f, 0.025f);
            ptSelectedName = "FINAL_YAW";
            ptNewCost = finalCost(x, y, yaw);
            drive.setRobotVelocity(ptSelectedV, ptSelectedW);
            break;

        case PT_ALIGNED:
            drive.stop();
            if (absX >= POSE_TRIAL_XY_TOL_M ||
                absY >= POSE_TRIAL_XY_TOL_M ||
                absYaw >= POSE_TRIAL_YAW_TOL_DEG) {
                ptState = PT_OBSERVE;
                ptStateStartMs = now;
            }
            break;

        case PT_LOST:
        case PT_UNSAFE:
            drive.stop();
            break;
    }

    printPoseTrialBrief(x, y, yaw);
}

void updateAlignmentController() {
    if (alignInputMode == ALIGN_INPUT_POSE_TRIAL) {
        updatePoseTrialController();
        return;
    }

    if (alignInputMode == ALIGN_INPUT_POSE_PRIMITIVE) {
        updatePosePrimitiveController();
        return;
    }

    if (alignInputMode == ALIGN_INPUT_POSE_GEOMETRIC) {
        updatePoseGeometricController();
        return;
    }

    if (alignInputMode == ALIGN_INPUT_POSE_TRACK) {
        updatePoseTrackController();
        return;
    }

    if (alignInputMode == ALIGN_INPUT_POSE) {
        updatePoseAlignmentController();
        return;
    }

    updatePixelAlignmentController();
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
    Serial.println("  ALIGN MODE PIXEL");
    Serial.println("  ALIGN MODE POSE");
    Serial.println("  ALIGN MODE TRACK");
    Serial.println("  ALIGN MODE GEOMETRIC");
    Serial.println("  ALIGN MODE PRIMITIVE");
    Serial.println("  ALIGN MODE TRIAL");
    Serial.println("  TAG <id> <x_norm> <y_norm> <theta_deg>");
    Serial.println("  TAGPOSE <id> <x_m> <y_m> <yaw_deg>");
    Serial.println("  TAG LOST");
    Serial.println("  NAV START");
    Serial.println("  NAV STOP");
    Serial.println("  NAV RESET");
    Serial.println("  NAV STATUS");
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

    Serial.print("Align input mode: ");
    Serial.println(alignInputModeName());

    Serial.print("Navigation enabled: ");
    Serial.println(navEnabled ? "YES" : "NO");

    Serial.print("Navigation state: ");
    Serial.println(navStateName(navState));

    Serial.print("Current tag id: ");
    Serial.println(currentTagId);

    Serial.print("Expected next tag id: ");
    Serial.println(expectedNextTagId);

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

    Serial.print("Pose visible: ");
    Serial.println(pose.visible ? "YES" : "NO");

    Serial.print("Pose id: ");
    Serial.println(pose.id);

    Serial.print("pose xM: ");
    Serial.println(pose.xM, 4);

    Serial.print("pose yM: ");
    Serial.println(pose.yM, 4);

    Serial.print("pose yawDeg: ");
    Serial.println(pose.yawDeg, 2);

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

void printNavStatus() {
    Serial.println("----- NAV STATUS -----");

    Serial.print("navEnabled: ");
    Serial.println(navEnabled ? "YES" : "NO");

    Serial.print("navState: ");
    Serial.println(navStateName(navState));

    Serial.print("currentTagId: ");
    Serial.println(currentTagId);

    Serial.print("expectedNextTagId: ");
    Serial.println(expectedNextTagId);

    Serial.print("navTargetHeadingDeg: ");
    Serial.println(navTargetHeadingDeg, 2);

    Serial.print("poseVisible: ");
    Serial.println(pose.visible ? "YES" : "NO");

    Serial.print("poseId: ");
    Serial.println(pose.id);

    Serial.print("pose xM: ");
    Serial.println(pose.xM, 4);

    Serial.print("pose yM: ");
    Serial.println(pose.yM, 4);

    Serial.print("pose yawDeg: ");
    Serial.println(pose.yawDeg, 2);

    Serial.println("----------------------");
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
        pose.visible = false;

        if (alignEnabled) {
            drive.stop();
            lastAlignV = 999.0f;
            lastAlignW = 999.0f;
            resetXStepState();
            resetPoseXStepState();
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

    if (first == "TAGPOSE" && count == 5) {
        pose.visible = true;
        pose.id = id;
        pose.xM = x;
        pose.yM = y;
        pose.yawDeg = theta;
        pose.lastUpdateMs = millis();

        Serial.print("TAGPOSE RECEIVED id=");
        Serial.print(pose.id);
        Serial.print(" xM=");
        Serial.print(pose.xM, 4);
        Serial.print(" yM=");
        Serial.print(pose.yM, 4);
        Serial.print(" yawDeg=");
        Serial.println(pose.yawDeg, 2);
        return true;
    }

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
        resetPoseXStepState();
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

    if (cmd == "NAV START") {
        navEnabled = true;
        navState = NAV_START_ALIGN;
        currentTagId = 0;
        expectedNextTagId = 1;
        drive.stop();
        Serial.println("NAV STARTED");
        return;
    }

    if (cmd == "NAV STOP") {
        navEnabled = false;
        navState = NAV_IDLE;
        alignEnabled = false;
        drive.stop();
        Serial.println("NAV STOPPED");
        return;
    }

    if (cmd == "NAV RESET") {
        navEnabled = false;
        navState = NAV_IDLE;
        currentTagId = 0;
        expectedNextTagId = 1;
        navTargetHeadingDeg = 0.0f;
        alignEnabled = false;
        drive.stop();
        Serial.println("NAV RESET");
        return;
    }

    if (cmd == "NAV STATUS") {
        printNavStatus();
        return;
    }

    if (cmd == "ALIGN MODE PIXEL") {
        alignInputMode = ALIGN_INPUT_PIXEL;
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        resetPoseXStepState();
        alignState = WAIT_TAG;
        lastRobotCommand = "ALIGN_STOP";
        Serial.println("ALIGN INPUT MODE: PIXEL");
        return;
    }

    if (cmd == "ALIGN MODE POSE") {
        alignInputMode = ALIGN_INPUT_POSE;
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        resetPoseXStepState();
        alignState = WAIT_TAG;
        lastRobotCommand = "ALIGN_STOP";
        Serial.println("ALIGN INPUT MODE: POSE");
        return;
    }

    if (cmd == "ALIGN MODE TRACK") {
        alignInputMode = ALIGN_INPUT_POSE_TRACK;
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        resetPoseXStepState();
        alignState = WAIT_TAG;
        lastRobotCommand = "ALIGN_STOP";
        Serial.println("ALIGN INPUT MODE: TRACK");
        return;
    }

    if (cmd == "ALIGN MODE GEOMETRIC") {
        alignInputMode = ALIGN_INPUT_POSE_GEOMETRIC;
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        resetPoseXStepState();
        pgState = PG_OBSERVE;
        pgStateStartMs = millis();
        pgTurnDeg = 0.0f;
        pgMoveM = 0.0f;
        pgMoveDir = 1.0f;
        pgTurnMs = 0;
        pgCreepMs = 0;
        pgTargetAngleDeg = 0.0f;
        alignState = WAIT_TAG;
        lastRobotCommand = "ALIGN_STOP";
        Serial.println("ALIGN INPUT MODE: GEOMETRIC");
        return;
    }

    if (cmd == "ALIGN MODE PRIMITIVE") {
        alignInputMode = ALIGN_INPUT_POSE_PRIMITIVE;
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        resetPoseXStepState();
        ppState = PP_SELECT;
        ppStateStartMs = millis();
        ppSelectedV = 0.0f;
        ppSelectedW = 0.0f;
        ppSelectedName = "NONE";
        ppSelectedCost = 0.0f;
        alignState = WAIT_TAG;
        lastRobotCommand = "ALIGN_STOP";
        Serial.println("ALIGN INPUT MODE: PRIMITIVE");
        return;
    }

    if (cmd == "ALIGN MODE TRIAL") {
        alignInputMode = ALIGN_INPUT_POSE_TRIAL;
        drive.stop();
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        resetPoseXStepState();
        ptState = PT_OBSERVE;
        ptStateStartMs = millis();
        ptBeforeCost = 0.0f;
        ptBeforeX = 0.0f;
        ptBeforeY = 0.0f;
        ptBeforeYaw = 0.0f;
        ptNewCost = 0.0f;
        ptPrimitiveIndex = 0;
        ptSelectedV = 0.0f;
        ptSelectedW = 0.0f;
        ptSelectedName = "NONE";
        alignState = WAIT_TAG;
        lastRobotCommand = "ALIGN_STOP";
        Serial.println("ALIGN INPUT MODE: TRIAL");
        return;
    }

    if (cmd == "ALIGN ON") {
        alignEnabled = true;
        lastAlignV = 999.0f;
        lastAlignW = 999.0f;
        resetXStepState();
        resetPoseXStepState();
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
        resetPoseXStepState();
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
