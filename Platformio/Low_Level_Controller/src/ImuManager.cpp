#include "ImuManager.h"

ImuManager::ImuManager()
    : _mpuAddr(MPU6050_ADDR_LOW),
      _state(IMU_BOOT),
      _lastUpdateUs(0),
      _calibrationStartMs(0),
      _gyroOffsetX(0.0f),
      _gyroOffsetY(0.0f),
      _gyroOffsetZ(0.0f),
      _relativeHeadingDeg(0.0f),
      _headingCandidateDeg(0.0f),
      _headingStableCount(0),
      _headingCandidateValid(false),
      _rollDeg(0.0f),
      _pitchDeg(0.0f),
      _yawDeg(0.0f),
      _q0(1.0f),
      _q1(0.0f),
      _q2(0.0f),
      _q3(0.0f) {}

bool ImuManager::begin() {
    _state = IMU_BOOT;
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    delay(250);

    if (!initMpu6050()) {
        _state = IMU_ERROR;
        return false;
    }

    _state = IMU_CALIBRATING;
    if (!calibrateGyro() || !initializeOrientation()) {
        _state = IMU_ERROR;
        return false;
    }

    _lastUpdateUs = micros();
    _calibrationStartMs = millis();
    _headingCandidateValid = false;
    _headingStableCount = 0;
    _relativeHeadingDeg = 0.0f;
    return true;
}

bool ImuManager::recalibrate() {
    if (_state == IMU_ERROR || _state == IMU_BOOT) {
        return false;
    }

    _state = IMU_CALIBRATING;
    _relativeHeadingDeg = 0.0f;
    _headingCandidateDeg = 0.0f;
    _headingCandidateValid = false;
    _headingStableCount = 0;

    if (!calibrateGyro() || !initializeOrientation()) {
        _state = IMU_ERROR;
        return false;
    }

    getEulerDegrees();
    _relativeHeadingDeg = 0.0f;
    _lastUpdateUs = micros();
    _calibrationStartMs = millis();
    _state = IMU_READY;
    return true;
}

void ImuManager::update() {
    if (_state == IMU_ERROR || _state == IMU_BOOT) {
        return;
    }

    uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - _lastUpdateUs) < SAMPLE_INTERVAL_US) {
        return;
    }

    float dt = (nowUs - _lastUpdateUs) / 1000000.0f;
    _lastUpdateUs = nowUs;

    MpuRaw raw = {};
    if (!readMpuRaw(raw)) {
        return;
    }

    float ax = raw.ax / ACCEL_SCALE;
    float ay = raw.ay / ACCEL_SCALE;
    float az = raw.az / ACCEL_SCALE;
    float gxDegPerSec = (raw.gx - _gyroOffsetX) / GYRO_SCALE;
    float gyDegPerSec = (raw.gy - _gyroOffsetY) / GYRO_SCALE;
    float gzDegPerSec = (raw.gz - _gyroOffsetZ) / GYRO_SCALE;

    updateMadgwickImu(
        gxDegPerSec * DEG_TO_RAD_F,
        gyDegPerSec * DEG_TO_RAD_F,
        gzDegPerSec * DEG_TO_RAD_F,
        ax, ay, az, dt
    );
    getEulerDegrees();

    if (_state == IMU_CALIBRATING) {
        updateStartupReference(_yawDeg);
        return;
    }

    _relativeHeadingDeg = wrapAngleDeg(
        _relativeHeadingDeg + gzDegPerSec * dt * GYRO_HEADING_SIGN
    );
}

bool ImuManager::isReady() const { return _state == IMU_READY; }
ImuState ImuManager::getState() const { return _state; }

const char* ImuManager::getStateText() const {
    switch (_state) {
        case IMU_BOOT: return "BOOT";
        case IMU_CALIBRATING: return "CALIBRATING";
        case IMU_READY: return "READY";
        case IMU_ERROR: return "ERROR";
        default: return "ERROR";
    }
}

ImuOrientation ImuManager::getOrientation() const {
    return {_rollDeg, _pitchDeg, _yawDeg, _relativeHeadingDeg};
}

float ImuManager::getHeadingDeg() const { return _relativeHeadingDeg; }

void ImuManager::resetHeadingZero() {
    _relativeHeadingDeg = 0.0f;
    _lastUpdateUs = micros();
}

float ImuManager::getGyroOffsetX() const { return _gyroOffsetX; }
float ImuManager::getGyroOffsetY() const { return _gyroOffsetY; }
float ImuManager::getGyroOffsetZ() const { return _gyroOffsetZ; }

bool ImuManager::findMpu6050() {
    if (devicePresent(MPU6050_ADDR_LOW)) {
        _mpuAddr = MPU6050_ADDR_LOW;
        return true;
    }
    if (devicePresent(MPU6050_ADDR_HIGH)) {
        _mpuAddr = MPU6050_ADDR_HIGH;
        return true;
    }
    return false;
}

bool ImuManager::initMpu6050() {
    if (!findMpu6050()) return false;
    if (!writeByte(_mpuAddr, 0x6B, 0x00)) return false;
    delay(100);
    writeByte(_mpuAddr, 0x6B, 0x01);
    writeByte(_mpuAddr, 0x6A, 0x00);
    writeByte(_mpuAddr, 0x37, 0x02);
    delay(10);
    writeByte(_mpuAddr, 0x19, 0x09);
    writeByte(_mpuAddr, 0x1A, 0x03);
    writeByte(_mpuAddr, 0x1B, 0x00);
    writeByte(_mpuAddr, 0x1C, 0x00);
    return true;
}

bool ImuManager::readMpuRaw(MpuRaw& raw) {
    uint8_t data[14];
    if (!readBytes(_mpuAddr, 0x3B, data, sizeof(data))) return false;
    raw.ax = readS16BE(&data[0]);
    raw.ay = readS16BE(&data[2]);
    raw.az = readS16BE(&data[4]);
    raw.temp = readS16BE(&data[6]);
    raw.gx = readS16BE(&data[8]);
    raw.gy = readS16BE(&data[10]);
    raw.gz = readS16BE(&data[12]);
    return true;
}

bool ImuManager::calibrateGyro() {
    constexpr uint16_t WARMUP_SAMPLES = 200;
    constexpr uint16_t CALIBRATION_SAMPLES = 1000;
    int32_t sumX = 0;
    int32_t sumY = 0;
    int32_t sumZ = 0;
    uint16_t goodSamples = 0;

    for (uint16_t i = 0; i < WARMUP_SAMPLES; ++i) {
        MpuRaw raw = {};
        readMpuRaw(raw);
        delay(3);
    }

    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ++i) {
        MpuRaw raw = {};
        if (readMpuRaw(raw)) {
            sumX += raw.gx;
            sumY += raw.gy;
            sumZ += raw.gz;
            ++goodSamples;
        }
        delay(3);
    }

    if (goodSamples == 0) return false;
    _gyroOffsetX = (float)sumX / goodSamples;
    _gyroOffsetY = (float)sumY / goodSamples;
    _gyroOffsetZ = (float)sumZ / goodSamples;
    return true;
}

bool ImuManager::initializeOrientation() {
    MpuRaw raw = {};
    if (!readMpuRaw(raw)) return false;
    float ax = raw.ax / ACCEL_SCALE;
    float ay = raw.ay / ACCEL_SCALE;
    float az = raw.az / ACCEL_SCALE;
    setQuaternionFromEuler(
        atan2(ay, az),
        atan2(-ax, sqrt(ay * ay + az * az)),
        0.0f
    );
    return true;
}

void ImuManager::updateStartupReference(float yawDeg) {
    if (millis() - _calibrationStartMs < STARTUP_STABILIZATION_MS) return;
    if (!_headingCandidateValid) {
        _headingCandidateDeg = yawDeg;
        _headingCandidateValid = true;
        _headingStableCount = 0;
        return;
    }
    if (fabs(angleDiffDeg(yawDeg, _headingCandidateDeg)) <= HEADING_STABLE_DEG) {
        ++_headingStableCount;
    } else {
        _headingCandidateDeg = yawDeg;
        _headingStableCount = 0;
    }
    if (_headingStableCount >= HEADING_STABLE_SAMPLES) {
        _relativeHeadingDeg = 0.0f;
        _state = IMU_READY;
    }
}

void ImuManager::updateMadgwickImu(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt
) {
    if (ax == 0.0f && ay == 0.0f && az == 0.0f) return;
    float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    if (recipNorm == 0.0f) return;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    float q0 = _q0;
    float q1 = _q1;
    float q2 = _q2;
    float q3 = _q3;
    float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
    float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
    float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;
    float j11 = 2.0f * q2;
    float j12 = 2.0f * q3;
    float j13 = 2.0f * q0;
    float j14 = 2.0f * q1;
    float j32 = 4.0f * q1;
    float j33 = 4.0f * q2;
    float s0 = j14 * f2 - j11 * f1;
    float s1 = j12 * f1 + j13 * f2 - j32 * f3;
    float s2 = j12 * f2 - j33 * f3 - j13 * f1;
    float s3 = j14 * f1 + j11 * f2;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (recipNorm == 0.0f) return;
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - MADGWICK_BETA * s0;
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - MADGWICK_BETA * s1;
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - MADGWICK_BETA * s2;
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - MADGWICK_BETA * s3;
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (recipNorm == 0.0f) return;
    _q0 = q0 * recipNorm;
    _q1 = q1 * recipNorm;
    _q2 = q2 * recipNorm;
    _q3 = q3 * recipNorm;
}

void ImuManager::getEulerDegrees() {
    _rollDeg = atan2(
        2.0f * (_q0 * _q1 + _q2 * _q3),
        1.0f - 2.0f * (_q1 * _q1 + _q2 * _q2)
    ) * RAD_TO_DEG_F;
    float sinPitch = 2.0f * (_q0 * _q2 - _q3 * _q1);
    sinPitch = constrain(sinPitch, -1.0f, 1.0f);
    _pitchDeg = asin(sinPitch) * RAD_TO_DEG_F;
    _yawDeg = atan2(
        2.0f * (_q0 * _q3 + _q1 * _q2),
        1.0f - 2.0f * (_q2 * _q2 + _q3 * _q3)
    ) * RAD_TO_DEG_F;
    if (_yawDeg < 0.0f) _yawDeg += 360.0f;
}

void ImuManager::setQuaternionFromEuler(float roll, float pitch, float yaw) {
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    _q0 = cr * cp * cy + sr * sp * sy;
    _q1 = sr * cp * cy - cr * sp * sy;
    _q2 = cr * sp * cy + sr * cp * sy;
    _q3 = cr * cp * sy - sr * sp * cy;
    float recipNorm = invSqrt(_q0 * _q0 + _q1 * _q1 + _q2 * _q2 + _q3 * _q3);
    if (recipNorm == 0.0f) {
        _q0 = 1.0f;
        _q1 = _q2 = _q3 = 0.0f;
        return;
    }
    _q0 *= recipNorm;
    _q1 *= recipNorm;
    _q2 *= recipNorm;
    _q3 *= recipNorm;
}

bool ImuManager::writeByte(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool ImuManager::readBytes(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    uint8_t received = Wire.requestFrom((int)addr, (int)len);
    if (received != len) return false;
    for (uint8_t i = 0; i < len; ++i) data[i] = Wire.read();
    return true;
}

bool ImuManager::devicePresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

int16_t ImuManager::readS16BE(const uint8_t* data) const {
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

float ImuManager::invSqrt(float value) const {
    return value > 0.0f ? 1.0f / sqrt(value) : 0.0f;
}

float ImuManager::angleDiffDeg(float current, float reference) const {
    return wrapAngleDeg(current - reference);
}

float ImuManager::wrapAngleDeg(float angle) const {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
