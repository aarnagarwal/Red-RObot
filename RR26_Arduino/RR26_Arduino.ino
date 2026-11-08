/** Red Robot 2026: calibrated line following with PI control. */
#include <Arduino.h>
#include "RedRobot.h"

struct PIState {
    float integral = 0.0f;
};
PIState pi;

// Error is measured in sensor spacings (-2.5 to +2.5).
// Tune Kp with Ki = 0 first, then gradually enable integral correction.
float Kp = 5.0f;
float Ki = 0.5f; // Starting value only; requires track tuning.
float current_speed = 15.0f;
const float MAX_INTEGRAL_CORRECTION = 5.0f;

// Raw discharge times in microseconds, NOT normalized 0..1000 readings.
// PLACEHOLDERS: measure every sensor on the floor and on the line at its
// installed height, then replace these arrays. Either line polarity works.
const float FLOOR_US[6] = {100, 100, 100, 100, 100, 100};
const float LINE_US[6] = {2500, 2500, 2500, 2500, 2500, 2500};
const float MIN_SENSOR_STRENGTH = 0.2f;
const float MIN_TOTAL_STRENGTH = 0.3f;

// Sensor order is defined in RedRobot.h. Verify that positive error steers
// toward the line; change this to -1 if physical sensor/motor orientation
// makes the robot steer away. Motors 2 and 3 must both drive forward at +speed.
const float STEERING_SIGN = 1.0f;

// 1 = button route; 2 = scoring route. Distances/times need track testing.
int auton_state = 1;
const unsigned long FOLLOW_DURATION_MS = 10000UL;
const unsigned long DISPENSE_DURATION_MS = 1000UL;
bool started = false;
bool finished = false;
unsigned long initialTimems = 0;
unsigned long previousSampleUs = 0;
bool havePreviousSample = false;

void resetPI() {
    pi.integral = 0.0f;
    havePreviousSample = false;
}

void stopDrive() {
    RedRobot::WriteDCMotor(2, 0);
    RedRobot::WriteDCMotor(3, 0);
}

void setup() {
    RedRobot::InitRedRobot();
    RedRobot::InitDCMotors();
    RedRobot::InitServo();
    RedRobot::InitLineSensors();
    resetPI();
    RedRobot::InitAuton();
}

bool calculateLineError(const uint16_t sensorValues[6], float &error) {
    float weightedSum = 0.0f;
    float totalStrength = 0.0f;
    for (int i = 0; i < 6; ++i) {
        float span = LINE_US[i] - FLOOR_US[i];
        // Invalid calibration: fail closed instead of dividing by zero.
        if (span == 0.0f) return false;
        float strength = (sensorValues[i] - FLOOR_US[i]) / span;
        strength = constrain(strength, 0.0f, 1.0f);
        if (strength < MIN_SENSOR_STRENGTH) strength = 0.0f;
        weightedSum += strength * (i - 2.5f);
        totalStrength += strength;
    }
    if (totalStrength < MIN_TOTAL_STRENGTH) return false;
    error = STEERING_SIGN * weightedSum / totalStrength;
    return true;
}

float calculatePI(float error, float dtSeconds) {
    float proportional = Kp * error;
    if (Ki <= 0.0f) {
        pi.integral = 0.0f;
        return proportional;
    }

    float integralLimit = MAX_INTEGRAL_CORRECTION / Ki;
    float candidate = pi.integral + error * dtSeconds;
    candidate = constrain(candidate, -integralLimit, integralLimit);
    float correction = proportional + Ki * candidate;

    // For base +/- correction, both motors fit within +/-100 when
    // abs(correction) <= 100 - abs(base). Allow integral to unwind even
    // during saturation, but do not accumulate error that worsens it.
    float base = constrain(current_speed, -100.0f, 100.0f);
    float correctionLimit = 100.0f - fabsf(base);
    bool worsensSaturation =
        (correction > correctionLimit && error > 0.0f) ||
        (correction < -correctionLimit && error < 0.0f);
    if (!worsensSaturation) pi.integral = candidate;
    return proportional + Ki * pi.integral;
}

void line_follow() {
    uint16_t sensorValues[6];
    RedRobot::ReadLineSensor(sensorValues);
    unsigned long sampleUs = micros();
    float error = 0.0f;
    if (!calculateLineError(sensorValues, error)) {
        stopDrive();
        resetPI();
        return; // Do not overwrite the stop with an old correction.
    }

    // Timestamp completed readings; unsigned subtraction tolerates rollover.
    float dtSeconds = havePreviousSample
        ? (sampleUs - previousSampleUs) * 1.0e-6f : 0.0f;
    previousSampleUs = sampleUs;
    havePreviousSample = true;
    // A long interruption should not produce a large integral jump.
    if (dtSeconds > 0.25f) {
        pi.integral = 0.0f;
        dtSeconds = 0.0f;
    }
    float correction = calculatePI(error, dtSeconds);
    float base = constrain(current_speed, -100.0f, 100.0f);
    float leftSpeed = constrain(base + correction, -100.0f, 100.0f);
    float rightSpeed = constrain(base - correction, -100.0f, 100.0f);
    RedRobot::WriteDCMotor(2, (int16_t)roundf(leftSpeed));
    RedRobot::WriteDCMotor(3, (int16_t)roundf(rightSpeed));
}

void AutonomousLoop() {
    if (finished) return;
    if (!started) {
        initialTimems = millis();
        resetPI();
        started = true;
    }
    unsigned long elapsedMs = millis() - initialTimems;
    if (elapsedMs < FOLLOW_DURATION_MS) {
        line_follow();
        return;
    }

    // Stop the drivetrain once the route's timed following phase ends.
    stopDrive();
    resetPI();
    if (auton_state == 2 &&
        elapsedMs - FOLLOW_DURATION_MS < DISPENSE_DURATION_MS) {
        RedRobot::WriteDCMotor(0, -100);
        return;
    }
    RedRobot::WriteDCMotor(0, 0);
    finished = true;
    RedRobot::StartRemoteControl();
}
