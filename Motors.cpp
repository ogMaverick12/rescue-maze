/**
 * @file Motors.cpp
 * @brief Implementation of the MotorController class.
 */

#include "Motors.h"
#include "Config.h"
#include "MazeUtils.h"

MotorController::MotorController() 
    : m1(1), m2(2), m3(3), m4(4), // Initialize motors M1, M2, M3, M4
      targetLeftSpeed(0), targetRightSpeed(0), 
      currentLeftSpeed(0), currentRightSpeed(0), 
      lastRampTime(0) {}

void MotorController::begin() {
    // Set initial speed to 0 and release all motors
    m1.setSpeed(0);
    m2.setSpeed(0);
    m3.setSpeed(0);
    m4.setSpeed(0);
    
    m1.run(RELEASE);
    m2.run(RELEASE);
    m3.run(RELEASE);
    m4.run(RELEASE);

    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    lastRampTime = millis();
    MazeUtils::debugPrint(F("[MOTOR] Initialized AFMotor L293D Shield."));
}

void MotorController::Forward(int targetSpeed) {
    targetLeftSpeed = targetSpeed;
    targetRightSpeed = targetSpeed;
}

void MotorController::Backward(int targetSpeed) {
    targetLeftSpeed = -targetSpeed;
    targetRightSpeed = -targetSpeed;
}

void MotorController::Stop() {
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    m1.run(RELEASE);
    m2.run(RELEASE);
    m3.run(RELEASE);
    m4.run(RELEASE);
}

void MotorController::Brake() {
    // Active braking: Stop immediately, bypass ramping, apply release
    Stop();
    // In addition, if we want to actively lock the L293D H-Bridge,
    // we can apply a reverse pulse, but standard release is usually safest for this shield.
    delay(20); // Very short delay for stabilizing
}

void MotorController::TurnLeft90() {
    // 1. Stop for stability before turn (40ms)
    Stop();
    delay(40);

    // 2. True Spin Left: Left side backward, Right side forward
    targetLeftSpeed = -SPEED_TURN;
    targetRightSpeed = SPEED_TURN;
    currentLeftSpeed = -SPEED_TURN;
    currentRightSpeed = SPEED_TURN;
    applySpeeds(currentLeftSpeed, currentRightSpeed);
}

void MotorController::TurnRight90() {
    // 1. Stop for stability before turn (40ms)
    Stop();
    delay(40);

    // 2. True Spin Right: Left side forward, Right side backward
    targetLeftSpeed = SPEED_TURN;
    targetRightSpeed = -SPEED_TURN;
    currentLeftSpeed = SPEED_TURN;
    currentRightSpeed = -SPEED_TURN;
    applySpeeds(currentLeftSpeed, currentRightSpeed);
}

void MotorController::TurnAround() {
    // 1. Stop for stability before turn (40ms)
    Stop();
    delay(40);

    // 2. True Spin 180 degrees (spin right)
    targetLeftSpeed = SPEED_TURN;
    targetRightSpeed = -SPEED_TURN;
    currentLeftSpeed = SPEED_TURN;
    currentRightSpeed = -SPEED_TURN;
    applySpeeds(currentLeftSpeed, currentRightSpeed);
}

void MotorController::SoftLeft(int baseSpeed, float bias) {
    // Left side slows down by bias percentage
    int leftSpeed = baseSpeed * (1.0f - bias);
    if (leftSpeed < SPEED_MIN) leftSpeed = SPEED_MIN;
    targetLeftSpeed = leftSpeed;
    targetRightSpeed = baseSpeed;
}

void MotorController::SoftRight(int baseSpeed, float bias) {
    // Right side slows down by bias percentage
    int rightSpeed = baseSpeed * (1.0f - bias);
    if (rightSpeed < SPEED_MIN) rightSpeed = SPEED_MIN;
    targetLeftSpeed = baseSpeed;
    targetRightSpeed = rightSpeed;
}

void MotorController::setDifferentialSpeeds(int leftSpeed, int rightSpeed) {
    targetLeftSpeed = leftSpeed;
    targetRightSpeed = rightSpeed;
}

void MotorController::update() {
    unsigned long now = millis();
    if (now - lastRampTime >= RAMP_DELAY_MS) {
        lastRampTime = now;
        bool speedChanged = false;

        // Ramp Left Speed
        if (currentLeftSpeed < targetLeftSpeed) {
            currentLeftSpeed += 45; // Acceleration step size
            if (currentLeftSpeed > targetLeftSpeed) currentLeftSpeed = targetLeftSpeed;
            speedChanged = true;
        } else if (currentLeftSpeed > targetLeftSpeed) {
            currentLeftSpeed -= 60; // Deceleration step size (decelerate faster for safety)
            if (currentLeftSpeed < targetLeftSpeed) currentLeftSpeed = targetLeftSpeed;
            speedChanged = true;
        }

        // Ramp Right Speed
        if (currentRightSpeed < targetRightSpeed) {
            currentRightSpeed += 45;
            if (currentRightSpeed > targetRightSpeed) currentRightSpeed = targetRightSpeed;
            speedChanged = true;
        } else if (currentRightSpeed > targetRightSpeed) {
            currentRightSpeed -= 60;
            if (currentRightSpeed < targetRightSpeed) currentRightSpeed = targetRightSpeed;
            speedChanged = true;
        }

        if (speedChanged) {
            applySpeeds(currentLeftSpeed, currentRightSpeed);
        }
    }
}

void MotorController::applySpeeds(int left, int right) {
    // Map absolute values to setSpeed (0-255)
    int leftSpeedAbs = abs(left);
    int rightSpeedAbs = abs(right);

    // Constrain within valid PWM limits
    leftSpeedAbs = constrain(leftSpeedAbs, 0, 255);
    rightSpeedAbs = constrain(rightSpeedAbs, 0, 255);

    // Write speed registers
    m1.setSpeed(leftSpeedAbs);  // Left 1
    m2.setSpeed(leftSpeedAbs);  // Left 2
    m3.setSpeed(rightSpeedAbs); // Right 1
    m4.setSpeed(rightSpeedAbs); // Right 2

    // Determine direction for Left Side (M1 = Port 1, M2 = Port 2)
    if (left > 0) {
        m1.run(FORWARD);
        m2.run(FORWARD);
    } else if (left < 0) {
        m1.run(BACKWARD);
        m2.run(BACKWARD);
    } else {
        m1.run(RELEASE);
        m2.run(RELEASE);
    }

    // Determine direction for Right Side (M3 = Port 3, M4 = Port 4)
    if (right > 0) {
        m3.run(FORWARD);
        m4.run(FORWARD);
    } else if (right < 0) {
        m3.run(BACKWARD);
        m4.run(BACKWARD);
    } else {
        m3.run(RELEASE);
        m4.run(RELEASE);
    }
}
