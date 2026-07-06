/**
 * @file MazeUltrasonic.cpp
 * @brief Implementation of the UltrasonicSensor and UltrasonicSystem classes.
 */

#include "MazeUltrasonic.h"
#include "Config.h"
#include "MazeUtils.h"

// ============================================================================
// UltrasonicSensor Implementation
// ============================================================================

UltrasonicSensor::UltrasonicSensor(uint8_t triggerPin, uint8_t echoPin)
    : trigPin(triggerPin), echoPin(echoPin), currentDistance(30.0f), readIndex(0), runningSum(0.0f) {
    // Initialize filter readings to a safe mid-range distance (30 cm)
    for (int i = 0; i < FILTER_WINDOW_SIZE; ++i) {
        readings[i] = 30.0f;
    }
    runningSum = 30.0f * FILTER_WINDOW_SIZE;
}

void UltrasonicSensor::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
}

float UltrasonicSensor::read() {
    float raw = getRawDistance();

    // FILTERING LOGIC & BAD READING REJECTION:
    // If pulseIn times out (returns 0) or reads out-of-range, it means either:
    // 1. The wall is too far (open path).
    // 2. Sensor glitch.
    // In either case, we should treat it as an open space (e.g., 100 cm) instead of 0.
    // If it returned 0, treating it as 0 would make the robot think it is colliding and steer away.
    if (raw <= 1.0f || raw > 200.0f) {
        raw = 100.0f; // Treat as clear path/no wall nearby
    }

    // Update moving average filter
    runningSum -= readings[readIndex];
    readings[readIndex] = raw;
    runningSum += readings[readIndex];
    readIndex = (readIndex + 1) % FILTER_WINDOW_SIZE;

    currentDistance = runningSum / FILTER_WINDOW_SIZE;
    return currentDistance;
}

float UltrasonicSensor::getDistance() const {
    return currentDistance;
}

float UltrasonicSensor::getRawDistance() {
    // Ensure trigger is clean
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Send a 10 microsecond HIGH pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read echo pulse duration. Timeout in microseconds (20000 us = ~340 cm max distance)
    unsigned long duration = pulseIn(echoPin, HIGH, 20000);

    if (duration == 0) {
        return -1.0f; // Indicates timeout
    }

    // Distance calculation: duration * speed of sound (0.0343 cm/us) / 2
    return (duration * 0.0343f) / 2.0f;
}

// ============================================================================
// UltrasonicSystem Implementation
// ============================================================================

UltrasonicSystem::UltrasonicSystem()
    : frontSensor(PIN_FRONT_TRIG, PIN_FRONT_ECHO),
      leftSensor(PIN_LEFT_TRIG, PIN_LEFT_ECHO),
      rightSensor(PIN_RIGHT_TRIG, PIN_RIGHT_ECHO) {}

void UltrasonicSystem::begin() {
    frontSensor.begin();
    leftSensor.begin();
    rightSensor.begin();
}

void UltrasonicSystem::update() {
    // Read all three sensors.
    // Note: To avoid sensor cross-talk, we add a delay (e.g. 30ms) between reading different sensors.
    frontSensor.read();
    delay(30);
    leftSensor.read();
    delay(30);
    rightSensor.read();
}

float UltrasonicSystem::getFrontDistance() const {
    return frontSensor.getDistance();
}

float UltrasonicSystem::getLeftDistance() const {
    return leftSensor.getDistance();
}

float UltrasonicSystem::getRightDistance() const {
    return rightSensor.getDistance();
}

bool UltrasonicSystem::isLeftOpen() const {
    return (leftSensor.getDistance() > DIST_WALL_THRESHOLD);
}

bool UltrasonicSystem::isRightOpen() const {
    return (rightSensor.getDistance() > DIST_WALL_THRESHOLD);
}

bool UltrasonicSystem::isFrontBlocked() const {
    return (frontSensor.getDistance() < DIST_FRONT_BLOCKED);
}

bool UltrasonicSystem::isDeadEnd() const {
    return (isFrontBlocked() && !isLeftOpen() && !isRightOpen());
}

bool UltrasonicSystem::isWallLost() const {
    // Wall is lost if both sides are reading very far away
    return (leftSensor.getDistance() > DIST_WALL_LOST && rightSensor.getDistance() > DIST_WALL_LOST);
}

void UltrasonicSystem::printDistances() const {
    MazeUtils::debugPrint(F("[US] Front"), frontSensor.getDistance());
    MazeUtils::debugPrint(F("[US] Left "), leftSensor.getDistance());
    MazeUtils::debugPrint(F("[US] Right"), rightSensor.getDistance());
}
