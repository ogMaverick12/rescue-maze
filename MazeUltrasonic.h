/**
 * @file MazeUltrasonic.h
 * @brief Header for the UltrasonicSensor and UltrasonicSystem classes.
 */

#ifndef MAZEULTRASONIC_H
#define MAZEULTRASONIC_H

#include <Arduino.h>

/**
 * @class UltrasonicSensor
 * @brief Represents a single HC-SR04 ultrasonic sensor with raw filtering and moving average.
 */
class UltrasonicSensor {
public:
    UltrasonicSensor(uint8_t triggerPin, uint8_t echoPin);

    /**
     * @brief Configure pins.
     */
    void begin();

    /**
     * @brief Trigger a measurement, filter noise, and update the internal moving average.
     * @return The updated filtered distance in centimeters.
     */
    float read();

    /**
     * @brief Get the last filtered distance in centimeters.
     */
    float getDistance() const;

private:
    uint8_t trigPin;
    uint8_t echoPin;
    float currentDistance;
    
    // Moving average filter definitions
    static const int FILTER_WINDOW_SIZE = 5;
    float readings[FILTER_WINDOW_SIZE];
    int readIndex;
    float runningSum;

    /**
     * @brief Perform physical pulse measurement.
     * @return Raw distance in centimeters. Returns -1.0 if timeout or bad reading.
     */
    float getRawDistance();
};

/**
 * @class UltrasonicSystem
 * @brief Coordinates the three ultrasonic sensors (Front, Left, Right) to evaluate maze environment.
 */
class UltrasonicSystem {
public:
    UltrasonicSystem();

    /**
     * @brief Initialize all sensors.
     */
    void begin();

    /**
     * @brief Read all sensors. Should be called periodically.
     */
    void update();

    // Getters for distances
    float getFrontDistance() const;
    float getLeftDistance() const;
    float getRightDistance() const;

    // Environmental detection functions
    bool isLeftOpen() const;
    bool isRightOpen() const;
    bool isFrontBlocked() const;
    bool isDeadEnd() const;
    bool isWallLost() const;

    /**
     * @brief Utility function to print all distances to Serial (when in DEBUG).
     */
    void printDistances() const;

private:
    UltrasonicSensor frontSensor;
    UltrasonicSensor leftSensor;
    UltrasonicSensor rightSensor;
};

#endif // MAZEULTRASONIC_H
