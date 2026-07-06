/**
 * @file ColorSensor.h
 * @brief Header for the ColorSensor class using Adafruit TCS34725.
 */

#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

/**
 * @class ColorSensor
 * @brief Encapsulates the TCS34725 I2C color sensor and provides classification for Blue and Black tiles.
 */
class ColorSensor {
public:
    ColorSensor();

    /**
     * @brief Initialize the sensor.
     * @return true if communication succeeded, false otherwise.
     */
    bool begin();

    /**
     * @brief Continuously read data from the sensor and update internal color values.
     * Must be called in the main loop.
     */
    void update();

    /**
     * @brief Check if the sensor is currently detecting a Blue tile.
     */
    bool isBlueDetected() const;

    /**
     * @brief Check if the sensor is currently detecting a Black tile.
     */
    bool isBlackDetected() const;

    /**
     * @brief Utility function to print raw and calculated color values (when in DEBUG).
     */
    void printColorInfo() const;

private:
    Adafruit_TCS34725 tcs;
    bool isInitialized;

    // Last read values
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
    uint32_t lux;
    uint16_t colorTemp;

    // Classification state
    bool blueDetected;
    bool blackDetected;

    /**
     * @brief Classifies the raw RGBC values into colors based on Config.h thresholds.
     */
    void classifyColor();
};

#endif // COLORSENSOR_H
