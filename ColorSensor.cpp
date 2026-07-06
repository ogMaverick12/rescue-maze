/**
 * @file ColorSensor.cpp
 * @brief Implementation of the ColorSensor class using the Adafruit TCS34725 library.
 */

#include "ColorSensor.h"
#include "Config.h"
#include "MazeUtils.h"

ColorSensor::ColorSensor() 
    : tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X),
      isInitialized(false),
      red(0), green(0), blue(0), clear(0), lux(0), colorTemp(0),
      blueDetected(false), blackDetected(false) {}

bool ColorSensor::begin() {
    #if !USE_COLOR_SENSOR
    isInitialized = false;
    MazeUtils::debugPrint(F("[COLOR] Bypassed: Disabled in Config.h."));
    return false;
    #endif

    // Enable I2C timeout to prevent lockup if sensor is missing or miswired
    Wire.begin();
    #if defined(ARDUINO_ARCH_AVR)
    Wire.setWireTimeout(3000, true); // Timeout in microseconds
    #endif

    // Start communication with TCS34725
    if (tcs.begin()) {
        isInitialized = true;
        MazeUtils::debugPrint(F("[COLOR] TCS34725 Initialized."));
        return true;
    } else {
        isInitialized = false;
        MazeUtils::debugPrint(F("[COLOR] ERROR: TCS34725 not found!"));
        return false;
    }
}

void ColorSensor::update() {
    #if !USE_COLOR_SENSOR
    return;
    #endif

    if (!isInitialized) {
        static unsigned long lastRetryTime = 0;
        unsigned long now = millis();
        if (now - lastRetryTime >= 5000) { // Limit retry attempts to once every 5 seconds
            lastRetryTime = now;
            if (!begin()) {
                return;
            }
        } else {
            return;
        }
    }

    // Read raw light values
    tcs.getRawData(&red, &green, &blue, &clear);
    
    // Calculate light temperature and lux
    if (clear > 0) {
        lux = tcs.calculateLux(red, green, blue);
        colorTemp = tcs.calculateColorTemperature(red, green, blue);
    } else {
        lux = 0;
        colorTemp = 0;
    }

    classifyColor();
}

void ColorSensor::classifyColor() {
    if (clear == 0) {
        blueDetected = false;
        blackDetected = false; // Do not detect black on hardware fault
        isInitialized = false; // Force re-initialization retry
        return;
    }

    // Avoid division by zero
    float r = red > 0 ? (float)red : 1.0f;
    float g = green > 0 ? (float)green : 1.0f;
    float b = blue > 0 ? (float)blue : 1.0f;

    float blueRatio = b / r;
    float blueToGreen = b / g;

    // Detect Blue Tile
    // The blue tile will have high blue-to-red and blue-to-green ratios.
    // Also, overall clear intensity shouldn't be too low (which would be black).
    if (blueRatio >= COLOR_BLUE_RATIO_MIN && blueToGreen >= 0.90f && clear >= COLOR_BLUE_CLEAR_MIN) {
        blueDetected = true;
        blackDetected = false;
    } 
    // Detect Black Tile
    // Black tile absorbs most light, so clear intensity and lux are extremely low.
    else if (lux < COLOR_BLACK_LUX_MAX && clear <= COLOR_BLACK_CLEAR_MAX) {
        blueDetected = false;
        blackDetected = true;
    } 
    // Standard Floor (White/Grey/Brown Wood etc.)
    else {
        blueDetected = false;
        blackDetected = false;
    }
}

bool ColorSensor::isBlueDetected() const {
    return isInitialized && blueDetected;
}

bool ColorSensor::isBlackDetected() const {
    return isInitialized && blackDetected;
}

void ColorSensor::printColorInfo() const {
    if (!isInitialized) {
        MazeUtils::debugPrint(F("[COLOR] Not Initialized"));
        return;
    }
    
    #if DEBUG
    Serial.print(F("[COLOR] R: ")); Serial.print(red);
    Serial.print(F(" G: ")); Serial.print(green);
    Serial.print(F(" B: ")); Serial.print(blue);
    Serial.print(F(" C: ")); Serial.print(clear);
    Serial.print(F(" | Lux: ")); Serial.print(lux);
    Serial.print(F(" Temp: ")); Serial.print(colorTemp);
    
    if (blueDetected) {
        Serial.println(F(" -> [BLUE TILE]"));
    } else if (blackDetected) {
        Serial.println(F(" -> [BLACK TILE]"));
    } else {
        Serial.println(F(" -> [FLOOR]"));
    }
    #endif
}
