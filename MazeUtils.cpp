/**
 * @file MazeUtils.cpp
 * @brief Implementation of utility functions and helper classes.
 */

#include "MazeUtils.h"
#include "Config.h"

namespace MazeUtils {

    void setupPins() {
        pinMode(PIN_BLUE_LED, OUTPUT);
        digitalWrite(PIN_BLUE_LED, LOW);
        
        pinMode(PIN_BUZZER, OUTPUT);
        digitalWrite(PIN_BUZZER, LOW);
    }

    void debugPrint(const __FlashStringHelper* label, float value) {
        #if DEBUG
        Serial.print(label);
        Serial.print(F(": "));
        Serial.println(value);
        #endif
    }

    void debugPrint(const __FlashStringHelper* label, const char* value) {
        #if DEBUG
        Serial.print(label);
        Serial.print(F(": "));
        Serial.println(value);
        #endif
    }

    void debugPrint(const __FlashStringHelper* label, const __FlashStringHelper* value) {
        #if DEBUG
        Serial.print(label);
        Serial.print(F(": "));
        Serial.println(value);
        #endif
    }

    void debugPrint(const __FlashStringHelper* label, int value) {
        #if DEBUG
        Serial.print(label);
        Serial.print(F(": "));
        Serial.println(value);
        #endif
    }

    void debugPrint(const __FlashStringHelper* message) {
        #if DEBUG
        Serial.println(message);
        #endif
    }

    // ============================================================================
    // Timer Class Implementation
    // ============================================================================
    Timer::Timer() : startTime(0), duration(0), active(false) {}

    void Timer::start(unsigned long durationMs) {
        startTime = millis();
        duration = durationMs;
        active = true;
    }

    bool Timer::isExpired() const {
        if (!active) return false;
        return (millis() - startTime >= duration);
    }

    void Timer::stop() {
        active = false;
    }

    bool Timer::isActive() const {
        return active;
    }

    // ============================================================================
    // LedBlinker Class Implementation
    // ============================================================================
    LedBlinker::LedBlinker() : targetBlinks(0), currentBlinks(0), interval(0), ledState(false), running(false) {}

    void LedBlinker::startBlinking(int count, unsigned long intervalMs) {
        targetBlinks = count;
        currentBlinks = 0;
        interval = intervalMs;
        ledState = true;
        running = true;
        
        digitalWrite(PIN_BLUE_LED, HIGH);
        blinkTimer.start(interval);
    }

    void LedBlinker::update() {
        if (!running) return;

        if (blinkTimer.isExpired()) {
            ledState = !ledState;
            digitalWrite(PIN_BLUE_LED, ledState ? HIGH : LOW);

            // If we transitioned to LOW, that marks the completion of one half-cycle
            if (!ledState) {
                currentBlinks++;
                if (currentBlinks >= targetBlinks) {
                    running = false;
                    blinkTimer.stop();
                    digitalWrite(PIN_BLUE_LED, LOW);
                    debugPrint(F("[UTILS] LED blinking complete."));
                } else {
                    blinkTimer.start(interval);
                }
            } else {
                blinkTimer.start(interval);
            }
        }
    }

    bool LedBlinker::isFinished() const {
        return !running;
    }

    // ============================================================================
    // BuzzerControl Class Implementation
    // ============================================================================
    BuzzerControl::BuzzerControl() : running(false) {}

    void BuzzerControl::trigger(unsigned long durationMs) {
        running = true;
        digitalWrite(PIN_BUZZER, HIGH);
        buzzerTimer.start(durationMs);
        debugPrint(F("[UTILS] Buzzer activated."));
    }

    void BuzzerControl::update() {
        if (!running) return;

        if (buzzerTimer.isExpired()) {
            digitalWrite(PIN_BUZZER, LOW);
            running = false;
            buzzerTimer.stop();
            debugPrint(F("[UTILS] Buzzer deactivated."));
        }
    }

    bool BuzzerControl::isFinished() const {
        return !running;
    }

} // namespace MazeUtils
