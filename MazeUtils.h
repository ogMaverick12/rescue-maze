/**
 * @file MazeUtils.h
 * @brief Header for utility functions and helper classes (e.g. non-blocking timers, debug printers).
 */

#ifndef MAZEUTILS_H
#define MAZEUTILS_H

#include <Arduino.h>

namespace MazeUtils {

    /**
     * @brief Setup utility pins (LED and Buzzer).
     */
    void setupPins();

    /**
     * @brief Conditionally print debug info to the Serial monitor.
     * Only outputs if DEBUG is defined as true in Config.h.
     */
    void debugPrint(const __FlashStringHelper* label, float value);
    void debugPrint(const __FlashStringHelper* label, const char* value);
    void debugPrint(const __FlashStringHelper* label, const __FlashStringHelper* value);
    void debugPrint(const __FlashStringHelper* label, int value);
    void debugPrint(const __FlashStringHelper* message);

    /**
     * @brief A simple non-blocking timer class using millis().
     */
    class Timer {
    public:
        Timer();
        
        /**
         * @brief Starts the timer with a given duration in milliseconds.
         */
        void start(unsigned long durationMs);

        /**
         * @brief Checks if the timer has expired.
         * @return true if duration has passed since starting, false otherwise.
         */
        bool isExpired() const;

        /**
         * @brief Stops the timer.
         */
        void stop();

        /**
         * @brief Checks if the timer is currently active.
         */
        bool isActive() const;

    private:
        unsigned long startTime;
        unsigned long duration;
        bool active;
    };

    /**
     * @brief Non-blocking handler for blinking the blue LED.
     */
    class LedBlinker {
    public:
        LedBlinker();

        /**
         * @brief Start blinking the LED a specific number of times.
         * @param count Number of blinks.
         * @param intervalMs Duration of half a blink cycle (on or off).
         */
        void startBlinking(int count, unsigned long intervalMs);

        /**
         * @brief Must be called continuously in the loop to drive the blinking state machine.
         */
        void update();

        /**
         * @brief Check if blinking is completed.
         */
        bool isFinished() const;

    private:
        Timer blinkTimer;
        int targetBlinks;
        int currentBlinks;
        unsigned long interval;
        bool ledState;
        bool running;
    };

    /**
     * @brief Non-blocking handler for the buzzer.
     */
    class BuzzerControl {
    public:
        BuzzerControl();

        /**
         * @brief Turn buzzer ON for a specified duration in milliseconds.
         */
        void trigger(unsigned long durationMs);

        /**
         * @brief Must be called continuously in the loop to drive the buzzer.
         */
        void update();

        /**
         * @brief Check if the buzzer has finished sounding.
         */
        bool isFinished() const;

    private:
        Timer buzzerTimer;
        bool running;
    };

} // namespace MazeUtils

#endif // MAZEUTILS_H
