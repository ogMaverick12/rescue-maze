/**
 * @file Navigation.h
 * @brief Header for the Navigation class containing the maze-solving state machine and PID controller.
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "MazeUltrasonic.h"
#include "ColorSensor.h"
#include "MazeMemory.h"
#include "MazeUtils.h"

/**
 * @enum RobotState
 * @brief State definitions for the robot's navigation state machine.
 */
enum RobotState {
    STATE_FORWARD,          // Moving forward, following left wall / centering
    STATE_ALIGN_JUNCTION,   // Driving forward briefly to center the pivot in the junction
    STATE_TURN_LEFT,        // Executing 90-degree left turn
    STATE_TURN_RIGHT,       // Executing 90-degree right turn
    STATE_TURN_AROUND,      // Executing 180-degree turn (dead end)
    STATE_BLUE_ACTION,      // End of maze: flashing blue LED, then initiating return
    STATE_BLACK_ACTION,     // Hazard tile: stopping, sounding buzzer, then continuing
    STATE_STUCK_RECOVERY,   // Executing stuck recovery routine
    STATE_FINISHED          // Safe final state at end of return run (no CPU blocking)
};

/**
 * @class Navigation
 * @brief Coordinates sensors, motors, and memory to autonomously navigate the maze.
 */
class Navigation {
public:
    Navigation();

    /**
     * @brief Initialize all components.
     */
    void begin();

    /**
     * @brief Main navigation logic called in the loop.
     */
    void update();

    /**
     * @brief Check if the robot is stuck.
     */
    void checkStuckCondition();

    /**
     * @brief Expose current state for debugging.
     */
    RobotState getCurrentState() const;
    const char* stateToString(RobotState state) const;

private:
    // Core hardware instances
    MotorController motors;
    UltrasonicSystem sonar;
    ColorSensor colorSensor;
    MazeMemory memory;

    // Navigation FSM State
    RobotState currentState;
    RobotState nextStateAfterAction; // Store next state during timed actions

    // PID Variables
    float pidError;
    float pidLastError;
    float pidIntegral;
    unsigned long pidLastTime;

    // Non-blocking Timers
    MazeUtils::Timer stateTimer;
    MazeUtils::Timer stuckTimer;
    MazeUtils::Timer junctionCooldownTimer;

    // Blinker & Buzzer
    MazeUtils::LedBlinker ledBlinker;
    MazeUtils::BuzzerControl buzzer;

    // Junction latching
    bool junctionDetected;
    bool latchLeftOpen;
    bool latchRightOpen;
    bool latchFrontBlocked;
    int junctionConfirmCount;
    int blueConfirmCount;
    int blackConfirmCount;

    // Tile Ignore / Lockout Cooldowns
    MazeUtils::Timer tileIgnoreTimer;
    bool tileIgnoreActive;
    int blackActionStep;

    // Stuck check variables
    float lastFrontDist;
    float lastLeftDist;
    float lastRightDist;
    unsigned long lastStuckCheckTime;

    // Helper functions
    void runForwardState();
    void runAlignJunctionState();
    void runTurnState();
    void runBlueActionState();
    void runBlackActionState();
    void runStuckRecoveryState();
    void runFinishedState();

    /**
     * @brief Computes PID steering correction using ultrasonic sensors.
     * @return Speed correction factor to apply to motors.
     */
    float computePID();

    /**
     * @brief Check if we are at an intersection.
     */
    bool checkJunctionTrigger();
};

#endif // NAVIGATION_H
