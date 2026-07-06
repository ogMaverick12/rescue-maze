/**
 * @file rescue_maze.ino
 * @brief Main entry point for the autonomous Arduino Uno R3 maze-solving robot.
 *
 * This file coordinates setup and loops, delegating the logic to the Navigation class.
 */

#include "Config.h"
#include "Navigation.h"

Navigation robot;

void setup() {
    // Enable Serial communication only if DEBUG mode is active.
    #if DEBUG
    Serial.begin(115200);
    delay(500); // Give Serial monitor time to stabilize
    Serial.println(F("[MAIN] Robot Startup. Initializing systems..."));
    #endif

    // Initialize the robot's navigation state machine and peripherals
    robot.begin();
}

void loop() {
    // Run the main navigation state machine updates
    robot.update();
}