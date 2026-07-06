/**
 * @file Config.h
 * @brief Global configuration parameters, pins, thresholds, and options.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// DEBUG MODE
// ============================================================================
/**
 * Set to true to enable Serial debugging output.
 * Set to false to completely disable Serial initialization and output.
 */
#define DEBUG false

// ============================================================================
// PIN CONFIGURATION
// ============================================================================

// Ultrasonic Sensors
#define PIN_FRONT_TRIG  9   // Servo 2 Header (Trigger - from test)
#define PIN_FRONT_ECHO  10  // Servo 1 Header (Echo - from test)

#define PIN_LEFT_TRIG   A1  // Left Trigger (from test)
#define PIN_LEFT_ECHO   A0  // Left Echo (from test)

#define PIN_RIGHT_TRIG  A3  // Right Trigger (from test)
#define PIN_RIGHT_ECHO  A2  // Right Echo (from test)

// Color Sensor (TCS34725)
// Sensor Compilation Flags (Set to false to disable/bypass components for testing)
#define USE_COLOR_SENSOR    true
#define USE_ULTRASONIC      true

// General Outputs
#define PIN_BLUE_LED    2   // Blue LED (User corrected from D9 to D2)
#define PIN_BUZZER      13  // Buzzer (User corrected from D10 to D13)

// ============================================================================
// NAVIGATIONAL THRESHOLDS & PARAMETERS
// ============================================================================

// Distances in centimeters
const float DIST_WALL_THRESHOLD     = 25.0f; // Distance below which a wall is considered present
const float DIST_FRONT_BLOCKED      = 26.0f; // Distance below which the front path is blocked
const float DIST_TARGET_CENTER      = 15.0f; // Target distance from a single wall (walls are ~30cm apart)
const float DIST_WALL_LOST          = 45.0f; // Distance above which we consider the wall completely lost

// Debounce Filter Configuration
const int JUNCTION_CONFIRM_READINGS = 3;     // Number of consecutive readings to confirm a junction

// Timing configuration in milliseconds
const unsigned long DURATION_BLUE_BLINK   = 300;   // Duration for LED on/off during blinks
const int           COUNT_BLUE_BLINK      = 3;     // Deprecated (replaced by full pause logic)
const unsigned long DURATION_BLUE_PAUSE   = 5000;  // 5 seconds pause when blue color is detected
const unsigned long DURATION_BLACK_BUZZER = 2000;  // 2 seconds buzzer sound for black tile
const unsigned long STUCK_TIMEOUT         = 4000;  // 4 seconds without progress implies stuck
const unsigned long DURATION_ALIGN_JUNCTION = 350; // Time to drive forward to center pivot at junction
const unsigned long DURATION_TURN_90      = 360;   // Time to complete 90-degree spin turn (from working code)
const unsigned long DURATION_TURN_180     = 720;   // Time to complete 180-degree spin turn (from working code)
const unsigned long COOLDOWN_JUNCTION     = 1000;  // Cooldown time to ignore junctions after a turn
const unsigned long COOLDOWN_TILE_IGNORE  = 5000;  // Cooldown time to ignore tile triggers after action (5s)

// ============================================================================
// PID CENTERING CONTROLLER
// ============================================================================
const float PID_KP = 8.5f;   // Proportional Gain
const float PID_KI = 0.05f;  // Integral Gain
const float PID_KD = 4.0f;   // Derivative Gain

const float PID_MAX_CORRECTION = 80.0f; // Maximum speed correction from PID

// ============================================================================
// MOTOR & SPEED SETTINGS
// ============================================================================
const int SPEED_MAX       = 200;  // Maximum base speed (0-255) - from working code
const int SPEED_MIN       = 100;  // Minimum speed to prevent motor stall
const int SPEED_SLOW      = 150;  // Speed when near walls, handling turns or stuck
const int SPEED_TURN      = 230;  // Speed during turns (from working code)
const int RAMP_DELAY_MS   = 8;    // Delay per speed step for smooth acceleration

// ============================================================================
// COLOR SENSOR THRESHOLDS (TCS34725)
// ============================================================================
// Adjust these thresholds based on ambient light and your specific colored tiles.
const float COLOR_BLUE_RATIO_MIN  = 2.10f;  // Blue channel / Red channel ratio to identify light blue tile (set to 2.10 to block white floor reflection)
const float COLOR_BLACK_LUX_MAX   = 8.0f;   // Lux value below which we consider the tile Black (lowered to avoid false alarms)
const uint16_t COLOR_BLACK_CLEAR_MAX = 40;  // Clear channel limit below which we consider it Black (lowered to avoid false alarms)
const uint16_t COLOR_BLUE_CLEAR_MIN  = 110; // Clear channel minimum (set to 110 to block low-light shadows but allow blue tile)

#endif // CONFIG_H
