/**
 * @file Navigation.cpp
 * @brief Implementation of the Navigation class, managing the maze-solving state machine.
 */

#include "Navigation.h"
#include "Config.h"
#include "MazeUtils.h"

Navigation::Navigation()
    : currentState(STATE_FORWARD),
      nextStateAfterAction(STATE_FORWARD),
      pidError(0.0f),
      pidLastError(0.0f),
      pidIntegral(0.0f),
      pidLastTime(0),
      junctionDetected(false),
      latchLeftOpen(false),
      latchRightOpen(false),
      latchFrontBlocked(false),
      junctionConfirmCount(0),
      tileIgnoreActive(false),
      blackActionStep(0),
      lastFrontDist(30.0f),
      lastLeftDist(30.0f),
      lastRightDist(30.0f),
      lastStuckCheckTime(0) {}

void Navigation::begin() {
    // 1. Set up pins for LED and buzzer
    MazeUtils::setupPins();

    // 2. Initialize motor controller
    motors.begin();

    // 3. Initialize ultrasonic system
    sonar.begin();

    // 4. Initialize TCS34725 color sensor
    colorSensor.begin();

    // 5. Reset maze path memory
    memory.reset();

    // Set initial state
    currentState = STATE_FORWARD;
    pidLastTime = millis();
    lastStuckCheckTime = millis();
    stuckTimer.start(STUCK_TIMEOUT);
    junctionCooldownTimer.stop();

    // Initialize new patch variables
    junctionConfirmCount = 0;
    latchLeftOpen = false;
    latchRightOpen = false;
    latchFrontBlocked = false;
    tileIgnoreActive = false;
    tileIgnoreTimer.stop();
    blackActionStep = 0;

    MazeUtils::debugPrint(F("[NAV] Navigation state machine started. Initial State: FORWARD"));
}

void Navigation::update() {
    // 1. Maintain hardware states (drives ramping, color updates, LED/buzzer timers)
    motors.update();
    sonar.update();
    colorSensor.update();
    ledBlinker.update();
    buzzer.update();

    // 2. Non-blocking Tile Ignore lockout timer update
    if (tileIgnoreActive && tileIgnoreTimer.isExpired()) {
        tileIgnoreActive = false;
        tileIgnoreTimer.stop();
        MazeUtils::debugPrint(F("[NAV] Tile ignore cooldown expired. Ready for new tiles."));
    }

    // 3. Debug print in true DEBUG mode
    #if DEBUG
    static unsigned long lastDebugPrintTime = 0;
    if (millis() - lastDebugPrintTime >= 500) {
        lastDebugPrintTime = millis();
        Serial.print(F("--- STATE: "));
        Serial.print(stateToString(currentState));
        Serial.print(F(" | ReturnMode: "));
        Serial.print(memory.isReturnMode() ? F("TRUE") : F("FALSE"));
        Serial.print(F(" | IgnoreLockout: "));
        Serial.println(tileIgnoreActive ? F("ACTIVE") : F("INACTIVE"));
        sonar.printDistances();
        colorSensor.printColorInfo();
        memory.printPath();
    }
    #endif

    // 4. Drive the State Machine
    switch (currentState) {
        case STATE_FORWARD:
            runForwardState();
            break;
            
        case STATE_ALIGN_JUNCTION:
            runAlignJunctionState();
            break;
            
        case STATE_TURN_LEFT:
        case STATE_TURN_RIGHT:
        case STATE_TURN_AROUND:
            runTurnState();
            break;
            
        case STATE_BLUE_ACTION:
            runBlueActionState();
            break;
            
        case STATE_BLACK_ACTION:
            runBlackActionState();
            break;
            
        case STATE_STUCK_RECOVERY:
            runStuckRecoveryState();
            break;

        case STATE_FINISHED:
            runFinishedState();
            break;
            
        default:
            currentState = STATE_FORWARD;
            break;
    }
}

void Navigation::runForwardState() {
    // A. Check for Tile triggers first (Priority over motion logic), with lockout protection
    if (!tileIgnoreActive) {
        if (colorSensor.isBlueDetected()) {
            // If we are NOT already in return mode, the blue tile represents the goal!
            if (!memory.isReturnMode()) {
                MazeUtils::debugPrint(F("[NAV] Goal reached! Entering STATE_BLUE_ACTION"));
                motors.Brake();
                // Pause for 5 seconds and blink the blue LED rapidly for the entire duration (25 cycles * 2 * 100ms = 5000ms)
                stateTimer.start(DURATION_BLUE_PAUSE);
                ledBlinker.startBlinking(25, 100);
                currentState = STATE_BLUE_ACTION;
                return;
            }
        }

        if (colorSensor.isBlackDetected()) {
            MazeUtils::debugPrint(F("[NAV] Black/hazard tile detected! Entering STATE_BLACK_ACTION"));
            motors.Brake();
            buzzer.trigger(DURATION_BLACK_BUZZER);
            stateTimer.start(DURATION_BLACK_BUZZER);
            blackActionStep = 0; // Reset black action sequence step
            currentState = STATE_BLACK_ACTION;
            return;
        }
    }

    // B. Check for stuck conditions periodically
    checkStuckCondition();
    if (currentState == STATE_STUCK_RECOVERY) return;

    // C. Check if we reached a junction/intersection (incorporates debouncing)
    if (checkJunctionTrigger()) {
        // CRITICAL PATCH: Latch the FULL snapshot (Left, Right, and Front) together at the moment of confirmation
        latchLeftOpen = sonar.isLeftOpen();
        latchRightOpen = sonar.isRightOpen();
        latchFrontBlocked = sonar.isFrontBlocked();
        
        // Reset debounce confirmation counter
        junctionConfirmCount = 0;
        
        if (latchFrontBlocked) {
            MazeUtils::debugPrint(F("[NAV] Junction confirmed (Front Blocked). Skipping alignment to prevent collision."));
            stateTimer.start(0); // Transition immediately in the next loop without driving forward
        } else {
            MazeUtils::debugPrint(F("[NAV] Junction confirmed (Front Open). Transitioning to STATE_ALIGN_JUNCTION"));
            stateTimer.start(DURATION_ALIGN_JUNCTION);
            motors.Forward(SPEED_SLOW);
        }
        
        currentState = STATE_ALIGN_JUNCTION;
        return;
    }

    // D. Normal Wall Following and Centering using PID
    float correction = computePID();
    
    // Speed profile logic:
    // If the front path is narrow/closing or we are near side walls, slow down base speed.
    int baseSpeed = SPEED_MAX;
    float frontDist = sonar.getFrontDistance();
    float leftDist = sonar.getLeftDistance();
    float rightDist = sonar.getRightDistance();
    
    if (frontDist < 35.0f || leftDist < 8.0f || rightDist < 8.0f) {
        baseSpeed = SPEED_SLOW; // Slow down near obstacles for safety
    }
    
    // Adjust differential motor speeds using the PID correction output
    int leftSpeed = baseSpeed - (int)correction;
    int rightSpeed = baseSpeed + (int)correction;
    
    // Enforce speed limits (SPEED_MIN to SPEED_MAX)
    leftSpeed = constrain(leftSpeed, SPEED_MIN, SPEED_MAX);
    rightSpeed = constrain(rightSpeed, SPEED_MIN, SPEED_MAX);
    
    motors.setDifferentialSpeeds(leftSpeed, rightSpeed);
}

void Navigation::runAlignJunctionState() {
    // Wait for the alignment timer to expire
    if (stateTimer.isExpired()) {
        motors.Brake();
        
        // -------------------------------------------------------------
        // DECISION MAKING BRANCH (uses consistent snapshot)
        // -------------------------------------------------------------
        if (!memory.isReturnMode()) {
            // Forward Run: Use LEFT-WALL FOLLOWING algorithm priority:
            // 1. LEFT, 2. FORWARD, 3. RIGHT, 4. TURN AROUND (Dead End)
            
            if (latchLeftOpen) {
                // Priority 1: Left is Open. Take the left turn.
                MazeUtils::debugPrint(F("[NAV] Decision: TURN LEFT"));
                memory.addDecision(DECISION_LEFT);
                motors.TurnLeft90();
                stateTimer.start(DURATION_TURN_90);
                currentState = STATE_TURN_LEFT;
            } 
            else if (!latchFrontBlocked) {
                // Priority 2: Front is Open. Go Straight.
                MazeUtils::debugPrint(F("[NAV] Decision: GO STRAIGHT"));
                // Record straight decision only if Right was also an option (a choice point)
                if (latchRightOpen) {
                    memory.addDecision(DECISION_STRAIGHT);
                }
                
                // Continue forward, starting the cooldown to not trigger again immediately
                junctionCooldownTimer.start(COOLDOWN_JUNCTION);
                currentState = STATE_FORWARD;
            } 
            else if (latchRightOpen) {
                // Priority 3: Right is Open. Take the right turn.
                MazeUtils::debugPrint(F("[NAV] Decision: TURN RIGHT"));
                memory.addDecision(DECISION_RIGHT);
                motors.TurnRight90();
                stateTimer.start(DURATION_TURN_90);
                currentState = STATE_TURN_RIGHT;
            } 
            else {
                // Priority 4: All paths blocked. Turn Around.
                MazeUtils::debugPrint(F("[NAV] Decision: TURN AROUND (Dead End)"));
                motors.TurnAround();
                stateTimer.start(DURATION_TURN_180);
                currentState = STATE_TURN_AROUND;
            }
        } 
        else {
            // Return Run: Retrieve inverted decision from memory
            Decision nextDec = memory.getNextDecision();
            
            if (nextDec == DECISION_LEFT) {
                MazeUtils::debugPrint(F("[NAV] Playback Turn: LEFT"));
                motors.TurnLeft90();
                stateTimer.start(DURATION_TURN_90);
                currentState = STATE_TURN_LEFT;
            } 
            else if (nextDec == DECISION_RIGHT) {
                MazeUtils::debugPrint(F("[NAV] Playback Turn: RIGHT"));
                motors.TurnRight90();
                stateTimer.start(DURATION_TURN_90);
                currentState = STATE_TURN_RIGHT;
            } 
            else {
                // Playback STRAIGHT or NONE (end reached)
                if (nextDec == DECISION_NONE) {
                    MazeUtils::debugPrint(F("[NAV] Return complete! Reached start tile."));
                    motors.Stop();
                    // Play a success indicator in a non-blocking way
                    ledBlinker.startBlinking(5, 200);
                    buzzer.trigger(1000);
                    
                    // CRITICAL PATCH: Switch to safe non-blocking finished state instead of while(true)
                    currentState = STATE_FINISHED;
                } else {
                    MazeUtils::debugPrint(F("[NAV] Playback Turn: STRAIGHT"));
                    junctionCooldownTimer.start(COOLDOWN_JUNCTION);
                    currentState = STATE_FORWARD;
                }
            }
        }
    }
}

void Navigation::runTurnState() {
    // Wait for spin turn timer to expire
    if (stateTimer.isExpired()) {
        motors.Brake();
        delay(60); // Stability delay after turn (from working code)
        MazeUtils::debugPrint(F("[NAV] Turn completed. Returning to STATE_FORWARD."));
        
        // Start cooldown timer to let the robot move away from the junction 
        // before re-enabling trigger detection
        junctionCooldownTimer.start(COOLDOWN_JUNCTION);
        
        // Reset PID integral and errors to prevent sudden jumps after turn
        pidIntegral = 0;
        pidLastError = 0;
        pidLastTime = millis();
        
        // Resume forward state
        currentState = STATE_FORWARD;
        stuckTimer.start(STUCK_TIMEOUT); // Reset stuck timer
    }
}

void Navigation::runBlueActionState() {
    // Keep robot stopped. Check if the 5-second goal pause timer has expired.
    if (stateTimer.isExpired()) {
        MazeUtils::debugPrint(F("[NAV] 5-second blue pause complete. Reversing memory and initiating return run."));
        
        // Ensure the LED is explicitly turned off
        digitalWrite(PIN_BLUE_LED, LOW);
        
        // 1. Convert memory to return path
        memory.prepareReturnPath();
        
        // 2. Set the tile ignore lockout so we don't immediately re-detect the Blue tile
        tileIgnoreActive = true;
        tileIgnoreTimer.start(COOLDOWN_TILE_IGNORE);
        
        // 3. Spin the robot 180 degrees to head back
        motors.TurnAround();
        stateTimer.start(DURATION_TURN_180);
        currentState = STATE_TURN_AROUND;
    }
}

void Navigation::runBlackActionState() {
    if (blackActionStep == 0) {
        // Step 0: Wait for beeping/brake timer to expire
        if (stateTimer.isExpired()) {
            // Check if there is an immediate exit to the Left or Right directly at the black tile
            bool leftOpen = sonar.isLeftOpen();
            bool rightOpen = sonar.isRightOpen();

            if (leftOpen) {
                MazeUtils::debugPrint(F("[NAV] Avoiding black: Turn LEFT directly from tile"));
                if (!memory.isReturnMode()) {
                    memory.addDecision(DECISION_LEFT);
                }
                motors.TurnLeft90();
                stateTimer.start(DURATION_TURN_90);
                blackActionStep = 2; // Skip backup, go directly to turn phase
            } 
            else if (rightOpen) {
                MazeUtils::debugPrint(F("[NAV] Avoiding black: Turn RIGHT directly from tile"));
                if (!memory.isReturnMode()) {
                    memory.addDecision(DECISION_RIGHT);
                }
                motors.TurnRight90();
                stateTimer.start(DURATION_TURN_90);
                blackActionStep = 2; // Skip backup, go directly to turn phase
            } 
            else {
                // Both sides blocked: We must back up to clear the black tile and go back
                MazeUtils::debugPrint(F("[NAV] Avoiding black: Both sides blocked. Backing up..."));
                motors.Backward(SPEED_SLOW);
                stateTimer.start(750); // Back up for 750 milliseconds
                blackActionStep = 1;   // Go to backup phase
            }
        }
    } 
    else if (blackActionStep == 1) {
        // Step 1: Wait for backup to complete, then turn around to return
        if (stateTimer.isExpired()) {
            motors.Brake();
            MazeUtils::debugPrint(F("[NAV] Backup complete. Turning around 180 degrees..."));

            // Since it was a straight corridor with a black tile, we must turn around
            motors.TurnAround();
            stateTimer.start(DURATION_TURN_180);
            blackActionStep = 2;
        }
    } 
    else if (blackActionStep == 2) {
        // Step 2: Wait for turn (90 or 180 deg) to complete
        if (stateTimer.isExpired()) {
            motors.Brake();
            MazeUtils::debugPrint(F("[NAV] Avoidance turn complete. Resuming forward search."));

            // Lockout the color sensor temporarily so we don't immediately re-detect black
            tileIgnoreActive = true;
            tileIgnoreTimer.start(COOLDOWN_TILE_IGNORE);

            // Set junction cooldown to prevent re-triggering immediately
            junctionCooldownTimer.start(COOLDOWN_JUNCTION);
            
            // Reset navigation and stuck timers
            stuckTimer.start(STUCK_TIMEOUT);
            pidLastTime = millis();
            
            blackActionStep = 0; // Reset sequence
            currentState = STATE_FORWARD;
        }
    }
}

void Navigation::runStuckRecoveryState() {
    // Wait for the recovery timer to expire (robot drives backward)
    if (stateTimer.isExpired()) {
        motors.Brake();
        MazeUtils::debugPrint(F("[NAV] Stuck recovery done. Retrying STATE_FORWARD."));
        
        // Restart navigation timers and state
        pidIntegral = 0;
        pidLastError = 0;
        pidLastTime = millis();
        
        junctionCooldownTimer.start(COOLDOWN_JUNCTION);
        stuckTimer.start(STUCK_TIMEOUT);
        currentState = STATE_FORWARD;
    }
}

void Navigation::runFinishedState() {
    // Keep motors turned off
    motors.Stop();

    // Periodically flash LED and beep buzzer in a non-blocking loop (every 3 seconds)
    static unsigned long lastFinishedIndicator = 0;
    unsigned long now = millis();
    if (now - lastFinishedIndicator >= 3000) {
        lastFinishedIndicator = now;
        ledBlinker.startBlinking(2, 150);
        buzzer.trigger(100);
        MazeUtils::debugPrint(F("[NAV] Safe Finished Idle: Robot reached start."));
    }
}

float Navigation::computePID() {
    // Calculate sample time (dt) in seconds
    unsigned long now = millis();
    float dt = (float)(now - pidLastTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f; // Prevent divide by zero if updates are extremely fast
    pidLastTime = now;

    float leftDist = sonar.getLeftDistance();
    float rightDist = sonar.getRightDistance();

    // Determine error based on wall presence:
    bool leftWallPresent = (leftDist < DIST_WALL_THRESHOLD);
    bool rightWallPresent = (rightDist < DIST_WALL_THRESHOLD);

    if (leftWallPresent && rightWallPresent) {
        // Both walls present: center the robot between them
        pidError = leftDist - rightDist;
    } 
    else if (leftWallPresent) {
        // Only Left wall present: follow the left wall at DIST_TARGET_CENTER
        pidError = leftDist - DIST_TARGET_CENTER;
    } 
    else if (rightWallPresent) {
        // Only Right wall present: follow the right wall at DIST_TARGET_CENTER
        pidError = DIST_TARGET_CENTER - rightDist;
    } 
    else {
        // No walls detected (e.g. wall lost or open space)
        pidError = 0.0f;
    }

    // PID Calculations
    // Proportional Term
    float pTerm = PID_KP * pidError;

    // Integral Term (with windup protection)
    pidIntegral += pidError * dt;
    pidIntegral = constrain(pidIntegral, -50.0f, 50.0f);
    float iTerm = PID_KI * pidIntegral;

    // Derivative Term
    float derivative = (pidError - pidLastError) / dt;
    float dTerm = PID_KD * derivative;

    pidLastError = pidError;

    // Summing terms to compute the final correction
    float correction = pTerm + iTerm + dTerm;

    // Constrain the correction to protect motor speed registers
    correction = constrain(correction, -PID_MAX_CORRECTION, PID_MAX_CORRECTION);

    return correction;
}

bool Navigation::checkJunctionTrigger() {
    // If we are in the cooldown period, ignore junction triggers
    if (junctionCooldownTimer.isActive() && !junctionCooldownTimer.isExpired()) {
        junctionConfirmCount = 0; // Keep debounce count reset during cooldown
        return false;
    }

    // A junction is physically present if Left/Right opens up or Front is blocked
    bool rawDetect = (sonar.isLeftOpen() || sonar.isRightOpen() || sonar.isFrontBlocked());

    // CRITICAL PATCH: Digital confirmation debounce filter to prevent false triggers from sensor noise
    if (rawDetect) {
        junctionConfirmCount++;
        if (junctionConfirmCount >= JUNCTION_CONFIRM_READINGS) {
            return true; // Confirmed over multiple consecutive cycles
        }
    } else {
        junctionConfirmCount = 0; // Immediately reset on noise drop-out
    }

    return false;
}

void Navigation::checkStuckCondition() {
    // CRITICAL PATCH: Verify we are actually trying to drive forward/backward
    bool isDriving = (currentState == STATE_FORWARD || currentState == STATE_ALIGN_JUNCTION);
    if (!isDriving) {
        stuckTimer.start(STUCK_TIMEOUT); // Reset stuck timer when stationary or turning
        return;
    }

    float currentFront = sonar.getFrontDistance();
    float currentLeft = sonar.getLeftDistance();
    float currentRight = sonar.getRightDistance();

    // CRITICAL PATCH: Avoid false stuck triggers in long straight corridors.
    // Only check stuck if we are physically close to some wall/obstacle where wedging or scraping can happen.
    bool nearObstacle = (currentFront < (DIST_FRONT_BLOCKED + 5.0f) || currentLeft < 8.0f || currentRight < 8.0f);

    if (!nearObstacle) {
        stuckTimer.start(STUCK_TIMEOUT); // Reset timer in open corridors
        lastFrontDist = currentFront;
        lastLeftDist = currentLeft;
        lastRightDist = currentRight;
        return;
    }

    unsigned long now = millis();
    if (now - lastStuckCheckTime >= 1000) { // Check once every second
        lastStuckCheckTime = now;

        // Calculate differences in distances
        float diffFront = abs(currentFront - lastFrontDist);
        float diffLeft = abs(currentLeft - lastLeftDist);
        float diffRight = abs(currentRight - lastRightDist);

        // If the robot is attempting to move but all distances stay unchanged (< 0.5 cm)
        if (diffFront < 0.5f && diffLeft < 0.5f && diffRight < 0.5f) {
            if (stuckTimer.isExpired()) {
                MazeUtils::debugPrint(F("[NAV] WARNING: Robot stuck detected! Reversing."));
                motors.Brake();
                
                // Set recovery state: drive backward slowly for 1 second
                motors.Backward(SPEED_SLOW);
                stateTimer.start(1000);
                currentState = STATE_STUCK_RECOVERY;
            }
        } else {
            // Robot is making progress, reset stuck timer and update references
            stuckTimer.start(STUCK_TIMEOUT);
            
            lastFrontDist = currentFront;
            lastLeftDist = currentLeft;
            lastRightDist = currentRight;
        }
    }
}

RobotState Navigation::getCurrentState() const {
    return currentState;
}

const char* Navigation::stateToString(RobotState state) const {
    switch (state) {
        case STATE_FORWARD:          return "FORWARD";
        case STATE_ALIGN_JUNCTION:   return "ALIGN_JUNCTION";
        case STATE_TURN_LEFT:        return "TURN_LEFT";
        case STATE_TURN_RIGHT:       return "TURN_RIGHT";
        case STATE_TURN_AROUND:      return "TURN_AROUND";
        case STATE_BLUE_ACTION:      return "BLUE_ACTION";
        case STATE_BLACK_ACTION:     return "BLACK_ACTION";
        case STATE_STUCK_RECOVERY:   return "STUCK_RECOVERY";
        case STATE_FINISHED:         return "FINISHED";
        default:                     return "UNKNOWN";
    }
}
