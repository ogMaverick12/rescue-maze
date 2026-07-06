/**
 * @file Motors.h
 * @brief Header for the MotorController class handling motor movements and speed ramping.
 */

#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <AFMotor.h>

/**
 * @class MotorController
 * @brief Encapsulates control of 4 DC motors on an L293D shield with speed ramping.
 */
class MotorController {
public:
    MotorController();

    /**
     * @brief Initialize motor speeds and stop them.
     */
    void begin();

    /**
     * @brief Drive the robot forward at the specified target speed.
     * Uses smooth acceleration.
     */
    void Forward(int targetSpeed);

    /**
     * @brief Drive the robot backward at the specified target speed.
     * Uses smooth acceleration.
     */
    void Backward(int targetSpeed);

    /**
     * @brief Stop all motors (coast to stop).
     */
    void Stop();

    /**
     * @brief Brake all motors actively by running them briefly in reverse or release.
     */
    void Brake();

    /**
     * @brief Turn 90 degrees to the left (spin-turn).
     */
    void TurnLeft90();

    /**
     * @brief Turn 90 degrees to the right (spin-turn).
     */
    void TurnRight90();

    /**
     * @brief Turn 180 degrees around (spin-turn).
     */
    void TurnAround();

    /**
     * @brief Gently turn left by slowing down the left side.
     */
    void SoftLeft(int baseSpeed, float bias);

    /**
     * @brief Gently turn right by slowing down the right side.
     */
    void SoftRight(int baseSpeed, float bias);

    /**
     * @brief Set direct motor speeds individually for differential drive steering (PID).
     * @param leftSpeed Speed for left motors (M1, M3). Can be positive or negative.
     * @param rightSpeed Speed for right motors (M2, M4). Can be positive or negative.
     */
    void setDifferentialSpeeds(int leftSpeed, int rightSpeed);

    /**
     * @brief Periodic update function. Ramps current speed towards target speed.
     * Must be called in the main execution loop.
     */
    void update();

private:
    // AFMotor instances
    // M1 = Front Left, M2 = Front Right, M3 = Rear Left, M4 = Rear Right
    AF_DCMotor m1;
    AF_DCMotor m2;
    AF_DCMotor m3;
    AF_DCMotor m4;

    // Speed Control State
    int targetLeftSpeed;
    int targetRightSpeed;
    int currentLeftSpeed;
    int currentRightSpeed;
    
    unsigned long lastRampTime;

    /**
     * @brief Internally applies speed and direction to physical motors.
     */
    void applySpeeds(int left, int right);
};

#endif // MOTORS_H
