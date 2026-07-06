/**
 * @file MazeMemory.h
 * @brief Header for the MazeMemory class to store and playback path decisions.
 */

#ifndef MAZEMEMORY_H
#define MAZEMEMORY_H

#include <Arduino.h>

/**
 * @enum Decision
 * @brief Represents navigation decisions made at junctions.
 */
enum Decision : uint8_t {
    DECISION_LEFT = 0,
    DECISION_STRAIGHT,
    DECISION_RIGHT,
    DECISION_NONE
};

/**
 * @class MazeMemory
 * @brief Manages the recorded path of the robot, enabling return-to-start navigation.
 */
class MazeMemory {
public:
    MazeMemory();

    /**
     * @brief Clear all path memory.
     */
    void reset();

    /**
     * @brief Add a decision to the path memory.
     * @param dec The decision made at the junction.
     */
    void addDecision(Decision dec);

    /**
     * @brief Reverse the path and invert turns (Left <-> Right) to prepare for return.
     */
    void prepareReturnPath();

    /**
     * @brief Retrieve the next decision in the return path.
     * @return The next Decision, or DECISION_NONE if end of path reached.
     */
    Decision getNextDecision();

    /**
     * @brief Check if we are currently in return mode.
     */
    bool isReturnMode() const;

    /**
     * @brief Get the number of stored decisions.
     */
    int getPathLength() const;

    /**
     * @brief Get the current playback index.
     */
    int getPlaybackIndex() const;

    /**
     * @brief Convert a Decision enum to a string.
     */
    const char* decisionToString(Decision dec) const;

    /**
     * @brief Print the stored decisions to the serial port (when in DEBUG).
     */
    void printPath() const;

private:
    static const int MAX_PATH_SIZE = 100;
    Decision path[MAX_PATH_SIZE];
    int pathLength;
    int playbackIndex;
    bool returnMode;
};

#endif // MAZEMEMORY_H
