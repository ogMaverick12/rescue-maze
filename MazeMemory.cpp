/**
 * @file MazeMemory.cpp
 * @brief Implementation of the MazeMemory class.
 */

#include "MazeMemory.h"
#include "Config.h"
#include "MazeUtils.h"

MazeMemory::MazeMemory() : pathLength(0), playbackIndex(0), returnMode(false) {
    reset();
}

void MazeMemory::reset() {
    pathLength = 0;
    playbackIndex = 0;
    returnMode = false;
    for (int i = 0; i < MAX_PATH_SIZE; ++i) {
        path[i] = DECISION_NONE;
    }
}

void MazeMemory::addDecision(Decision dec) {
    if (returnMode) {
        // We do not record decisions while playing back the return path
        return;
    }

    if (pathLength < MAX_PATH_SIZE) {
        path[pathLength] = dec;
        pathLength++;
        
        #if DEBUG
        Serial.print(F("[MEMORY] Recorded Decision: "));
        Serial.println(decisionToString(dec));
        #endif
    } else {
        MazeUtils::debugPrint(F("[MEMORY] WARNING: Path memory full, cannot record!"));
    }
}

void MazeMemory::prepareReturnPath() {
    if (pathLength == 0) {
        returnMode = true;
        playbackIndex = 0;
        return;
    }

    // 1. Reverse the path array
    for (int i = 0; i < pathLength / 2; ++i) {
        Decision temp = path[i];
        path[i] = path[pathLength - 1 - i];
        path[pathLength - 1 - i] = temp;
    }

    // 2. Invert the directions: Left <-> Right, Straight stays Straight
    for (int i = 0; i < pathLength; ++i) {
        if (path[i] == DECISION_LEFT) {
            path[i] = DECISION_RIGHT;
        } else if (path[i] == DECISION_RIGHT) {
            path[i] = DECISION_LEFT;
        }
        // DECISION_STRAIGHT remains DECISION_STRAIGHT
    }

    playbackIndex = 0;
    returnMode = true;
    
    MazeUtils::debugPrint(F("[MEMORY] Path reversed and inverted for return run."));
    printPath();
}

Decision MazeMemory::getNextDecision() {
    if (!returnMode || playbackIndex >= pathLength) {
        return DECISION_NONE;
    }

    Decision nextDec = path[playbackIndex];
    playbackIndex++;
    
    #if DEBUG
    Serial.print(F("[MEMORY] Playback Decision #"));
    Serial.print(playbackIndex);
    Serial.print(F(": "));
    Serial.println(decisionToString(nextDec));
    #endif

    return nextDec;
}

bool MazeMemory::isReturnMode() const {
    return returnMode;
}

int MazeMemory::getPathLength() const {
    return pathLength;
}

int MazeMemory::getPlaybackIndex() const {
    return playbackIndex;
}

const char* MazeMemory::decisionToString(Decision dec) const {
    switch (dec) {
        case DECISION_LEFT:     return "LEFT";
        case DECISION_STRAIGHT: return "STRAIGHT";
        case DECISION_RIGHT:    return "RIGHT";
        default:                return "NONE";
    }
}

void MazeMemory::printPath() const {
    #if DEBUG
    Serial.print(F("[MEMORY] Path Log [Len="));
    Serial.print(pathLength);
    Serial.print(F(", ReturnMode="));
    Serial.print(returnMode ? F("TRUE") : F("FALSE"));
    Serial.print(F("]: "));
    for (int i = 0; i < pathLength; ++i) {
        Serial.print(decisionToString(path[i]));
        if (i < pathLength - 1) {
            Serial.print(F(" -> "));
        }
    }
    Serial.println();
    #endif
}
