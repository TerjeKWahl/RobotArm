#ifndef ROBOT_MESSAGE_MANAGER_H
#define ROBOT_MESSAGE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#define REC_MSG_LENGTH 18
#define SEND_MSG_LENGTH 12

// Define enum for movement modes
typedef enum {
    MOVE_FAST = 1,       // Move all joints as fast as possible (for continuous tracking)
    RUN_TO_TARGET = 2,   // Move to target with smooth acceleration and deceleration (run to target)
    CALIBRATION = 3,     // Calibration mode
    RETURN_TO_ZERO = 4   // Return to standard/zero angles, and exit
} MovementMode;

// Define enum for information sources
typedef enum {
    ARDUINO_GRIPPER = 12,       // Arduino (gripper)
} InformationSource;

// Define struct for joint angles
typedef struct {
    int8_t gripper;
    int8_t wrist;
    int8_t underarm;
    int8_t elbow;
    int8_t overarm;
    int8_t shoulderOut;
    int8_t shoulderForward;
} JointAngles;

typedef struct {
    char prefixT;
    char prefixW;
    int8_t apiVersion;
    int8_t movementMode;
    JointAngles desiredAngles;
    JointAngles lastKnownAngles;
} MessageFromPcToController;

typedef struct {
    char prefixT;
    char prefixW;
    int8_t apiVersion;
    int8_t informationSource;
    int8_t errorCode;
    JointAngles currentAngles;
} MessageFromControllerToPC;

// Function declarations
/**
 * Parses a message from the PC to the robot arm controller.
 * 
 * @param data: The byte array representing the message
 * @param dataLength: Length of the data array
 * @param isSuccessful: Out parameter - false if message is not as expected, true if valid
 * @return: MessageFromPcToController struct with parsed data
 */
MessageFromPcToController parseMessageFromPcToController(const uint8_t* data, size_t dataLength, bool* isSuccessful);

/**
 * Creates a binary message from controller to PC.
 * For now, the errorCode is always 0 (no error).
 *
 * @param currentGripperAngleDeg: Current gripper angle, in degrees
 * @param output: Output buffer to write the message (must be at least SEND_MSG_LENGTH bytes)
 * @return: Number of bytes written to output buffer
 */
size_t createMessageFromControllerToPc(int8_t currentGripperAngleDeg, uint8_t* output);

#endif // ROBOT_MESSAGE_MANAGER_H
