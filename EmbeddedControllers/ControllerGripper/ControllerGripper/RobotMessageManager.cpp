#include "RobotMessageManager.h"
#include "Configuration.h"


/**
 * Parses a message from the PC to the robot arm controller.
 * 
 * @param data: The byte array representing the message
 * @param dataLength: Length of the data array
 * @param isSuccessful: Out parameter - false if message is not as expected, true if valid
 * @return: MessageFromPcToController struct with parsed data
 */
MessageFromPcToController parseMessageFromPcToController(const uint8_t* data, size_t dataLength, bool* isSuccessful) {
    MessageFromPcToController message = {0};
    *isSuccessful = false;

    if (dataLength != REC_MSG_LENGTH) {
        printf("Invalid message length: Got %zu but expected %d\n", dataLength, REC_MSG_LENGTH);
        return message;
    }

    // Parse the message directly from byte array
    message.prefixT = data[0];
    message.prefixW = data[1];
    message.apiVersion = (int8_t)data[2];
    message.movementMode = (int8_t)data[3];
    
    message.desiredAngles.gripper = (int8_t)data[4];
    message.desiredAngles.wrist = (int8_t)data[5];
    message.desiredAngles.underarm = (int8_t)data[6];
    message.desiredAngles.elbow = (int8_t)data[7];
    message.desiredAngles.overarm = (int8_t)data[8];
    message.desiredAngles.shoulderOut = (int8_t)data[9];
    message.desiredAngles.shoulderForward = (int8_t)data[10];
    
    message.lastKnownAngles.gripper = (int8_t)data[11];
    message.lastKnownAngles.wrist = (int8_t)data[12];
    message.lastKnownAngles.underarm = (int8_t)data[13];
    message.lastKnownAngles.elbow = (int8_t)data[14];
    message.lastKnownAngles.overarm = (int8_t)data[15];
    message.lastKnownAngles.shoulderOut = (int8_t)data[16];
    message.lastKnownAngles.shoulderForward = (int8_t)data[17];

    // Validate prefix
    if (message.prefixT != 84 || message.prefixW != 87) { // 84 is ASCII "T" and 87 is ASCII "W"
        printf("Invalid prefix: Got %c%c but expected TW\n", message.prefixT, message.prefixW);
        return message;
    }
    
    // Validate API version
    if (message.apiVersion != 1) {
        printf("Invalid API version: Got %d but expected 1\n", message.apiVersion);
        return message;
    }
    
    // Validate movement mode
    if (message.movementMode != MOVE_FAST && 
        message.movementMode != RUN_TO_TARGET && 
        message.movementMode != CALIBRATION && 
        message.movementMode != RETURN_TO_ZERO) {
        printf("Invalid movement mode: Got %d but expected one of %d, %d, %d, %d\n", 
               message.movementMode, MOVE_FAST, RUN_TO_TARGET, CALIBRATION, RETURN_TO_ZERO);
        return message;
    }

    // Sanitize desired gripper angle to be between GRIPPER_ANGLE_MIN_DEG and GRIPPER_ANGLE_MAX_DEG
    if (message.desiredAngles.gripper < GRIPPER_ANGLE_MIN_DEG) {
        message.desiredAngles.gripper = GRIPPER_ANGLE_MIN_DEG;
    } else if (message.desiredAngles.gripper > GRIPPER_ANGLE_MAX_DEG) {
        message.desiredAngles.gripper = GRIPPER_ANGLE_MAX_DEG;
    }

    *isSuccessful = true;
    return message;
}



/**
 * Creates a binary message from controller to PC.
 * For now, the errorCode is always 0 (no error).
 *
 * @param currentAngles: Current joint angles
 * @param output: Output buffer to write the message (must be at least SEND_MSG_LENGTH bytes)
 * @return: Number of bytes written to output buffer
 */
size_t createMessageFromControllerToPc(JointAngles currentAngles, uint8_t* output) {
    MessageFromControllerToPC message = {0};
    
    message.prefixT = 84;  // ASCII "T"
    message.prefixW = 87;  // ASCII "W"
    message.apiVersion = 1;
    message.informationSource = ARDUINO_GRIPPER;
    message.errorCode = 0;  // No error
    message.currentAngles = currentAngles;
    
    // Pack the message into byte array
    output[0] = message.prefixT;
    output[1] = message.prefixW;
    output[2] = (uint8_t)message.apiVersion;
    output[3] = (uint8_t)message.informationSource;
    output[4] = (uint8_t)message.errorCode;
    output[5] = (uint8_t)message.currentAngles.gripper;
    output[6] = (uint8_t)message.currentAngles.wrist;
    output[7] = (uint8_t)message.currentAngles.underarm;
    output[8] = (uint8_t)message.currentAngles.elbow;
    output[9] = (uint8_t)message.currentAngles.overarm;
    output[10] = (uint8_t)message.currentAngles.shoulderOut;
    output[11] = (uint8_t)message.currentAngles.shoulderForward;
    
    return SEND_MSG_LENGTH;
}
