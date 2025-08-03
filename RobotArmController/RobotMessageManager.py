"""
This file creates and checks outgoing and incoming messages to the the robot arm
controllers (Lego hubs 1 and 2, and the arduino) and the VR app, according to 
the API defined in Docs/APIs.md.
"""

import ctypes
import enum

# Constants
UNKNOWN_ANGLE = -128  # Value used to indicate unknown angle


# Define enum for movement modes:
class MovementMode(enum.IntEnum):
    MOVE_FAST = 1      # Move all joints as fast as possible (for continuous tracking)
    RUN_TO_TARGET = 2  # Move to target with smooth acceleration and deceleration (run to target)
    CALIBRATION = 3    # Calibration mode
    RETURN_TO_ZERO = 4 # Return to standard/zero angles, and exit


# Define enum for information sources:
class InformationSource(enum.IntEnum):
    LEGO_HUB_1_LOWER_ARM = 10  # Lego Technic Hub 1 (lower arm)
    LEGO_HUB_2_SHOULDER = 11   # Lego Technic Hub 2 (shoulder)
    ARDUINO_GRIPPER = 12       # Arduino (gripper)
    VR_APP = 20                # VR app
    PC_APP = 21                # PC app


# Define struct for joint angles:
class JointAngles(ctypes.Structure):
    _fields_ = [
        ("gripper", ctypes.c_int8),
        ("wrist", ctypes.c_int8),
        ("underarm", ctypes.c_int8),
        ("elbow", ctypes.c_int8),
        ("overarm", ctypes.c_int8),
        ("shoulder_out", ctypes.c_int8),
        ("shoulder_forward", ctypes.c_int8)
    ]


# Define struct for distance and angle offsets (used in VR-PC communication):
class DistanceAndAngleOffset(ctypes.Structure):
    _fields_ = [
        ("x_distance", ctypes.c_int16),  # X distance offset in mm
        ("y_distance", ctypes.c_int16),  # Y distance offset in mm  
        ("z_distance", ctypes.c_int16),  # Z distance offset in mm
        ("x_angle", ctypes.c_int16),     # X angle offset in degrees
        ("y_angle", ctypes.c_int16),     # Y angle offset in degrees
        ("z_angle", ctypes.c_int16)      # Z angle offset in degrees
    ]

    # Define pretty printing for debugging:
    def __repr__(self):
        return (f"XYZ distance: {self.x_distance} {self.y_distance} {self.z_distance}. XYZ angles: { \
                self.x_angle} {self.y_angle} {self.z_angle}.")


class MessageFromPcToController(ctypes.Structure):
    _fields_ = [
        ("prefix_t", ctypes.c_char),
        ("prefix_w", ctypes.c_char),
        ("api_version", ctypes.c_int8),
        ("movement_mode", ctypes.c_int8),
        ("desired_angles", JointAngles),
        ("last_known_angles", JointAngles)
    ]


class MessageFromControllerToPC(ctypes.Structure):
    _fields_ = [
        ("prefix_t", ctypes.c_char),
        ("prefix_w", ctypes.c_char),
        ("api_version", ctypes.c_int8),
        ("information_source", ctypes.c_int8),
        ("error_code", ctypes.c_int8),
        ("current_angles", JointAngles)
    ]


class MessageFromPcToVR(ctypes.Structure):
    _fields_ = [
        ("prefix_t", ctypes.c_char),
        ("prefix_k", ctypes.c_char), 
        ("prefix_w", ctypes.c_char),
        ("api_version", ctypes.c_int8),
        ("information_source", ctypes.c_int8),
        ("connection_status", ctypes.c_int8),
        ("last_known_distance_and_angle_offset", DistanceAndAngleOffset),
        ("last_known_angles", JointAngles)
    ]


class MessageFromVRToPC(ctypes.Structure):
    _fields_ = [
        ("prefix_t", ctypes.c_char),
        ("prefix_k", ctypes.c_char),
        ("prefix_w", ctypes.c_char),
        ("api_version", ctypes.c_int8),
        ("information_source", ctypes.c_int8),
        ("desired_distance_and_angle_offset", DistanceAndAngleOffset)
    ]


def create_message_from_PC_to_controller(movement_mode: MovementMode, 
                                         desired_angles: JointAngles, 
                                         last_known_angles: JointAngles) -> bytes:
    """
    Creates a message to send from the PC to the robot arm controller.

    :param movement_mode: The movement mode to set.
    :param desired_angles: The desired angles for the joints.
    :param last_known_angles: The last known angles for the joints.
    :return: A byte array representing the message.
    """
    # Clip desired angles to valid signed byte range (-128 to 127), to avoid overflow:
    desired_angles.gripper = max(-128, min(127, desired_angles.gripper))
    desired_angles.wrist = max(-128, min(127, desired_angles.wrist))
    desired_angles.underarm = max(-128, min(127, desired_angles.underarm))
    desired_angles.elbow = max(-128, min(127, desired_angles.elbow))
    desired_angles.overarm = max(-128, min(127, desired_angles.overarm))
    desired_angles.shoulder_out = max(-128, min(127, desired_angles.shoulder_out))
    desired_angles.shoulder_forward = max(-128, min(127, desired_angles.shoulder_forward))

    message = MessageFromPcToController(
        prefix_t = b'T',
        prefix_w = b'W',
        api_version = 1,
        movement_mode = movement_mode,
        desired_angles = desired_angles,
        last_known_angles = last_known_angles
    )
    return ctypes.string_at(ctypes.byref(message), ctypes.sizeof(message))


def parse_message_from_controller_to_PC(data: bytes) -> MessageFromControllerToPC:
    """
    Parses a message from the robot arm controller (Lego hub or Arduino) to the PC.

    :param data: The byte array representing the message.
    :return: A MessageFromControllerToPC object, or None if the message is invalid.
    """
    if len(data) != ctypes.sizeof(MessageFromControllerToPC):
        print(f"Invalid message length: Got {len(data)} but expected {ctypes.sizeof(MessageFromControllerToPC)}")
        return None
    if data[0:2] != b'TW':
        print(f"Invalid prefix: Got {data[0:2]} but expected b'TW'")
        return None
    if data[2] != 1:  # Check API version
        print(f"Invalid API version: Got {data[2]} but expected 1")
        return None
    if data[3] not in (InformationSource.LEGO_HUB_1_LOWER_ARM, 
                       InformationSource.LEGO_HUB_2_SHOULDER, 
                       InformationSource.ARDUINO_GRIPPER):
        print(f"Invalid information source: Got {data[3]} but expected one of {list(InformationSource)}")
        return None
    
    # Create a MessageFromControllerToPC object from the byte array
    message = MessageFromControllerToPC.from_buffer_copy(data)
    return message


def create_message_from_PC_to_VR(is_connection_to_controllers_ok: bool,
                                last_known_distance_and_angle_offset: DistanceAndAngleOffset,
                                last_known_angles: JointAngles) -> bytes:
    """
    Creates a message to send from the PC to the VR app.

    :param is_connection_to_controllers_ok: False = Not connected to controllers, True = Connected to controllers
    :param last_known_distance_and_angle_offset: The last known distance and angle offsets
    :param last_known_angles: The last known joint angles
    :return: A byte array representing the message with proper big-endian encoding
    """
    message = MessageFromPcToVR(
        prefix_t = b'T',
        prefix_k = b'K',
        prefix_w = b'W',
        api_version = 1,
        information_source = InformationSource.PC_APP,
        connection_status = 0 if not is_connection_to_controllers_ok else 1,
        last_known_distance_and_angle_offset = last_known_distance_and_angle_offset,
        last_known_angles = last_known_angles
    )
    
    # Convert to bytes
    message_bytes = ctypes.string_at(ctypes.byref(message), ctypes.sizeof(message))
    
    # Convert 16-bit integers to big-endian format
    result = bytearray(message_bytes)
    
    # Offsets for the 16-bit fields (after 5 single-byte fields: T, K, W, version, source, status)
    offset_base = 6
    
    # Convert each 16-bit field to big-endian
    for i in range(6):  # 6 int16 fields in DistanceAndAngleOffset
        field_offset = offset_base + i * 2
        value = int.from_bytes(result[field_offset:field_offset+2], 'little')
        result[field_offset:field_offset+2] = value.to_bytes(2, 'big', signed=True)
    
    return bytes(result)


def parse_message_from_VR_to_PC(data: bytes) -> MessageFromVRToPC:
    """
    Parses a message from the VR app to the PC.

    :param data: The byte array representing the message.
    :return: A MessageFromVRToPC object, or None if the message is invalid.
    """
    if len(data) != ctypes.sizeof(MessageFromVRToPC):
        print(f"Invalid message length: Got {len(data)} but expected {ctypes.sizeof(MessageFromVRToPC)}")
        return None
    if data[0:3] != b'TKW':
        print(f"Invalid prefix: Got {data[0:3]} but expected b'TKW'")
        return None
    if data[3] != 1:  # Check API version
        print(f"Invalid API version: Got {data[3]} but expected 1")
        return None
    if data[4] != InformationSource.VR_APP:
        print(f"Invalid information source: Got {data[4]} but expected {InformationSource.VR_APP}")
        return None
    
    # Convert big-endian 16-bit fields to little-endian before parsing
    data_converted = bytearray(data)
    
    # Offsets for the 16-bit fields (after 5 single-byte fields: T, K, W, version, source)
    offset_base = 5
    
    # Convert each 16-bit field from big-endian to little-endian
    for i in range(6):  # 6 int16 fields in DistanceAndAngleOffset
        field_offset = offset_base + i * 2
        value = int.from_bytes(data_converted[field_offset:field_offset+2], 'big', signed=True)
        data_converted[field_offset:field_offset+2] = value.to_bytes(2, 'little', signed=True)
    
    # Create a MessageFromVRToPC object from the converted byte array
    message = MessageFromVRToPC.from_buffer_copy(data_converted)
    return message
