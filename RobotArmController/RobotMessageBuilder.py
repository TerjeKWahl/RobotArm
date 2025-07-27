"""
This file creates and checks outgoing and incoming messages to the the robot arm
controllers (Lego hubs 1 and 2, and the arduino), according to the API defined in Docs/APIs.md.
"""

import ctypes
import enum

# Define enum for movement modes:
class MovementMode(enum.IntEnum):
    MOVE_FAST = 1  # move all joints as fast as possible
    SYNCHRONIZE = 2  # synchronize joints to arrive at the destination simultaneously
    CALIBRATION = 3  # calibration mode
    RETURN_TO_ZERO = 4  # return to standard/zero angles, and exit


# Define struct for joint angles:
class JointAngles(ctypes.Structure):
    _fields_ = [
        ("gripper", ctypes.c_int8),
        ("wrist", ctypes.c_int8),
        ("underarm", ctypes.c_int8),
        ("elbow", ctypes.c_int8),
        ("overarm", ctypes.c_int8),
        ("shoulder_forward", ctypes.c_int8),
        ("shoulder_out", ctypes.c_int8)
    ]


class MessageFromPcToController(ctypes.Structure):
    _fields_ = [
        ("prefix_t", ctypes.c_char),
        ("prefix_w", ctypes.c_char),
        ("api_version", ctypes.c_int8),
        ("movement_mode", ctypes.c_int8),
        ("desired_angles", JointAngles),
        ("last_known_angles", JointAngles)
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
    message = MessageFromPcToController(
        prefix_t=b'T',
        prefix_w=b'W',
        api_version=1,
        movement_mode=movement_mode,
        desired_angles=desired_angles,
        last_known_angles=last_known_angles
    )
    return ctypes.string_at(ctypes.byref(message), ctypes.sizeof(message))


def parse_message_from_PC_to_controller(data: bytes) -> MessageFromPcToController:
    """
    Parses a message from the PC to the robot arm controller.

    :param data: The byte array representing the message.
    :return: A MessageFromPcToController object, or None if the message is invalid.
    """
    if len(data) != ctypes.sizeof(MessageFromPcToController):
        print(f"Invalid message length: Got {len(data)} but expected {ctypes.sizeof(MessageFromPcToController)}")
        return None
    if data[0:2] != b'TW':
        print(f"Invalid prefix: Got {data[0:2]} but expected b'TW'")
        return None
    if data[2] != 1:  # Check API version
        print(f"Invalid API version: Got {data[2]} but expected 1")
        return None
    # Create a MessageFromPcToController object from the byte array
    message = MessageFromPcToController.from_buffer_copy(data)
    return message