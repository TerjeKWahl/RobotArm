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

    # Define pretty printing for debugging:
    def __repr__(self):
        return (f"Gripper: {self.gripper}, Wrist: {self.wrist}, Underarm: {self.underarm}, "
                f"Elbow: {self.elbow}, Overarm: {self.overarm}, Shoulder out: {self.shoulder_out}, "
                f"Shoulder forward: {self.shoulder_forward}")


# Define struct for 4x4 matrix (used in VR-PC communication):
class Matrix4x4(ctypes.Structure):
    _fields_ = [
        ("m00", ctypes.c_double), ("m01", ctypes.c_double), ("m02", ctypes.c_double), ("m03", ctypes.c_double),
        ("m10", ctypes.c_double), ("m11", ctypes.c_double), ("m12", ctypes.c_double), ("m13", ctypes.c_double),
        ("m20", ctypes.c_double), ("m21", ctypes.c_double), ("m22", ctypes.c_double), ("m23", ctypes.c_double),
        ("m30", ctypes.c_double), ("m31", ctypes.c_double), ("m32", ctypes.c_double), ("m33", ctypes.c_double)
    ]

    # Define pretty printing for debugging:
    def __repr__(self):
        return (f"[{self.m00:8.3f} {self.m01:8.3f} {self.m02:8.3f} {self.m03:8.3f}]\n"
                f"[{self.m10:8.3f} {self.m11:8.3f} {self.m12:8.3f} {self.m13:8.3f}]\n"
                f"[{self.m20:8.3f} {self.m21:8.3f} {self.m22:8.3f} {self.m23:8.3f}]\n"
                f"[{self.m30:8.3f} {self.m31:8.3f} {self.m32:8.3f} {self.m33:8.3f}]")


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
        ("reserved_1", ctypes.c_int8),
        ("reserved_2", ctypes.c_int8),
        ("reserved_3", ctypes.c_int8),
        ("matrix_4x4", Matrix4x4)
    ]


class MessageFromVRToPC(ctypes.Structure):
    _fields_ = [
        ("prefix_t", ctypes.c_char),
        ("prefix_k", ctypes.c_char),
        ("prefix_w", ctypes.c_char),
        ("api_version", ctypes.c_int8),
        ("information_source", ctypes.c_int8),
        ("reserved_1", ctypes.c_int8),
        ("reserved_2", ctypes.c_int8),
        ("reserved_3", ctypes.c_int8),
        ("matrix_4x4", Matrix4x4)
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

    # TODO: Remove when no longer necessary for testing:
    # Clip desired angles to valid range based on limits:
    desired_angles.gripper = max(-128, min(127, desired_angles.gripper))
    desired_angles.wrist = max(-110, min(110, desired_angles.wrist))
    desired_angles.underarm = max(-120, min(120, desired_angles.underarm))
    desired_angles.elbow = max(-20, min(13, desired_angles.elbow))
    desired_angles.overarm = max(-120, min(35, desired_angles.overarm))
    desired_angles.shoulder_out = max(0, min(35, desired_angles.shoulder_out))
    desired_angles.shoulder_forward = max(-10, min(45, desired_angles.shoulder_forward))

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


def create_message_from_PC_to_VR(last_known_pose_matrix_4x4: Matrix4x4) -> bytes:
    """
    Creates a message to send from the PC to the VR app.

    :param last_known_pose_matrix_4x4: The last known pose matrix (4x4) of the physical robot 
                                       arm's end-effector, in Unity/VR coordinate system
    :return: A byte array representing the message
    """
    message = MessageFromPcToVR(
        prefix_t = b'T',
        prefix_k = b'K',
        prefix_w = b'W',
        api_version = 1,
        information_source = InformationSource.PC_APP,
        reserved_1 = 0,  # Reserved for future use
        reserved_2 = 0,  # Reserved for future use
        reserved_3 = 0,  # Reserved for future use
        matrix_4x4 = last_known_pose_matrix_4x4
    )
    
    message_bytes = ctypes.string_at(ctypes.byref(message), ctypes.sizeof(message))
    return message_bytes


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
    
    # Create a MessageFromVRToPC object from the byte array
    # Matrix4x4 doubles are already in the correct byte order (no conversion needed)
    message = MessageFromVRToPC.from_buffer_copy(data)
    return message
