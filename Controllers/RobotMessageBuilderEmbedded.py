"""
Helper class that defines classes representing angles and messages, and defines
functions for parsing and creating messages, and sanitizing joint angles for possible
illegal values.
"""
from ustruct import unpack

REC_MSG_LENGTH = 18
UNKNOWN_ANGLE = -128

class JointAngles:
    def __init__(self, set_all_unknown = False):
        if not set_all_unknown:
            self.gripper = 0
            self.wrist = 0
            self.underarm = 0
            self.elbow = 0
            self.overarm = 0
            self.shoulder_forward = 0
            self.shoulder_out = 0
        else:
            self.gripper = UNKNOWN_ANGLE # According to the API spec, -128 means "unknown"
            self.wrist = UNKNOWN_ANGLE
            self.underarm = UNKNOWN_ANGLE
            self.elbow = UNKNOWN_ANGLE
            self.overarm = UNKNOWN_ANGLE
            self.shoulder_forward = UNKNOWN_ANGLE
            self.shoulder_out = UNKNOWN_ANGLE


class MessageFromPcToController:
    def __init__(self):
        self.prefix_t = 0
        self.prefix_w = 0
        self.api_version = 0
        self.movement_mode = 0
        self.desired_angles = JointAngles()
        self.last_known_angles = JointAngles()


def sanitize_angles(joint_angles: JointAngles):
    """
    Checks for illegal angles, and corrects them if they are too large or too small.
    """
    if joint_angles.shoulder_forward > 45:
        print("Warning: shoulder_forward value too big.")
        joint_angles.shoulder_forward = 45
    if joint_angles.shoulder_forward < -10: # TODO: Make more liberal, but depending on overarm twist and elbow angle
        joint_angles.shoulder_forward = -10

    if joint_angles.shoulder_out > 35:
        joint_angles.shoulder_out = 35
    if joint_angles.shoulder_out < 0: # TODO: Make slightly more liberal
        joint_angles.shoulder_out = 0


def parse_message_from_PC_to_controller(data: bytes) -> tuple[bool, MessageFromPcToController]:
    """
    Parses a message from the PC to the robot arm controller.

    :param data: The byte array representing the message.
    :return: A tuple of a bool and a MessageFromPcToController instance. The bool value:
             False if message is not as expected, or True if message is valid.
    """
    message = MessageFromPcToController()

    if len(data) != REC_MSG_LENGTH:
        print(f"Invalid message length: Got {len(data)} but expected {REC_MSG_LENGTH}")
        return False, message

    format_string = "<bbbbbbbbbbbbbbbbbb" # 18 signed bytes
    message.prefix_t, message.prefix_w, message.api_version, message.movement_mode, \
    message.desired_angles.gripper, \
    message.desired_angles.wrist, \
    message.desired_angles.underarm, \
    message.desired_angles.elbow, \
    message.desired_angles.overarm, \
    message.desired_angles.shoulder_forward, \
    message.desired_angles.shoulder_out, \
    message.last_known_angles.gripper_last, \
    message.last_known_angles.wrist_last, \
    message.last_known_angles.underarm_last, \
    message.last_known_angles.elbow_last, \
    message.last_known_angles.overarm_last, \
    message.last_known_angles.shoulder_forward_last, \
    message.last_known_angles.shoulder_out_last = unpack(format_string, data)

    #print(f"t={message.prefix_t} w={message.prefix_w} sf={message.desired_angles.shoulder_forward} so={message.desired_angles.shoulder_out}")

    # Always sanitize the received data to check for invalid angles:
    sanitize_angles(message.desired_angles)

    if (message.prefix_t != 84) or (message.prefix_w != 87): # 84 is ASCII "T" and 87 is ASCII "W"
        print(f"Invalid prefix: Got {data[0:2]} but expected b'TW'")
        return False, message
    if message.api_version != 1:
        print(f"Invalid API version: Got {data[2]} but expected 1")
        return False, message

    return True, message
