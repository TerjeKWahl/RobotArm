"""
Helper class that defines classes representing angles and messages, and defines
functions for parsing and creating messages, and sanitizing joint angles for possible
illegal values.
"""
from ustruct import unpack, pack

REC_MSG_LENGTH = 18
UNKNOWN_ANGLE = -128

# Movement mode constants
MOVEMENT_MODE_MOVE_FAST = 1
MOVEMENT_MODE_RUN_TO_TARGET = 2
MOVEMENT_MODE_CALIBRATION = 3  # Not used so far
MOVEMENT_MODE_RETURN_TO_ZERO_AND_EXIT = 4

# Information source constants
INFORMATION_SOURCE_HUB_1 = 10  # Lego Technic Hub 1 (lower arm)
INFORMATION_SOURCE_HUB_2 = 11  # Lego Technic Hub 2 (shoulder)

class JointAngles:
    def __init__(self, set_all_unknown = False):
        if not set_all_unknown:
            self.gripper = 0
            self.wrist = 0
            self.underarm = 0
            self.elbow = 0
            self.overarm = 0
            self.shoulder_out = 0
            self.shoulder_forward = 0
        else:
            self.gripper = UNKNOWN_ANGLE # According to the API spec, -128 means "unknown"
            self.wrist = UNKNOWN_ANGLE
            self.underarm = UNKNOWN_ANGLE
            self.elbow = UNKNOWN_ANGLE
            self.overarm = UNKNOWN_ANGLE
            self.shoulder_out = UNKNOWN_ANGLE
            self.shoulder_forward = UNKNOWN_ANGLE


class MessageFromPCToController:
    def __init__(self):
        self.prefix_t = 0
        self.prefix_w = 0
        self.api_version = 0
        self.movement_mode = 0
        self.desired_angles = JointAngles()
        self.last_known_angles = JointAngles()


class MessageFromControllerToPC:
    def __init__(self, information_source: int, current_angles: JointAngles):
        self.prefix_t = 84  # ASCII "T"
        self.prefix_w = 87  # ASCII "W"
        self.api_version = 1
        self.information_source = information_source
        self.error_code = 0  # 0 means no error
        self.current_angles = current_angles



def __sanitize_angles(joint_angles: JointAngles):
    """
    Checks for illegal angles, and corrects them if they are too large or too small.
    Assumes that measures have already been taken (in the PC side) to avoid crashing into
    the table/desk or the robot torso.

    :param joint_angles: The joint angles to sanitize, in degrees.
    """

    # Joint angle limits copied from Configuration.py:
    joint_limits_deg = [
        [-35, 45],    # Joint 1 (shoulder forward)
        [-10, 35],    # Joint 2 (shoulder out/up): 
        [-120, 120],  # Joint 3 (overarm):        
        [-85, 23],    # Joint 4 (elbow):          
        [-120, 120],  # Joint 5 (underarm):       
        [-120, 120],  # Joint 6 (wrist):           - Wrist down/up (as seen when right thumb is up)
        [-60, 60]     # Joint 7 (wrist other way): - This joint is not physically on the robot (yet), but greatly improves IK solutions and control stability as a "virtual" joint
    ]

    if joint_angles.wrist > joint_limits_deg[5][1]:
        print("Warning: wrist value too large.")
        joint_angles.wrist = joint_limits_deg[5][1]
    if joint_angles.wrist < joint_limits_deg[5][0]:
        print("Warning: wrist value too small.")
        joint_angles.wrist = joint_limits_deg[5][0]

    if joint_angles.underarm > joint_limits_deg[4][1]:
        print("Warning: underarm value too large.")
        joint_angles.underarm = joint_limits_deg[4][1]
    if joint_angles.underarm < joint_limits_deg[4][0]:
        print("Warning: underarm value too small.")
        joint_angles.underarm = joint_limits_deg[4][0]

    if joint_angles.elbow > joint_limits_deg[3][1]:
        print("Warning: elbow value too large.")
        joint_angles.elbow = joint_limits_deg[3][1]
    if joint_angles.elbow < joint_limits_deg[3][0]:
        print("Warning: elbow value too small.")
        joint_angles.elbow = joint_limits_deg[3][0]

    if joint_angles.overarm > joint_limits_deg[2][1]:
        print("Warning: overarm value too large.")
        joint_angles.overarm = joint_limits_deg[2][1]
    if joint_angles.overarm < joint_limits_deg[2][0]:
        print("Warning: overarm value too small.")
        joint_angles.overarm = joint_limits_deg[2][0]

    if joint_angles.shoulder_out > joint_limits_deg[1][1]:
        print("Warning: shoulder_out value too large.")
        joint_angles.shoulder_out = joint_limits_deg[1][1]
    if joint_angles.shoulder_out < joint_limits_deg[1][0]:
        print("Warning: shoulder_out value too small.")
        joint_angles.shoulder_out = joint_limits_deg[1][0]

    if joint_angles.shoulder_forward > joint_limits_deg[0][1]:
        print("Warning: shoulder_forward value too large.")
        joint_angles.shoulder_forward = joint_limits_deg[0][1]
    if joint_angles.shoulder_forward < joint_limits_deg[0][0]:
        print("Warning: shoulder_forward value too small.")
        joint_angles.shoulder_forward = joint_limits_deg[0][0]



def parse_message_from_PC_to_controller(data: bytes) -> tuple[bool, MessageFromPCToController]:
    """
    Parses a message from the PC to the robot arm controller.

    :param data: The byte array representing the message.
    :return: A tuple of a bool and a MessageFromPcToController instance. The bool value:
             False if message is not as expected, or True if message is valid.
    """
    message = MessageFromPCToController()

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
    message.desired_angles.shoulder_out, \
    message.desired_angles.shoulder_forward, \
    message.last_known_angles.gripper, \
    message.last_known_angles.wrist, \
    message.last_known_angles.underarm, \
    message.last_known_angles.elbow, \
    message.last_known_angles.overarm, \
    message.last_known_angles.shoulder_out, \
    message.last_known_angles.shoulder_forward = unpack(format_string, data)

    #print(f"t={message.prefix_t} w={message.prefix_w} sf={message.desired_angles.shoulder_forward} so={message.desired_angles.shoulder_out}")

    # Always sanitize the received data to check for invalid angles:
    __sanitize_angles(message.desired_angles)

    if (message.prefix_t != 84) or (message.prefix_w != 87): # 84 is ASCII "T" and 87 is ASCII "W"
        print(f"Invalid prefix: Got {data[0:2]} but expected b'TW'")
        return False, message
    if message.api_version != 1:
        print(f"Invalid API version: Got {data[2]} but expected 1")
        return False, message
    if message.movement_mode not in (MOVEMENT_MODE_MOVE_FAST, MOVEMENT_MODE_RUN_TO_TARGET, \
                                     MOVEMENT_MODE_CALIBRATION, MOVEMENT_MODE_RETURN_TO_ZERO_AND_EXIT):
        print(f"Invalid movement mode: Got {message.movement_mode} but expected one of { \
            MOVEMENT_MODE_MOVE_FAST}, {MOVEMENT_MODE_RUN_TO_TARGET}, {MOVEMENT_MODE_CALIBRATION}, { \
            MOVEMENT_MODE_RETURN_TO_ZERO_AND_EXIT}")
        return False, message

    return True, message



def create_message_from_controller_to_PC(information_source: int, current_angles: JointAngles) -> bytes:
    """
    Creates a binary message from controller to PC.
    For now, the error_code is always 0 (no error).
    
    :param information_source: Source identifier (10=Hub1, 11=Hub2)
    :param current_angles: Current joint angles
    :return: Binary message as bytes
    """
    message = MessageFromControllerToPC(information_source, current_angles)
    
    format_string = "<bbbbbbbbbbbb"  # 12 signed bytes
    
    return pack(format_string,
                message.prefix_t,
                message.prefix_w, 
                message.api_version,
                message.information_source,
                message.error_code,
                message.current_angles.gripper,
                message.current_angles.wrist,
                message.current_angles.underarm,
                message.current_angles.elbow,
                message.current_angles.overarm,
                message.current_angles.shoulder_out,
                message.current_angles.shoulder_forward)
