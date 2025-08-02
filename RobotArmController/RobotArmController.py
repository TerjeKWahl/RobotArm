"""
This file handles higher level logic for controlling the robot arm.
"""

from time import sleep
import asyncio
import threading
from RobotMessageManager import create_message_from_PC_to_controller, MovementMode, JointAngles, parse_message_from_controller_to_PC, InformationSource, UNKNOWN_ANGLE

desired_angles = JointAngles(
    gripper=0,
    wrist=0,
    underarm=0,
    elbow=0,
    overarm=0,
    shoulder_forward=0,
    shoulder_out=0
)
last_known_angles = JointAngles(
    gripper=0,
    wrist=0,
    underarm=0,
    elbow=0,
    overarm=0,
    shoulder_forward=0,
    shoulder_out=0
)

# define a function pointer to send data to the hub
send_to_lego_hubs = None  # This will be set later when the Bluetooth connection is established.

# Thread lock for protecting shared state
_state_lock = threading.Lock()

async def control_robot_arm(send_to_lego_hubs_function):
    global send_to_lego_hubs
    send_to_lego_hubs = send_to_lego_hubs_function
    print("Starting to control the robot arm...")

    underarm = 60
    underarm_twist = 40
    wrist = -60
    joint_angles_playlist = [ # Wave with underarm. First 6 values are joint angles, and the last value is the duration in milliseconds.
        [  0,   0,   0,   0,                       0,        0, 2000],
        [ 45,  15,   0,  13,                underarm,    wrist, 2000],
        [ 45,  15, -25,  13, underarm-underarm_twist,    wrist, 1000],
        [ 45,   0,  25,  13, underarm+underarm_twist,    wrist, 800],
        [ 45,  15, -25,  13, underarm-underarm_twist,    wrist, 800],
        [ 45,   0,  25,  13, underarm+underarm_twist,    wrist, 800],
        [ 45,  15, -25,  13, underarm-underarm_twist,    wrist, 800],
        [ 45,   0,  25,  13, underarm+underarm_twist,    wrist, 800],
        [ 45,   0,   0,  13,                      90,    wrist, 800],
        [ 45,   0,   0,  13,                      90, wrist-45, 500],
        [ 45,   0,   0,  13,                      90, wrist+ 0, 500],
        [ 45,   0,   0,  13,                      90, wrist-45, 500],
        [ 45,   0,   0,  13,                      90, wrist+ 0, 500],
        [  0,   0,   0,   0,                       0,        0, 1000],
        ]
    
    
    # Send the angles to the robot arm with a delay between each set of angles:
    for _ in range(3):
        for angles in joint_angles_playlist:
            desired_angles.gripper = 0
            desired_angles.wrist = angles[5]
            desired_angles.underarm = angles[4]
            desired_angles.elbow = angles[3]
            desired_angles.overarm = angles[2]
            desired_angles.shoulder_out = angles[1]
            desired_angles.shoulder_forward = angles[0]

            message = create_message_from_PC_to_controller(
                movement_mode=MovementMode.MOVE_FAST,
                desired_angles=desired_angles,
                last_known_angles=last_known_angles
            )
            print("Sending message to the hub:", message)
            await send_to_lego_hubs(message)
            print("Message sent to the hub.")

            wait_time_ms = angles[6]
            await asyncio.sleep(wait_time_ms / 1000)  # Wait to let the hubs process the command.

    """
    # Loop 3 times a small demo movement:
    for _ in range(3):

        desired_angles.gripper = 0
        desired_angles.wrist = 90
        desired_angles.underarm = 90
        desired_angles.elbow = -20
        desired_angles.overarm = -45
        desired_angles.shoulder_out = 35
        desired_angles.shoulder_forward = 35
        message = create_message_from_PC_to_controller(
            movement_mode=MovementMode.MOVE_FAST,
            desired_angles=desired_angles,
            last_known_angles=last_known_angles
        )
        print("Sending message to the hub:", message)
        await send(message)
        print("Message sent to the hub.")
        await asyncio.sleep(5)  # Wait to let the hub process the command.

        # Now let's change the angles and send another message
        desired_angles.gripper = 0
        desired_angles.wrist = 0
        desired_angles.underarm = 0
        desired_angles.elbow = 0
        desired_angles.overarm = 0
        desired_angles.shoulder_out = 0
        desired_angles.shoulder_forward = 0
        message = create_message_from_PC_to_controller(
            movement_mode=MovementMode.MOVE_FAST,
            desired_angles=desired_angles,
            last_known_angles=last_known_angles
        )
        print("Sending message to the hub:", message)
        await send(message)
        print("Message sent to the hub.")
        await asyncio.sleep(5)  # Wait to let the hub process the command.
    """

    message = create_message_from_PC_to_controller(
        movement_mode=MovementMode.RETURN_TO_ZERO,
        desired_angles=desired_angles,
        last_known_angles=last_known_angles
    )
    print("Sending message to the hub:", message)
    await send_to_lego_hubs(message)
    print("Message sent to the hub.")


    print("Exiting the control_robot_arm function.")

def handle_message_from_controller_to_PC(data: bytes):
    """
    Handle incoming messages from robot arm controllers (Lego hubs or Arduino).
    Updates the last_known_angles based on the received data.
    
    :param data: Raw bytes received from a controller
    """
    global last_known_angles
    
    # Parse the message
    message = parse_message_from_controller_to_PC(data)
    if message is None:
        print("Failed to parse message from controller")
        return
    
    # Print error if any
    if message.error_code != 0:
        print(f"Controller reported error code: {message.error_code}")
    
    # Update last_known_angles based on the information source
    with _state_lock:
        if message.information_source == InformationSource.LEGO_HUB_1_LOWER_ARM:
            # Hub 1 controls lower arm: wrist, underarm, elbow, overarm
            if message.current_angles.wrist != UNKNOWN_ANGLE:
                last_known_angles.wrist = message.current_angles.wrist
            if message.current_angles.underarm != UNKNOWN_ANGLE:
                last_known_angles.underarm = message.current_angles.underarm
            if message.current_angles.elbow != UNKNOWN_ANGLE:
                last_known_angles.elbow = message.current_angles.elbow
            if message.current_angles.overarm != UNKNOWN_ANGLE:
                last_known_angles.overarm = message.current_angles.overarm
                
        elif message.information_source == InformationSource.LEGO_HUB_2_SHOULDER:
            # Hub 2 controls shoulder: shoulder_out, shoulder_forward
            if message.current_angles.shoulder_out != UNKNOWN_ANGLE:
                last_known_angles.shoulder_out = message.current_angles.shoulder_out
            if message.current_angles.shoulder_forward != UNKNOWN_ANGLE:
                last_known_angles.shoulder_forward = message.current_angles.shoulder_forward
                
        elif message.information_source == InformationSource.ARDUINO_GRIPPER:
            # Arduino controls gripper
            if message.current_angles.gripper != UNKNOWN_ANGLE:
                last_known_angles.gripper = message.current_angles.gripper
                
        else:
            print(f"Unknown information source: {message.information_source}")
            return
    
    print(f"Updated angles from {message.information_source}: "
          f"gripper={last_known_angles.gripper}, "
          f"wrist={last_known_angles.wrist}, "
          f"underarm={last_known_angles.underarm}, "
          f"elbow={last_known_angles.elbow}, "
          f"overarm={last_known_angles.overarm}, "
          f"shoulder_out={last_known_angles.shoulder_out}, "
          f"shoulder_forward={last_known_angles.shoulder_forward}")
