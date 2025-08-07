"""
This file handles higher level logic for controlling the robot arm.
It can run in different modes depending on a constant: Demo mode or VR following mode.
"""

import asyncio
import threading
from RobotMessageManager import create_message_from_PC_to_controller, parse_message_from_controller_to_PC, \
    create_message_from_PC_to_VR, parse_message_from_VR_to_PC, \
    MovementMode, JointAngles, InformationSource, Matrix4x4, UNKNOWN_ANGLE
from RunMode import RunMode
from Configuration import RUN_MODE
from InverseKinematicsHelper import get_desired_angles_from_VR_position_and_orientation_matrix, calculate_distance_and_angle_offset


desired_angles = JointAngles(
    gripper = 0,
    wrist = 0,
    underarm = 0,
    elbow = 0,
    overarm = 0,
    shoulder_forward = 0,
    shoulder_out = 0
)
last_known_angles = JointAngles(
    gripper = UNKNOWN_ANGLE,
    wrist = UNKNOWN_ANGLE,
    underarm = UNKNOWN_ANGLE,
    elbow = UNKNOWN_ANGLE,
    overarm = UNKNOWN_ANGLE,
    shoulder_forward = UNKNOWN_ANGLE,
    shoulder_out = UNKNOWN_ANGLE
)
last_known_vr_unity_matrix_4x4 = None  # Last known position and orientation of the robot arm in the VR app (Unity).

# define function pointers to send data to the Lego hubs (Bluetooth) and VR app (UDP).
send_to_lego_hubs = None  # This will be set later when the Bluetooth connection is established.
send_to_VR = None         # This will be set later from the caller of control_robot_arm() (main program).

# Thread lock for protecting shared state
_state_lock = threading.Lock()



async def control_robot_arm(send_to_lego_hubs_function, send_to_VR_function):
    global send_to_lego_hubs, send_to_VR
    send_to_lego_hubs = send_to_lego_hubs_function
    send_to_VR = send_to_VR_function
    print("Starting to control the robot arm...")
    if RUN_MODE == RunMode.DEMO_MODE:
        await control_robot_arm_demo_mode()
    elif RUN_MODE == RunMode.VR_FOLLOWING_MODE:
        await control_robot_arm_vr_following_mode()
    else:
        print(f"ERROR: Unknown run mode: {RUN_MODE}. Please set RUN_MODE to either RunMode.DEMO_MODE or RunMode.VR_FOLLOWING_MODE.")



async def control_robot_arm_vr_following_mode():
    """
    Control the robot arm in VR following mode.
    This function will listen for messages from the VR system, calculate desired joint angles
    and send them to the robot arm.
    """
    global desired_angles, last_known_angles
    print("Running in VR following mode.")

    send_to_lego_hubs_task = None  # Task for sending messages to the Lego hubs
    send_to_VR_task = None         # Task for sending messages to the VR app
    while True:
        if last_known_vr_unity_matrix_4x4 is not None:
            # Try to calculate new desired angles every time a message is received from the VR app:
            new_desired_angles = get_desired_angles_from_VR_position_and_orientation_matrix(last_known_angles, last_known_vr_unity_matrix_4x4)
            if new_desired_angles is None:
                #print("Failed to calculate desired angles from VR position and orientation matrix")
                pass
            else:
                with _state_lock:
                    desired_angles = new_desired_angles
                print(f"Updated desired angles from VR gripper position: {desired_angles}")

        message_to_controller = create_message_from_PC_to_controller(
            movement_mode=MovementMode.MOVE_FAST,
            desired_angles=desired_angles,
            last_known_angles=last_known_angles
        )
        #print("Sending message to the hub:", message)
        if send_to_lego_hubs_task is not None and not send_to_lego_hubs_task.done():
            print("Waiting for previous send_to_lego_hubs_task to finish...")
            await send_to_lego_hubs_task
        send_to_lego_hubs_task = asyncio.create_task(send_to_lego_hubs(message_to_controller))
        #print("Message sent to the hub.")

        # TODO: Send messages to VR not all the time, but on regular intervals.
        last_known_distance_and_angle_offset = await calculate_distance_and_angle_offset(last_known_angles)
        print(f"Calculated last known distance and angle offset: {last_known_distance_and_angle_offset}")
        message_to_VR = create_message_from_PC_to_VR(
            is_connection_to_controllers_ok = True,
            last_known_distance_and_angle_offset = last_known_distance_and_angle_offset,
            last_known_angles = last_known_angles
        )

        if send_to_VR_task is not None and not send_to_VR_task.done():
            print("Waiting for previous send_to_VR_task to finish...")
            await send_to_VR_task
        send_to_VR_task = asyncio.create_task(send_to_VR(message_to_VR))

        wait_time_ms = 20 # TODO: Make faster and move this to a configuration constant.
        await asyncio.sleep(wait_time_ms / 1000)




async def control_robot_arm_demo_mode():
    """
    Control the robot arm in demo mode.
    This function sends a series of joint angles to the robot arm to demonstrate its capabilities.
    """
    underarm = -60
    underarm_twist = -40
    wrist = -60
    joint_angles_playlist = [ # Wave with underarm. First 6 values are joint angles, and the last value is the duration in milliseconds.
        [  0,   0,   0,   0,                       0,        0, 1000],
        [ 45,  15,   0,  13,                underarm,    wrist, 2000],
        [ 45,  15, -25,  13, underarm-underarm_twist,    wrist, 1000],
        [ 45,   0,  25,  13, underarm+underarm_twist,    wrist, 800],
        [ 45,  15, -25,  13, underarm-underarm_twist,    wrist, 800],
        [ 45,   0,  25,  13, underarm+underarm_twist,    wrist, 800],
        [ 45,  15, -25,  13, underarm-underarm_twist,    wrist, 800],
        [ 45,   0,  25,  13, underarm+underarm_twist,    wrist, 800],
        [ 45,   0,   0,  13,                     -90,    wrist, 800],
        [ 45,   0,   0,  13,                     -90, wrist-45, 500],
        [ 45,   0,   0,  13,                     -90, wrist+ 0, 500],
        [ 45,   0,   0,  13,                     -90, wrist-45, 500],
        [ 45,   0,   0,  13,                     -90, wrist+ 0, 500],
        [  0,   0,   0,   0,                       0,        0, 2000],
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
                movement_mode=MovementMode.RUN_TO_TARGET,
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
            movement_mode=MovementMode.RUN_TO_TARGET,
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
            movement_mode=MovementMode.RUN_TO_TARGET,
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
    """
    print(f"Updated angles from {message.information_source}: "
          f"gripper={last_known_angles.gripper}, "
          f"wrist={last_known_angles.wrist}, "
          f"underarm={last_known_angles.underarm}, "
          f"elbow={last_known_angles.elbow}, "
          f"overarm={last_known_angles.overarm}, "
          f"shoulder_out={last_known_angles.shoulder_out}, "
          f"shoulder_forward={last_known_angles.shoulder_forward}")
    """


def handle_message_from_VR_to_PC(data: bytes):
    """
    Handle incoming messages from VR app.
    Updates the desired_angles based on the received data.
    
    :param data: Raw bytes received from VR app
    """
    global last_known_vr_unity_matrix_4x4

    # Parse the message
    message = parse_message_from_VR_to_PC(data)
    if message is None:
        print("Failed to parse message from VR app")
        return

    #print(f"Received message from VR app.")
    #print(f"Received message from VR app with the following 4x4 matrix: \n{message.matrix_4x4}")

    with _state_lock:
        last_known_vr_unity_matrix_4x4 = message.matrix_4x4
