"""
This file handles higher level logic for controlling the robot arm.
It can run in different modes depending on a constant: Demo mode or VR following mode.
"""

from time import sleep
import asyncio
import threading
import numpy as np
from spatialmath import SE3
from RobotMessageManager import create_message_from_PC_to_controller, MovementMode, JointAngles, \
    parse_message_from_controller_to_PC, InformationSource, DistanceAndAngleOffset, UNKNOWN_ANGLE
from RunMode import RunMode
from Configuration import robot_arm, neutral_pose_SE3, RUN_MODE


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
desired_distance_and_angle_offset = DistanceAndAngleOffset(
    x_distance = 0,
    y_distance = 0,
    z_distance = 0,
    x_angle = 0,
    y_angle = 0,
    z_angle = 0
)

# define a function pointer to send data to the hub
send_to_lego_hubs = None  # This will be set later when the Bluetooth connection is established.

# Thread lock for protecting shared state
_state_lock = threading.Lock()


async def control_robot_arm(send_to_lego_hubs_function):
    global send_to_lego_hubs
    send_to_lego_hubs = send_to_lego_hubs_function
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
    print("The robot arm structure is:")
    print(robot_arm) # View the ETS (Elementary Transformation Sequence, where ET is a translation or rotation).
    print(f"The robot has {robot_arm.n} joints")
    print(f"The robot has {robot_arm.m} Elementary Transformations (ETs)")


    while True:
        # Calculate the desired angles based on the desired distance and angle offset of the robot arm.
        try:            
            desired_translation = SE3.Trans(
                desired_distance_and_angle_offset.x_distance,
                desired_distance_and_angle_offset.y_distance, 
                desired_distance_and_angle_offset.z_distance
            )            
            desired_rotation = SE3.RPY( # TODO: Fix/test ordering of rotations
                np.deg2rad(desired_distance_and_angle_offset.x_angle),
                np.deg2rad(desired_distance_and_angle_offset.y_angle),
                np.deg2rad(desired_distance_and_angle_offset.z_angle)
            )
            # Apply the offsets relative to the neutral pose
            desired_pose = neutral_pose_SE3 * desired_translation * desired_rotation

            current_pose_q = [last_known_angles.shoulder_forward, last_known_angles.shoulder_out, last_known_angles.overarm, last_known_angles.elbow, last_known_angles.underarm, last_known_angles.wrist]
            print(f"\nDesired distance and angle offset: {desired_distance_and_angle_offset}")

            mask_priority = np.array([2, 2, 2, 0, 1, 1])  # We want to prioritize the position over the orientation in the IK solution, 
                                                          # and for orientation we don't prioritize rotation around the X axis (should be along the gripper "fingers" so not too important).

            # Calculate inverse kinematics to get joint angles
            joint_angles_solution = robot_arm.ik_LM(Tep=desired_pose, q0=current_pose_q, mask=mask_priority, joint_limits=True, ilimit=300, slimit=1000)

            solution_q, was_successful, iterations, num_searches, residual = joint_angles_solution
            #print(f"solution_q: {solution_q}")
            joint_angles_deg = np.round(np.rad2deg(solution_q)).astype(int)
            print(f"Inverse kinematics solution {'SUCCESS' if was_successful else 'FAILED'}")
            #print(f"Termination status: {reason} (1=success, 0=failure)")
            #print(f"Solution found in {iterations} iterations")
            #print(f"Number of searches performed: {num_searches}")
            #print(f"Final residual error: {residual:.6f}")
            print(f"Solution angles (degrees): {joint_angles_deg}")
            #print(f"Rotation around the Z, Y and Z axes for the desired_pose: {desired_pose.eul(unit='deg')}")
            #solution_se3 = robot_arm.fkine(solution_q)
            #print(f"Rotation around the Z, Y and Z axes for the solution:     {solution_se3.eul(unit='deg')}")

            if was_successful:
                # Update desired angles
                desired_angles.shoulder_forward = joint_angles_deg[0]
                desired_angles.shoulder_out = joint_angles_deg[1]
                desired_angles.overarm = joint_angles_deg[2]
                desired_angles.elbow = joint_angles_deg[3]
                desired_angles.underarm = joint_angles_deg[4]
                desired_angles.wrist = joint_angles_deg[5]
            else:
                print("Failed to find inverse kinematics solution, using previous angles")
                
        except Exception as e:
            print(f"Error in inverse kinematics calculation: {e}")
            print("Using previous desired angles")

        message = create_message_from_PC_to_controller(
            movement_mode=MovementMode.MOVE_FAST,
            desired_angles=desired_angles,
            last_known_angles=last_known_angles
        )
        #print("Sending message to the hub:", message)
        await send_to_lego_hubs(message)
        #print("Message sent to the hub.")

        wait_time_ms = 500 # TODO: Make faster
        await asyncio.sleep(wait_time_ms / 1000)


async def control_robot_arm_demo_mode():
    """
    Control the robot arm in demo mode.
    This function sends a series of joint angles to the robot arm to demonstrate its capabilities.
    """
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
