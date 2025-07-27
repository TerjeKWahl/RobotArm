"""
This file handles higher level logic for controlling the robot arm.
"""

from time import sleep
import asyncio
from RobotMessageBuilder import create_message_from_PC_to_controller, MovementMode, JointAngles

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
send = None  # This will be set later when the Bluetooth connection is established.

async def control_robot_arm(send_function):
    global send
    send = send_function
    print("Starting to control the robot arm...")

    # Loop 3 times:
    for _ in range(3):

        desired_angles.gripper = 0
        desired_angles.wrist = 90
        desired_angles.underarm = 90
        desired_angles.elbow = -20
        desired_angles.overarm = -45
        desired_angles.shoulder_forward = 35
        desired_angles.shoulder_out = 35
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
        desired_angles.shoulder_forward = 0
        desired_angles.shoulder_out = 0
        message = create_message_from_PC_to_controller(
            movement_mode=MovementMode.MOVE_FAST,
            desired_angles=desired_angles,
            last_known_angles=last_known_angles
        )
        print("Sending message to the hub:", message)
        await send(message)
        print("Message sent to the hub.")
        await asyncio.sleep(5)  # Wait to let the hub process the command.

    message = create_message_from_PC_to_controller(
        movement_mode=MovementMode.RETURN_TO_ZERO,
        desired_angles=desired_angles,
        last_known_angles=last_known_angles
    )
    print("Sending message to the hub:", message)
    await send(message)
    print("Message sent to the hub.")


    print("Exiting the control_robot_arm function.")
