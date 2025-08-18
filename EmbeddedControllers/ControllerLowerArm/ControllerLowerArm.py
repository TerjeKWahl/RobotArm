"""
Controls the lower 4 axis on a robot arm (wrist, underarm twist, elbow, overarm twist), 
and communicates with a PC app.
Built using Pybricks (see pybricks.com for more info).
"""

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.tools import wait, StopWatch
from usys import stdin, stdout
from uselect import poll, POLLIN
from micropython import kbd_intr
from RobotMessageBuilderEmbedded import JointAngles, MessageFromPCToController, \
     parse_message_from_PC_to_controller, create_message_from_controller_to_PC, \
     INFORMATION_SOURCE_HUB_1, REC_MSG_LENGTH, \
     MOVEMENT_MODE_MOVE_FAST, MOVEMENT_MODE_RUN_TO_TARGET, \
     MOVEMENT_MODE_CALIBRATION, MOVEMENT_MODE_RETURN_TO_ZERO_AND_EXIT

PRINT_DEBUG_INFO = False # Set to True to enable debug messages via printing to stdout. This will interfere with Bluetooth communication that also uses stdout!
SEND_PERIOD_MS = 20 # How often to send current angles to the PC app

is_continuous_tracking = False
hub = TechnicHub()
stopwatch = StopWatch()
stopwatch.resume()
receive_buffer = bytearray(REC_MSG_LENGTH)

# Motor definitions, including gearing ratios:
wrist    = a = Motor(Port.A, positive_direction=Direction.CLOCKWISE,        reset_angle=False, gears=[[20, 12], [ 1, 12], [12, 28]]) # Wrist left/right (as seen when right thumb is up)
underarm = b = Motor(Port.B, positive_direction=Direction.CLOCKWISE       , reset_angle=False, gears=[[ 8, 24], [12, 20], [ 8, 28]]) # Underarm twist
elbow    = c = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[[ 1, 24], [20, 28]]) # Elbow
overarm  = d = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[[ 8, 24], [12, 60]]) # Overarm twist

last_known_angles = JointAngles(set_all_unknown = True)
last_send_time = 0
last_receive_time = 0

def print_debug(string):
    if PRINT_DEBUG_INFO:
        print(string)


def send_current_angles_to_pc(last_known_angles: JointAngles):
    """
    Reads current motor positions and sends them to PC via stdout.
    Updates the relevant values in last_known_angles.
    """
    # Update known angles from motor positions
    known_angle_wrist = wrist.angle()
    known_angle_underarm = underarm.angle()
    known_angle_elbow = elbow.angle()
    known_angle_overarm = overarm.angle()
    last_known_angles.wrist = known_angle_wrist
    last_known_angles.underarm = known_angle_underarm
    last_known_angles.elbow = known_angle_elbow
    last_known_angles.overarm = known_angle_overarm

    # Create JointAngles with only lower arm angles known
    current_angles = JointAngles(set_all_unknown=True)
    current_angles.wrist = known_angle_wrist
    current_angles.underarm = known_angle_underarm
    current_angles.elbow = known_angle_elbow
    current_angles.overarm = known_angle_overarm

    # Create and send reply message
    reply_message = create_message_from_controller_to_PC(INFORMATION_SOURCE_HUB_1, current_angles)
    stdout.buffer.write(reply_message)
    stdout.flush()  # Ensure the message is sent immediately



def track_with_compensated_angles(desired_angles : JointAngles, last_known_angles : JointAngles):
    """
    Adjusts the motor targets based on the desired angles and the last known angles.
    This compensates for any differences in the arm's position.
    Assumes that the last known angles are just read and therefore relatively accurate.
    """
    # Calculate the compensation of underarm due to elbow angle
    underarm_compensation = last_known_angles.elbow * (28/12) * (12/20) * (8/28)

    # Calculate the compensation of wrist due to underarm and elbow angles
    wrist_compensation_due_to_underarm = last_known_angles.underarm * (12/28)
    wrist_compensation_due_to_elbow = last_known_angles.elbow * (28/12) * (12/28)

    # Track the compensated angles of the motors
    wrist.track_target(desired_angles.wrist + wrist_compensation_due_to_underarm + wrist_compensation_due_to_elbow)
    underarm.track_target(desired_angles.underarm + underarm_compensation)
    elbow.track_target(desired_angles.elbow)
    overarm.track_target(desired_angles.overarm)



print_debug("Control limits (speed, acceleration, torque):")
print_debug(a.control.limits())
print_debug(b.control.limits())
print_debug(c.control.limits())
print_debug(d.control.limits())

print_debug("Control target tolerances (speed, position):")
print_debug(a.control.target_tolerances())
print_debug(b.control.target_tolerances())
print_debug(c.control.target_tolerances())
print_debug(d.control.target_tolerances())

# Registering stdin for polling. This allows us to wait for incoming data without blocking:
kbd_intr(-1) # Disables keyboard interupt on stdin
stdin_event_monitor = poll()
stdin_event_monitor.register(stdin, POLLIN)

# Reset position:
hub.light.blink(Color.ORANGE,[600,200]) # Orange blinking indicates calibrating or resetting to zero angles
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)
c.run_target(1500, 0, then=Stop.HOLD, wait=True)
d.run_target(1500, 0, then=Stop.HOLD, wait=True)
#a.reset_angle(0)
#b.reset_angle(0)
#c.reset_angle(0)
#d.reset_angle(0)
#wait(2000)

# Dummy sequences for testing:
"""
wrist.run_target(1500, 180, then=Stop.HOLD, wait=True)
wait(1000)
wrist.run_target(1500, 0, then=Stop.HOLD, wait=True)
wait(1000)
wrist.run_target(1500, -180, then=Stop.HOLD, wait=True)
wait(1000)
wrist.run_target(1500, 0, then=Stop.HOLD, wait=True)
wait(1000)
"""
"""
underarm.run_target(1500, 180, then=Stop.HOLD, wait=True)
wait(1000)
underarm.run_target(1500, 0, then=Stop.HOLD, wait=True)
wait(1000)
underarm.run_target(1500, -180, then=Stop.HOLD, wait=True)
wait(1000)
underarm.run_target(1500, 0, then=Stop.HOLD, wait=True)
wait(1000)
"""
"""
wrist.run_target(1500, 90*12/28, then=Stop.HOLD, wait=False) # Compensate wrist when underarm is moving
underarm.run_target(1500, 90, then=Stop.HOLD, wait=True)
wait(1000)
wrist.run_target(1500, 0*12/28, then=Stop.HOLD, wait=False) # Compensate wrist when underarm is moving
underarm.run_target(1500, 0, then=Stop.HOLD, wait=True)
wait(1000)
wrist.run_target(1500, -90*12/28, then=Stop.HOLD, wait=False) # Compensate wrist when underarm is moving
underarm.run_target(1500, -90, then=Stop.HOLD, wait=True)
wait(1000)
wrist.run_target(1500, 0*12/28, then=Stop.HOLD, wait=False) # Compensate wrist when underarm is moving
underarm.run_target(1500, 0, then=Stop.HOLD, wait=True)
wait(1000)
"""
"""
#wait(1000)
#elbow.run_target(1500, 10, then=Stop.HOLD, wait=True)
#wait(1000)
#elbow.run_target(1500, 0, then=Stop.HOLD, wait=True)
wait(1000)
desired_elbow_angle = -80
desired_underarm_angle = -90
underarm_compensation = desired_elbow_angle*(28/12)*(12/20)*(8/28)
underarm.run_target(1500, desired_underarm_angle + underarm_compensation, then=Stop.HOLD, wait=False) # Compensate underarm when elbow is moving
wrist_compensation_due_to_underarm = desired_underarm_angle*(12/28)
wrist_compensation_due_to_elbow = desired_elbow_angle*(28/12)*(12/28)
wrist.run_target(1500, wrist_compensation_due_to_underarm + wrist_compensation_due_to_elbow , then=Stop.HOLD, wait=False) # Compensate wrist when underarm and elbow is moving
elbow.run_target(1500, desired_elbow_angle, then=Stop.HOLD, wait=True)
wait(2000)
underarm.run_target(1500, 0, then=Stop.HOLD, wait=False) # Compensate underarm when elbow is moving
wrist.run_target(1500, 0, then=Stop.HOLD, wait=False) # Compensate wrist when underarm and elbow is moving
elbow.run_target(1500, 0, then=Stop.HOLD, wait=True)
"""
"""
wrist.run_target(1500, 110, then=Stop.HOLD, wait=True)
wrist.run_target(1500, -110, then=Stop.HOLD, wait=True)
wrist.run_target(1500, 0, then=Stop.HOLD, wait=False)

#wrist.run_target(1500, -180*12/32, then=Stop.HOLD, wait=False) # Compensate a when b is moving
underarm.run_target(1500, 120, then=Stop.HOLD, wait=True)
underarm.run_target(1500, -120, then=Stop.HOLD, wait=True)
underarm.run_target(1500, 0, then=Stop.HOLD, wait=False)
underarm.run_target(1500, 0, then=Stop.HOLD, wait=True)

elbow.run_target(1500, 23, then=Stop.HOLD, wait=True)
wait(1000)
elbow.run_target(1500, -20, then=Stop.HOLD, wait=True)
wait(1000)
elbow.run_target(1500, 0, then=Stop.HOLD, wait=False)

overarm.run_target(1500, 35, then=Stop.HOLD, wait=True)
overarm.run_target(1500, -120, then=Stop.HOLD, wait=True)
overarm.run_target(1500, 0, then=Stop.HOLD, wait=False)
#wait(2000)
"""

hub.light.blink(Color.VIOLET,[600,200]) # Violet blinking to indicate waiting for connection/data to begin.

# Let the remote program know we are ready for commands
stdout.buffer.write(b"rdy")

stopwatch.reset()

while True:
    is_timeout_already_expired = False
    # Check if data is available
    while not stdin_event_monitor.poll(0):
        current_time = stopwatch.time()
        if current_time - last_receive_time > 3000:
            if not is_timeout_already_expired:
                hub.light.blink(Color.VIOLET,[600,200]) # Violet blinking to indicate waiting for connection/data to begin or resume.
                is_timeout_already_expired = True
                # Move back to start position when no message is received for a while
                a.run_target(1500, 0, then=Stop.HOLD, wait=False)
                b.run_target(1500, 0, then=Stop.HOLD, wait=False)
                c.run_target(1500, 0, then=Stop.HOLD, wait=False)
                d.run_target(1500, 0, then=Stop.HOLD, wait=False)

        # Send last known angles every SEND_PERIOD_MS ms
        if current_time - last_send_time >= SEND_PERIOD_MS:
            send_current_angles_to_pc(last_known_angles)
            last_send_time = current_time
            if is_continuous_tracking:
                track_with_compensated_angles(message.desired_angles, last_known_angles)

        wait(1)

    # Read message from stdin
    num_bytes_read = stdin.buffer.readinto(receive_buffer, REC_MSG_LENGTH)
    print_debug(f"Read {num_bytes_read} bytes: {receive_buffer}")
    is_message_ok, message = parse_message_from_PC_to_controller(receive_buffer)
    
    current_time = stopwatch.time()

    # Decide what to do based on the command.
    if not is_message_ok:
        hub.light.on(Color.RED)
    else:
        hub.light.on(Color.GREEN)
        last_receive_time = current_time

        # Update last known angles except the angles that are owned by this controller:
        last_known_angles.gripper = message.last_known_angles.gripper
        last_known_angles.shoulder_out = message.last_known_angles.shoulder_out
        last_known_angles.shoulder_forward = message.last_known_angles.shoulder_forward

        is_continuous_tracking = False

        # Now move as instructed:
        if message.movement_mode == MOVEMENT_MODE_RETURN_TO_ZERO_AND_EXIT:
            break
        elif message.movement_mode == MOVEMENT_MODE_MOVE_FAST:
            is_continuous_tracking = True
            # Move all joints as fast as possible (for continuous tracking)
            track_with_compensated_angles(message.desired_angles, last_known_angles)
        elif message.movement_mode == MOVEMENT_MODE_RUN_TO_TARGET:
            # Move to target with smooth acceleration and deceleration (run to target)
            wrist.run_target(   1500, message.desired_angles.wrist, then=Stop.HOLD, wait=False)
            underarm.run_target(1500, message.desired_angles.underarm, then=Stop.HOLD, wait=False)
            elbow.run_target(   1500, message.desired_angles.elbow, then=Stop.HOLD, wait=False)
            overarm.run_target( 1500, message.desired_angles.overarm, then=Stop.HOLD, wait=False)
        elif message.movement_mode == MOVEMENT_MODE_CALIBRATION:
            print_debug("Calibration mode is not implemented.")
        else:
            print_debug(f"WARNING: Unknown movement mode: {message.movement_mode}. Ignoring the command.")

    # Send last known angles every SEND_PERIOD_MS ms, also here (in addition to in the receive wait loop) to
    # ensure we don't miss updates in case of frequent incoming messages
    if current_time - last_send_time >= SEND_PERIOD_MS:
        send_current_angles_to_pc(last_known_angles)            
        last_send_time = current_time
        if is_continuous_tracking:
            track_with_compensated_angles(message.desired_angles, last_known_angles)



# Return to zero angles and exit:
hub.light.blink(Color.ORANGE,[600,200])

# Clean up and finish by returning to starting position:
a.run_target(1500, 0, then=Stop.HOLD, wait=False)
b.run_target(1500, 0, then=Stop.HOLD, wait=False)
c.run_target(1500, 0, then=Stop.HOLD, wait=False)
d.run_target(1500, 0, then=Stop.HOLD, wait=False)
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)
c.run_target(1500, 0, then=Stop.HOLD, wait=True)
d.run_target(1500, 0, then=Stop.HOLD, wait=True)

print_debug("Done!")
