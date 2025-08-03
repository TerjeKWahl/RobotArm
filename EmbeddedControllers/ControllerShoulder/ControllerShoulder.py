"""
Controls the first two shoulder joints on a robot arm, and communicates with a PC app.
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
     parse_message_from_PC_to_controller, REC_MSG_LENGTH, MOVEMENT_MODE_RETURN_TO_ZERO_AND_EXIT, \
     INFORMATION_SOURCE_HUB_2, create_message_from_controller_to_PC

PRINT_DEBUG_INFO = False # Set to True to enable debug messages via printing to stdout. This will interfere with Bluetooth communication that also uses stdout!

hub = TechnicHub()
stopwatch = StopWatch()
stopwatch.resume()
receive_buffer = bytearray(REC_MSG_LENGTH)

# Motor definitions, including gearing ratios:
shoulder_forward = a = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[[8, 24],[12,36],[8,60]]) # Shoulder forward/back
shoulder_out     = b = Motor(Port.B, positive_direction=Direction.CLOCKWISE, reset_angle=False, gears=[[8,36],[20, 20],[12,20],[12,60]]) # Shoulder out/up / in/down to the side

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
    known_angle_shoulder_forward = shoulder_forward.angle()
    known_angle_shoulder_out = shoulder_out.angle()
    last_known_angles.shoulder_forward = known_angle_shoulder_forward
    last_known_angles.shoulder_out = known_angle_shoulder_out
    
    # Create JointAngles with only shoulder angles known
    current_angles = JointAngles(set_all_unknown=True)
    current_angles.shoulder_forward = known_angle_shoulder_forward
    current_angles.shoulder_out = known_angle_shoulder_out
    
    # Create and send reply message
    reply_message = create_message_from_controller_to_PC(INFORMATION_SOURCE_HUB_2, current_angles)
    stdout.buffer.write(reply_message)
    stdout.flush()  # Ensure the message is sent immediately


print_debug("Control limits (speed, acceleration, torque):")
print_debug(a.control.limits())
print_debug(b.control.limits())

print_debug("Control target tolerances (speed, position):")
print_debug(a.control.target_tolerances())
print_debug(b.control.target_tolerances())

# Registering stdin for polling. This allows us to wait for incoming data without blocking:
kbd_intr(-1) # Disables keyboard interupt on stdin
stdin_event_monitor = poll()
stdin_event_monitor.register(stdin, POLLIN)

# Reset position:
hub.light.blink(Color.ORANGE,[600,200]) # Orange blinking indicates calibrating or resetting to zero angles
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)
#a.reset_angle(0)
#b.reset_angle(0)

"""
# A dummy sequence for testing:
shoulder_forward.run_target(1500, 45, then=Stop.HOLD, wait=True)
shoulder_forward.run_target(1500, 0, then=Stop.HOLD, wait=False)

shoulder_out.run_target(1500, 35, then=Stop.HOLD, wait=True)
shoulder_out.run_target(1500, 0, then=Stop.HOLD, wait=False)

shoulder_out.run_target(1500, 0, then=Stop.HOLD, wait=True)
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
        if current_time - last_receive_time > 1000:
            if not is_timeout_already_expired:
                hub.light.blink(Color.VIOLET,[600,200]) # Violet blinking to indicate waiting for connection/data to begin or resume.
                is_timeout_already_expired = True
        
        # Send last known angles every 100 ms
        if current_time - last_send_time >= 100:
            send_current_angles_to_pc(last_known_angles)            
            last_send_time = current_time
        
        wait(10)

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
        last_known_angles.wrist = message.last_known_angles.wrist
        last_known_angles.underarm = message.last_known_angles.underarm
        last_known_angles.elbow = message.last_known_angles.elbow
        last_known_angles.overarm = message.last_known_angles.overarm

        if message.movement_mode == MOVEMENT_MODE_RETURN_TO_ZERO_AND_EXIT:
            break

        # TODO: On first received good message, activate a timeout that returns motors to starting position if communication stops (message not reveived for X ms)? (tapping into the wait loop above)

        # Now move as instructed:
        shoulder_forward.run_target(1500, message.desired_angles.shoulder_forward, then=Stop.HOLD, wait=False)
        shoulder_out.run_target(1500, message.desired_angles.shoulder_out, then=Stop.HOLD, wait=False)

    # Send last known angles every 100 ms, also here (in addition to in the receive wait loop) to 
    # ensure we don't miss updates in case of frequent incoming messages
    if current_time - last_send_time >= 100:
        send_current_angles_to_pc(last_known_angles)            
        last_send_time = current_time
    
    

# Return to zero angles and exit:
hub.light.blink(Color.ORANGE,[600,200])

# Clean up and finish by returning to starting position:
a.run_target(1500, 0, then=Stop.HOLD, wait=False)
b.run_target(1500, 0, then=Stop.HOLD, wait=False)
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)

print_debug("Done!")
