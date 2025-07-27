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
from RobotMessageBuilderEmbedded import JointAngles, MessageFromPcToController, \
     parse_message_from_PC_to_controller, REC_MSG_LENGTH, MOVEMENT_MODE_RETURN_TO_ZERO_AND_EXIT

PRINT_DEBUG_INFO = False # Set to True to enable debug messages via printing to stdout. This will interfere with Bluetooth communication that also uses stdout!

hub = TechnicHub()
stopwatch = StopWatch()
stopwatch.resume()
receive_buffer = bytearray(REC_MSG_LENGTH)

# Motor definitions, including gearing ratios:
wrist    = a = Motor(Port.A, positive_direction=Direction.CLOCKWISE, reset_angle=False, gears=[12, 32]) # Wrist up/down
underarm = b = Motor(Port.B, positive_direction=Direction.CLOCKWISE, reset_angle=False, gears=[[12, 20],[8,28]]) # Underarm twist
elbow    = c = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[[1, 24],[20,28]]) # Elbow
overarm  = d = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[[8, 24],[12,60]]) # Overarm twist

last_known_angles = JointAngles(set_all_unknown = True)


def print_debug(string):
    if PRINT_DEBUG_INFO:
        print(string)


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


"""
# A dummy sequence for testing:
wrist.run_target(1500, 110, then=Stop.HOLD, wait=True)
wrist.run_target(1500, -110, then=Stop.HOLD, wait=True)
wrist.run_target(1500, 0, then=Stop.HOLD, wait=False)

#wrist.run_target(1500, -180*12/32, then=Stop.HOLD, wait=False) # Compensate a when b is moving
underarm.run_target(1500, 120, then=Stop.HOLD, wait=True)
underarm.run_target(1500, -120, then=Stop.HOLD, wait=True)
underarm.run_target(1500, 0, then=Stop.HOLD, wait=False)
underarm.run_target(1500, 0, then=Stop.HOLD, wait=True)

elbow.run_target(1500, 13, then=Stop.HOLD, wait=True)
elbow.run_target(1500, -20, then=Stop.HOLD, wait=True)
elbow.run_target(1500, 0, then=Stop.HOLD, wait=False)

overarm.run_target(1500, 35, then=Stop.HOLD, wait=True)
overarm.run_target(1500, -120, then=Stop.HOLD, wait=True)
overarm.run_target(1500, 0, then=Stop.HOLD, wait=False)
#wait(2000)
"""

hub.light.blink(Color.VIOLET,[600,200]) # Violet blinking to indicate waiting for connection/data to begin.

# Let the remote program know we are ready for commands
stdout.buffer.write(b"rdy") # TODO Check if we need to call flush() after writing.

while True:
    stopwatch.reset()
    is_timeout_already_expired = False
    # Check if data is available
    while not stdin_event_monitor.poll(0):
        if stopwatch.time() > 1000:
            if not is_timeout_already_expired:
                hub.light.blink(Color.VIOLET,[600,200]) # Violet blinking to indicate waiting for connection/data to begin or resume.
                is_timeout_already_expired = True
        # TODO: Send last known angles every X ms
        wait(10)

    # Read message from stdin
    num_bytes_read = stdin.buffer.readinto(receive_buffer, REC_MSG_LENGTH)
    print_debug(f"Read {num_bytes_read} bytes: {receive_buffer}")
    is_message_ok, message = parse_message_from_PC_to_controller(receive_buffer)
    
    # Decide what to do based on the command.
    if not is_message_ok:
        hub.light.on(Color.RED)
    else:
        hub.light.on(Color.GREEN)

        if message.movement_mode == MOVEMENT_MODE_RETURN_TO_ZERO_AND_EXIT:
            break

        # TODO: On first received good message, activate a timeout that returns and stops motors if communication stops (message not reveived for X ms) (tapping into the wait loop above)

        # Now move as instructed:
        wrist.run_target(   1500, message.desired_angles.wrist, then=Stop.HOLD, wait=False)
        underarm.run_target(1500, message.desired_angles.underarm, then=Stop.HOLD, wait=False)
        elbow.run_target(   1500, message.desired_angles.elbow, then=Stop.HOLD, wait=False)
        overarm.run_target( 1500, message.desired_angles.overarm, then=Stop.HOLD, wait=False)


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
