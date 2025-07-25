"""
Controls the first two shoulder joints on a robot arm, and communicates with a PC app.
Built using Pybricks (see pybricks.com for more info).
"""

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.tools import wait, StopWatch
# Standard MicroPython modules
from usys import stdin, stdout
from uselect import poll, POLLIN
from micropython import kbd_intr

hub = TechnicHub()
stopwatch = StopWatch()
stopwatch.resume()

PRINT_DEBUG_INFO = False # Set to True to enable debug messages via printing to stdout. This will interfere with Bluetooth communication that also uses stdout!
REC_MSG_LENGTH = 3 # TODO: 16
receive_buffer = bytearray(REC_MSG_LENGTH)

# Motor definitions, including gearing ratios:
shoulder_forward = a = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[[8, 24],[12,36],[8,60]]) # Shoulder forward/back
shoulder_out     = b = Motor(Port.B, positive_direction=Direction.CLOCKWISE, reset_angle=False, gears=[[8,36],[20, 20],[12,20],[12,60]]) # Shoulder out/up / in/down to the side

def print_debug(string):
    if PRINT_DEBUG_INFO:
        print(string)


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

hub.light.blink(Color.VIOLET,[600,200]) # Violet blinking to indicate waiting for connection/data to begin.

while True:
    # Let the remote program know we are ready for a command
    stdout.buffer.write(b"rdy") # TODO Check if we need to call flush() after writing.

    # Check if data is available
    stopwatch.reset()
    is_timeout_already_expired = False
    while not stdin_event_monitor.poll(0):
        if stopwatch.time() > 2000:
            if not is_timeout_already_expired:
                hub.light.blink(Color.VIOLET,[600,200]) # Violet blinking to indicate waiting for connection/data to begin or resume.
                is_timeout_already_expired = True
        wait(10)

    # Read message from stdin
    num_bytes_read = stdin.buffer.readinto(receive_buffer, REC_MSG_LENGTH)
    print_debug(f"Read {num_bytes_read} bytes: {receive_buffer}")
    cmd = receive_buffer

    # Decide what to do based on the command.
    if cmd == b"gre":
        hub.light.on(Color.GREEN)
    elif cmd == b"yel":
        hub.light.on(Color.YELLOW)
    elif cmd == b"bye":
        break
    else:
        hub.light.on(Color.RED)


#hub.light.blink(Color.VIOLET,[600,200])
#data = bytearray(b"TW2")
#while True:
    #hub.ble.broadcast(data)
    #data = hub.ble.observe(BLE_BROADCAST_CHANNEL)
    #wait(100)

# Reset position:
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)
#a.reset_angle(0)
#b.reset_angle(0)

# Just a dummy sequence for now:
#a.run_target(1500, 45, then=Stop.HOLD, wait=True)
#a.run_target(1500, 0, then=Stop.HOLD, wait=False)

#b.run_target(1500, 40, then=Stop.HOLD, wait=True)
#b.run_target(1500, 0, then=Stop.HOLD, wait=False)

#wait(2000)

# Clean up and finish by returning to starting position:
a.run_target(1500, 0, then=Stop.HOLD, wait=False)
b.run_target(1500, 0, then=Stop.HOLD, wait=False)
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)

print_debug("Done!")
