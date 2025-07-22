"""
Controls the first two shoulder joints on a robot arm, and communicates with a PC app.
Built using Pybricks (see pybricks.com for more info).
"""

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = TechnicHub()

# Motor definitions, including gearing ratios:
shoulder_forward = a = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[[8, 24],[12,36],[8,60]]) # Shoulder forward/back
shoulder_out     = b = Motor(Port.B, positive_direction=Direction.CLOCKWISE, reset_angle=False, gears=[[8,36],[20, 20],[12,20],[12,60]]) # Shoulder out/up / in/down to the side

print("Control limits (speed, acceleration, torque):")
print(a.control.limits())
print(b.control.limits())

print("Control target tolerances (speed, position):")
print(a.control.target_tolerances())
print(b.control.target_tolerances())


# Reset position:
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)
#a.reset_angle(0)
#b.reset_angle(0)

# Just a dummy sequence for now:
a.run_target(1500, 45, then=Stop.HOLD, wait=True)
a.run_target(1500, 0, then=Stop.HOLD, wait=False)

b.run_target(1500, 40, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=False)

wait(1000)

# Clean up and finish by returning to starting position:
a.run_target(1500, 0, then=Stop.HOLD, wait=False)
b.run_target(1500, 0, then=Stop.HOLD, wait=False)
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)

print("Done!")