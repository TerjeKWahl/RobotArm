"""
Controls the lower 4 axis on a robot arm, and communicates with a PC app.
Built using Pybricks (see pybricks.com for more info).
"""

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = TechnicHub()

# Motor definitions, including gearing ratios:
wrist    = a = Motor(Port.A, positive_direction=Direction.CLOCKWISE, reset_angle=False, gears=[12, 32]) # Wrist
underarm = b = Motor(Port.B, positive_direction=Direction.CLOCKWISE, reset_angle=False, gears=[[12, 20],[8,28]]) # Underarm twist
elbow    = c = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[[1, 24],[20,28]]) # Elbow
overarm  = d = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE, reset_angle=False, gears=[[8, 24],[12,60]]) # Overarm twist

print("Control limits (speed, acceleration, torque):")
print(a.control.limits())
print(b.control.limits())
print(c.control.limits())
print(d.control.limits())

print("Control target tolerances (speed, position):")
print(a.control.target_tolerances())
print(b.control.target_tolerances())
print(c.control.target_tolerances())
print(d.control.target_tolerances())

# Reset position:
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)
c.run_target(1500, 0, then=Stop.HOLD, wait=True)
d.run_target(1500, 0, then=Stop.HOLD, wait=True)
a.reset_angle(0)
b.reset_angle(0)
c.reset_angle(0)
d.reset_angle(0)

# Just a dummy sequence for now:
a.run_target(1500, 90, then=Stop.HOLD, wait=True)
a.run_target(1500, -90, then=Stop.HOLD, wait=True)
a.run_target(1500, 0, then=Stop.HOLD, wait=False)

#a.run_target(1500, -180*12/32, then=Stop.HOLD, wait=False) # Compensate a when b is moving
b.run_target(1500, 180, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=False)

c.run_target(1500, 90, then=Stop.HOLD, wait=True)
c.run_target(1500, 0, then=Stop.HOLD, wait=False)

d.run_target(1500, 90, then=Stop.HOLD, wait=True)
d.run_target(1500, -90, then=Stop.HOLD, wait=True)
d.run_target(1500, 0, then=Stop.HOLD, wait=False)

wait(1000)

#a.run_target(1500, -90, then=Stop.HOLD, wait=True)
#c.run_target(1500, 0, then=Stop.HOLD, wait=True)

# Clean up and finish by returning to starting position:
a.run_target(1500, 0, then=Stop.HOLD, wait=False)
b.run_target(1500, 0, then=Stop.HOLD, wait=False)
c.run_target(1500, 0, then=Stop.HOLD, wait=False)
d.run_target(1500, 0, then=Stop.HOLD, wait=False)
a.run_target(1500, 0, then=Stop.HOLD, wait=True)
b.run_target(1500, 0, then=Stop.HOLD, wait=True)
c.run_target(1500, 0, then=Stop.HOLD, wait=True)
d.run_target(1500, 0, then=Stop.HOLD, wait=True)
print("Done!")
