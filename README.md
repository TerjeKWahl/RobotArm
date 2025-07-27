# RobotArm

Humanoid robot arm with 6 DoF and a gripper, built mostly in Lego, controllable from VR and (todo) with AI to make it autonomous.

This repository contains the following project parts in these folders:
- `EmbeddedControllers` Robot arm embedded controllers
  - `ControllerLowerArm` Lego Technic Hub 1 controller (lower arm) using Pybricks
  - `ControllerShoulder` Lego Technic Hub 2 controller (shoulder) using Pybricks
  - `ControllerGripper` Controller for the gripper using Arduino
- `RobotArmController` Main controller program in Python, executed on a PC to 
  - control the robot arm using inverse kinematics,
  - communicate with the Lego Technic Hubs and Arduino controllers,
  - communicate with any connected VR app,
  - (todo) gather images from vision camera,
  - (todo) do AI raw data collection and
  - (todo) do AI inference
- `VR` Meta Quest 3 VR app to remote control the robot arm and (todo) record AI training data
- `AI` TODO: AI stuff: Training and inference
- `Mechanics` 3D files for custom made mechanical parts
  - `Gripper` 3D files for robot gripper
  - `LegoTechnicHubWireInset` 3D file for modifying Lego Technic Hub to be driven by a power adapter in stead of batteries

These components communicate using well defined communication APIs. Please see the `Docs` folder for more information.
