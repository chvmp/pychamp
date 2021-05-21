import time
import numpy as np
import matplotlib.pyplot as plt

from champ.types import Pose, GaitConfig, Velocities
from champ.base import Base
from champ.controllers.cheetah_one import CheetahOne
from champ.kinematics import Kinematics
from champ.visualizer.leg_visualizer import LegVisualizer

gait_config = GaitConfig()
gait_config.pantograph_leg = False
gait_config.max_linear_velocity_x = 0.5
gait_config.max_linear_velocity_y = 0.5
gait_config.max_angular_velocity_z = 1.0
gait_config.swing_height = 0.04
gait_config.stance_depth = 0.0
gait_config.stance_duration = 0.25
gait_config.nominal_height = 0.2
gait_config.knee_orientation = ">>"

quadruped = Base()
quadruped.gait_config = gait_config

quadruped.hips.set_origin(0, 0.175, 0.105, 0)
quadruped.upper_legs.set_origin(0, 0, 0.06, 0)
quadruped.lower_legs.set_origin(0, 0, 0, -0.141)
quadruped.feet.set_origin(0, 0, 0, -0.141)
   
quadruped.hips.set_origin(1, 0.175, -0.105, 0)
quadruped.upper_legs.set_origin(1, 0, -0.06, 0)
quadruped.lower_legs.set_origin(1, 0, 0, -0.141)
quadruped.feet.set_origin(1, 0, 0, -0.141)

quadruped.hips.set_origin(2, -0.175, 0.105, 0)
quadruped.upper_legs.set_origin(2, 0, 0.06, 0)
quadruped.lower_legs.set_origin(2, 0, 0, -0.141)
quadruped.feet.set_origin(2, 0, 0, -0.141)

quadruped.hips.set_origin(3, -0.175, -0.105, 0)
quadruped.upper_legs.set_origin(3, 0, -0.06, 0)
quadruped.lower_legs.set_origin(3, 0, 0, -0.141)
quadruped.feet.set_origin(3, 0, 0, -0.141)

quadruped.init()
controller = CheetahOne(quadruped, gait_config)
ik = Kinematics(quadruped, gait_config.knee_orientation)
leg_viz = LegVisualizer(quadruped)

req_pose = Pose()
req_pose.position.z = gait_config.nominal_height

req_vel = Velocities()
req_vel.linear.x = 1.0

average_duration = 0
iterations = 5000

for i in range(iterations):
    leg_viz.plot_legs(id=0,latch=100)
    foot_positions = controller.walk(req_pose, req_vel)
    joint_positions = ik.inverse(foot_positions)
    quadruped.joint_positions = joint_positions
    time.sleep(0.01)