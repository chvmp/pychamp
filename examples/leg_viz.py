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

quadruped.lf.hip.set_origin(0.175, 0.105, 0)
quadruped.lf.upper_leg.set_origin(0, 0.06, 0)
quadruped.lf.lower_leg.set_origin(0, 0, -0.141)
quadruped.lf.foot.set_origin(0, 0, -0.141)
   
quadruped.rf.hip.set_origin(0.175, -0.105, 0)
quadruped.rf.upper_leg.set_origin(0, -0.06, 0)
quadruped.rf.lower_leg.set_origin(0, 0, -0.141)
quadruped.rf.foot.set_origin(0, 0, -0.141)

quadruped.lh.hip.set_origin(-0.175, 0.105, 0)
quadruped.lh.upper_leg.set_origin(0, 0.06, 0)
quadruped.lh.lower_leg.set_origin(0, 0, -0.141)
quadruped.lh.foot.set_origin(0, 0, -0.141)

quadruped.rh.hip.set_origin(-0.175, -0.105, 0)
quadruped.rh.upper_leg.set_origin(0, -0.06, 0)
quadruped.rh.lower_leg.set_origin(0, 0, -0.141)
quadruped.rh.foot.set_origin(0, 0, -0.141)

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
    start = time.time()
    leg_viz.plot_legs(id=0,latch=100)
    foot_positions = controller.walk(req_pose, req_vel)
    joint_positions = ik.inverse(foot_positions)
    quadruped.joint_positions = joint_positions
    upper_leg = quadruped.upper_legs.position
    lower_leg = quadruped.lower_legs.position
    foot = quadruped.lf.foot.position
    current_duration = time.time() - start
    average_duration += current_duration
