import time
import numpy as np
import matplotlib.pyplot as plt

from champ.types import Pose, Velocities
from champ.base import Base
from champ.controllers.cheetah_one import CheetahOne
from champ.kinematics import Kinematics
from champ.robots.profile import RobotProfile

custom_profile = RobotProfile()
custom_profile.pantograph_leg = False
custom_profile.max_linear_velocity_x = 0.5
custom_profile.max_linear_velocity_y = 0.5
custom_profile.max_angular_velocity_z = 1.0
custom_profile.swing_height = 0.04
custom_profile.stance_depth = 0.0
custom_profile.stance_duration = 0.25
custom_profile.nominal_height = 0.2
custom_profile.com_x_translation = 0.0
custom_profile.knee_orientation = ">>"

quadruped = Base()

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

controller = CheetahOne(quadruped, custom_profile)
ik = Kinematics(quadruped)

req_pose = Pose()
req_pose.position.z = custom_profile.nominal_height

req_vel = Velocities()
req_vel.linear.x = 1.0
# req_vel.linear.y = 1.0

average_duration = 0
iterations = 1000

for i in range(iterations):
    start = time.time()
    foot_positions = controller.walk(req_pose, req_vel)
    joint_positions = ik.inverse(foot_positions)
    quadruped.joint_positions = joint_positions

    current_duration = time.time() - start
    average_duration += current_duration
        
average_duration /= iterations
print(f'Average controller execution time per step: {average_duration * 1000} milliseconds')
print(f'Max frequency {1 / average_duration} Hz')


