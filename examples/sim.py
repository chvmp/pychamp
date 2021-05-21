import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
from champ.types import Pose, GaitConfig, Velocities
from champ.base import Base
from champ.controllers.cheetah_one import CheetahOne
from champ.kinematics import Kinematics
from champ.robots.profile import AlienGo as aliengo
from champ.robots.profile import AnymalB as anymal_b
from champ.robots.profile import AnymalC as anymal_c
from champ.robots.profile import DKitty as dkitty
from champ.robots.profile import LittleDog as littledog
from champ.robots.profile import MiniCheetah as mini_cheetah
from champ.robots.profile import OpenQuadruped as open_quadruped
from champ.robots.profile import OpenDog as opendog
from champ.robots.profile import Spot as spot
from champ.robots.profile import SpotMicro as spotmicro
from champ.robots.profile import StochLite as stochlite
from champ.pybullet.sensors import PyBulletSensors
import champ.pybullet.utils as pb_utils

class Champ:
    def __init__(self):
        #change to the robot you want to use ie.
        # robot_profile = spot
        # robot_profile = anymal_b
        # robot_profile = anymal_c
        robot_profile = open_quadruped

        physics_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        base_id = p.loadURDF(
            robot_profile.urdf, 
            [0, 0, robot_profile.gait_config.nominal_height * 1.5], 
            p.getQuaternionFromEuler([0, 0, 0])
        )
        
        quadruped = Base(robot_profile)
        controller = CheetahOne(quadruped, robot_profile.gait_config)
        ik = Kinematics(quadruped, robot_profile.gait_config.knee_orientation)
        sensors = PyBulletSensors(plane_id, base_id, robot_profile.joint_names, robot_profile.link_names)

        req_pose = Pose()
        req_pose.position.z = robot_profile.gait_config.nominal_height
        req_vel = Velocities()

        pb_utils.print_teleop_instructions()

        for i in range(p.getNumJoints(base_id)):
            joint_info = p.getJointInfo(base_id, i)

        while True:
            req_vel = pb_utils.get_req_vel(base_id, req_vel)
       
            foot_positions = controller.walk(req_pose, req_vel)
            target_joint_positions = ik.inverse(foot_positions)

            quadruped.joint_positions = sensors.joint_positions()
            foot_positions_from_base = quadruped.feet.position
            foot_positions_from_hip = quadruped.transform_to_hip(foot_positions_from_base)
           
            p.setJointMotorControlArray(base_id, sensors.actuator_ids, p.POSITION_CONTROL, list(target_joint_positions))
            p.stepSimulation()

        p.disconnect()

if __name__ == '__main__':
    c = Champ()
