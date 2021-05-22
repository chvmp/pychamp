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
import champ.geometry as geom

class Champ:
    def __init__(self):
        #change to the robot you want to use ie.
        robot_profile = spot
        # robot_profile = anymal_b
        # robot_profile = anymal_c
        # robot_profile = open_quadruped

        physics_client = p.connect(p.GUI)
        p.setRealTimeSimulation(True)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        base_id = p.loadURDF(
            robot_profile.urdf, 
            [0, 0, robot_profile.nominal_height * 1.5], 
            p.getQuaternionFromEuler([0, 0, 0])
        )
        
        quadruped = Base(robot_profile)
        controller = CheetahOne(quadruped, robot_profile)
        ik = Kinematics(quadruped, robot_profile.knee_orientation)
        sensors = PyBulletSensors(plane_id, base_id, robot_profile.joint_names, robot_profile.link_names)

        req_pose = Pose()
        req_pose.position.z = robot_profile.nominal_height
        req_vel = Velocities()

        pb_utils.print_teleop_instructions()

        for i in range(p.getNumJoints(base_id)):
            joint_info = p.getJointInfo(base_id, i)

        while True:
            req_vel = pb_utils.get_req_vel(base_id, req_vel)
       
            foot_positions = controller.walk(req_pose, req_vel)
            target_joint_positions = ik.inverse(foot_positions)

            quadruped.joint_positions = sensors.joint_positions()

            base_orientation = sensors.base_orientation_quat()
            base_rot_matrix = geom.quat_to_matrix(
                x=base_orientation[0],
                y=base_orientation[1],
                z=base_orientation[2],
                w=base_orientation[3]
            )
 
            base_orientation = sensors.base_orientation_rpy()
            base_rot_matrix = geom.rpy_to_matrix(
                roll=base_orientation[0],
                pitch=base_orientation[1],
                yaw=base_orientation[2]
            )
     
            foot_positions_from_base = quadruped.feet.position
            foot_positions_from_hip = quadruped.transform_to_hip(foot_positions_from_base)
            
            linear, angular = sensors.base_velocity()
            l_vx, l_vy, l_vz = linear
            a_vx, a_vy, a_vz = angular

            world_x, world_y, world_z = sensors.base_position()

            contact_states = sensors.contact_states()

            p.setJointMotorControlArray(base_id, sensors.actuator_ids, p.POSITION_CONTROL, list(target_joint_positions))

        p.disconnect()

if __name__ == '__main__':
    c = Champ()
