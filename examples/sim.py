import sys
import pybullet as p
import numpy as np

import time
import pybullet_data
from champ.types import Pose, GaitConfig, Velocities
from champ.base import Base
from champ.controllers.cheetah_one import CheetahOne
from champ.body_controller import PoseGenerator
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

class Champ:
    def __init__(self):
        #change to the robot you want to use ie.
        # robot_profile = spot
        robot_profile = anymal_b
        # robot_profile = anymal_c
        # robot_profile = open_quadruped

        quadruped = Base(robot_profile)
        controller = CheetahOne(quadruped, robot_profile.gait_config)
        ik = Kinematics(quadruped, robot_profile.gait_config.knee_orientation)

        req_pose = Pose()
        req_pose.position.z = robot_profile.gait_config.nominal_height
        req_vel = Velocities()

        joint_names = []
        for i, joint in enumerate(robot_profile.joint_names):
            if i not in [3,7,11,15]:
                joint_names.append(joint)

        #source https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3
        physics_client = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-10)
        plane_id = p.loadURDF("plane.urdf")
        champ_start_pos = [0,0, robot_profile.gait_config.nominal_height * 1.5]
        champ_start_orientation = p.getQuaternionFromEuler([0,0,0])
        champ = p.loadURDF(robot_profile.urdf, champ_start_pos, champ_start_orientation)
        num_joints = p.getNumJoints(champ)
        
        bullet_joint_indices = [0] * 12

        #iterate all joints in the URDF
        for j_idx in range(num_joints):
            joint_info= p.getJointInfo(champ, j_idx)
            joint_name = joint_info[1]
            joint_name = joint_name.decode("utf-8")
        
            if joint_name in joint_names:
                #if the current joint in the URDF is one of the joint name received from controller
                #save the index of the that joint in the received joint name's index
                bullet_joint_indices[joint_names.index(joint_name)] = j_idx

        roll_gen = PoseGenerator(0.4, 0.5)
        pitch_gen = PoseGenerator(0.3, 0.5, phase_shift=1.5708)
        yaw_gen = PoseGenerator(0.3, 0.5)
        for i in range(3500):
            if i == 0:
                print("Linear Velocity X")
                req_vel.linear.x = 0.5
                req_vel.linear.y = 0.0
                req_vel.angular.z = 0.0
            elif i == 500:
                print("Linear Velocity Y")
                req_vel.linear.x = 0.0
                req_vel.linear.y = 1.0
                req_vel.angular.z = 0.0
            elif i == 1000:
                print("Angular Velocity Z")
                req_vel.linear.x = 0.0
                req_vel.linear.y = 0.0
                req_vel.angular.z = 1.0
            elif i == 1500:
                print("Turning")
                req_vel.linear.x = 0.5
                req_vel.linear.y = 0.0
                req_vel.angular.z = 1.0
            elif i == 2000:
                print("Linear Velocity XY")
                req_vel.linear.x = 0.5
                req_vel.linear.y = 1.0
                req_vel.angular.z = 0.0
            elif i ==  2500:
                print("Pose Command")
            elif i > 2500: 
                req_vel.linear.x = 0.0
                req_vel.linear.y = 0.0
                req_vel.angular.z = 0.0
                req_pose.orientation.roll = roll_gen.sine()
                req_pose.orientation.pitch = pitch_gen.sine()
                req_pose.orientation.yaw = yaw_gen.sine()

            foot_positions = controller.walk(req_pose, req_vel)
            target_joint_positions = ik.inverse(foot_positions)

            joint_positions = [0.0] * 12
            
            joint_states = p.getJointStates(champ, bullet_joint_indices)
            for i, joint_info in enumerate(joint_states):
                position = joint_info[0]
                joint_positions[i] = position

            quadruped.joint_positions = joint_positions
            foot_positions_from_base = quadruped.feet.position
            foot_positions_from_hip = quadruped.transform_to_hip(foot_positions_from_base)

            p.setJointMotorControlArray(champ, bullet_joint_indices, p.POSITION_CONTROL, list(target_joint_positions))
            p.stepSimulation()
        p.disconnect()

if __name__ == '__main__':
    c = Champ()
