import time
import math
import copy
import numpy as np
from champ.geometry import *
from champ.gait import TDEvent
from champ.body_controller import BodyController
from champ.foot_planner import BezierCurve, project_foot, raibert_heuristic
from champ.utils import *

class CheetahOne(object):
    def __init__(self, base, gait_config):
        self._gait_config = gait_config
        self._center_to_nominal = base.lf.center_to_nominal
        self._zero_stances = base.zero_stances

        self._body_controller = BodyController(base)
        self._gait_generator = TDEvent(base, 
                                       gait_config.stance_duration)
        self._foot_planner = BezierCurve(gait_config.swing_height, 
                                         gait_config.stance_depth)

    def walk(self, req_pose, req_vel, now=None):
        if now is None:
            now = int(round(time.time() * 1000))

        #create reference foot positions based on required pose 
        foot_positions = self._body_controller.pose_command(req_pose)

        #constrain velocities to user defined min/max vels
        req_vel.linear.x = clip(req_vel.linear.x, 
                                -self._gait_config.max_linear_velocity_x,
                                self._gait_config.max_linear_velocity_x)
        req_vel.linear.y = clip(req_vel.linear.y, 
                                -self._gait_config.max_linear_velocity_y,
                                self._gait_config.max_linear_velocity_y)
        req_vel.angular.z = clip(req_vel.angular.z, 
                                 -self._gait_config.max_angular_velocity_z,
                                 self._gait_config.max_angular_velocity_z)

        #use Raibert Heuristic to calculate body's translation and rotation
        step_x, step_y, theta = raibert_heuristic(req_vel, 
                                                  self._gait_config.stance_duration,
                                                  self._center_to_nominal)
        
        #using the calculataed translations and rotations,
        #we can project virtual foot positions from ref foot positions
        projected_foot = project_foot(self._zero_stances, #reference foot
                                      step_x, #translation in X axis
                                      step_y, #translation in Y axis
                                      theta)  #rotation in the Z axis
        
        delta = projected_foot - self._zero_stances

        #get the angle between ref foot position and projected foot position in XY plane
        rotations = np.arctan2(delta[:, 1], delta[:, 0]).reshape((4,1))
        
        #distance between ref foot position and projected foot position in XY Plane
        step_length = np.sqrt(np.power(delta[:, 0], 2) + np.power(delta[:, 1], 2)) * 2
        step_length = np.mean(step_length)
        
        #get switch states whether a leg is supposed to swing or stay on the ground
        swing, stance = self._gait_generator.run(step_length, now)
        
        #generate foot positions following a Bezier Curve trajectory
        #the generated positions will be passed to the IK engine to
        #calculate joint positions for the actuators
        foot_positions = self._foot_planner.generate(foot_positions, 
                                                     step_length, 
                                                     rotations, 
                                                     (swing, stance))

        return foot_positions