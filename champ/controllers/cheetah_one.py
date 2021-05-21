import time
import math
import copy
import numpy as np
from champ.utils import *
from champ.geometry import *
from champ.gait import TDEvent
from champ.body_controller import BodyController
from champ.foot_planner import project_foot, raibert_heuristic
from champ.foot_trajectory import BezierCurve, HalfSine


class CheetahOne(object):
    def __init__(self, base, gait_config):
        self._max_linear_x = gait_config.max_linear_velocity_x
        self._max_linear_y = gait_config.max_linear_velocity_y
        self._max_angular_z = gait_config.max_angular_velocity_z
        self._stance_duration = gait_config.stance_duration
        self._center_to_nominal = base.legs.center_to_nominal
        self._ref_foot = copy.deepcopy(base.legs.zero_stances)
        self._ref_foot = translate_x(self._ref_foot, gait_config.com_x_translation)

        self._body_controller = BodyController(base, gait_config.com_x_translation)
        self._gait_generator = TDEvent(gait_config.stance_duration)
        self._swing_trajectory = BezierCurve(gait_config.swing_height)
        self._stance_trajectory = HalfSine(gait_config.stance_depth)
   
    def walk(self, req_pose, req_vel, now=None):
        if now is None:
            now = int(round(time.perf_counter() * 1000000))
            
        #create reference foot positions based on required pose 
        foot_positions = self._body_controller.pose_command(req_pose)

        #constrain velocities to user defined min/max vels
        req_vel.linear.x = clip(req_vel.linear.x, -self._max_linear_x, self._max_linear_x)
        req_vel.linear.y = clip(req_vel.linear.y, -self._max_linear_y, self._max_linear_y)
        req_vel.angular.z = clip(req_vel.angular.z, -self._max_angular_z, self._max_angular_z)

        #use Raibert Heuristic to calculate how much the body should 
        #translate and rotate
        step_x, step_y, theta = raibert_heuristic(
            req_vel,
            self._stance_duration,
            self._center_to_nominal
        )
        
        #using the calculated translations and rotations,
        #we can project virtual foot positions from ref foot positions
        projected_foot = project_foot(
            self._ref_foot, #reference foot
            step_x, #translation in X axis
            step_y, #translation in Y axis
            theta #rotation in the Z axis
        )  
        
        #get the angle between ref foot position and projected foot position in XY plane
        #anddistance between ref foot position and projected foot position in XY Plane
        delta = projected_foot - self._ref_foot
        rotations = np.arctan2(delta[:, 1], delta[:, 0]).reshape((4,1))
        step_length = np.sqrt(np.power(delta[:, 0], 2) + np.power(delta[:, 1], 2)) * 2
        step_length = np.mean(step_length)
        
        #get switch states whether a leg is supposed to swing or stay on the ground
        swing, stance = self._gait_generator.run(step_length, now)
        
        #generate foot positions following a Bezier Curve trajectory - Swing
        #or HalfSine Trajectory - Stance
        #the generated positions will be passed to the IK engine to
        #calculate joint positions for the actuators
        foot_positions = np.where(
            stance > swing,
            self._stance_trajectory.generate(
                foot_positions, 
                step_length, 
                rotations, 
                stance
            ),
            foot_positions
        )

        foot_positions = np.where(
            swing > stance,
            self._swing_trajectory.generate(
                foot_positions, 
                step_length, 
                rotations, 
                swing
            ),
            foot_positions
        )

        return foot_positions