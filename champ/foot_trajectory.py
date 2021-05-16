import numpy as np
import copy
from champ.geometry import *


def constrain_swing_height(swing_height):
    if swing_height < 0.0:
        swing_height = -swing_height
    
    return swing_height


def constrain_stance_depth(stance_depth):
    if stance_depth > 0.0:
        stance_depth = -stance_depth
    
    return stance_depth


def transform_trajectory_point(foot_positions, x, z, trajectory_rotation):
    y = np.zeros((4,1))
    #create a projected foot position using calculated X, and Zs
    projected_foot = np.hstack((x, y, z))
    
    #rotate the trajectory plane in the Z axis
    #rotation is based on RaiberHeuristic(velocities, stance_duration, COM_TO_LEG_DISTANCE)
    projected_foot = rotate_z(projected_foot, trajectory_rotation)
    
    foot_positions = translate(foot_positions, projected_foot)

    return foot_positions


class BezierCurve(object):
    def __init__(self, swing_height):
        swing_height = constrain_swing_height(swing_height)
        
        self._ref_control_points_x = np.array([-0.15, -0.2805,-0.3,-0.3,-0.3, 0.0, 0.0, 0.0, 0.3032, 0.3032, 0.2826, 0.15])
        self._ref_control_points_z = np.array([-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611, -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5])
        
        self._control_points_x = np.zeros(12)
        self._control_points_z = np.zeros(12)

        indices_m_one = []
        factorial = []
        factorial_m_one = []

        for i in range(12):
            factorial.append(np.math.factorial(i))
            factorial_m_one.append(np.math.factorial(11-i))
            indices_m_one.append(11-i)

        factorial = np.array(factorial)
        factorial = factorial.reshape((1,12))
        factorial_m_one = np.array(factorial_m_one)
        factorial_m_one = factorial_m_one.reshape((1,12))
        factorial_n = factorial[0,11]
        self._bernstein_coeff = factorial_n / (factorial * factorial_m_one)

        self._indices = np.arange(12)
        self._indices = self._indices.reshape((1,12))
        self._indices_m_one = np.array(indices_m_one)
        self._indices_m_one = self._indices_m_one.reshape((1,12))

        self._run_once = False
        self._height_ratio = 0.0
        self._length_ratio = 0.0
        self._pref_foot_positions = np.zeros((4, 3))

        self.update_control_points_height(swing_height)

    def update_control_points_height(self, swing_height):
        new_height_ratio = swing_height / 0.15

        if self._height_ratio != new_height_ratio:
            self._height_ratio = new_height_ratio
            self._control_points_z = -((self._ref_control_points_z * self._height_ratio) + (0.5 * self._height_ratio))
    
    def update_control_points_length(self, step_length):
        new_length_ratio = step_length / 0.4
        half_step = step_length / 2.0
        if self._length_ratio != new_length_ratio:
            self._length_ratio = new_length_ratio
            self._control_points_x = self._ref_control_points_x * self._length_ratio
            self._control_points_x[0] = -half_step
            self._control_points_x[11] = half_step

    def generate(self, foot_positions, step_length, rotation, swing_phase_signals):
        if step_length == 0.0:
            return foot_positions

        self.update_control_points_length(step_length)

        x = np.zeros((4,1))
        z = np.zeros((4,1))

        #swing phase
        #http://graphics.cs.ucdavis.edu/education/CAGDNotes/Bernstein-Polynomials.pdf
        bernstein = self._bernstein_coeff * np.power(swing_phase_signals, self._indices) * np.power((1 - swing_phase_signals), self._indices_m_one) 
        
        #calculate bezier curve using bernstein polynomial and control points
        bezier = np.zeros((2,4,12))
        bezier[0, :, :] = bernstein * self._control_points_x
        bezier[1, :, :] = -bernstein * self._control_points_z
        bezier = np.sum(bezier, axis=2)

        #use bezier curve as a trajectory during swing phase period    
        x = np.where(
            swing_phase_signals > 0.0,
            bezier[0,:].reshape(4,1),
            x
        )

        z = np.where(
            swing_phase_signals > 0.0,
            bezier[1,:].reshape(4,1),
            z
        )

        return transform_trajectory_point(foot_positions, x, z, rotation)



class HalfSine(object):
    def __init__(self, stance_depth):
        self._stance_depth = constrain_stance_depth(stance_depth)

    def generate(self, foot_positions, step_length, rotation, stance_phase_signals): 
        if step_length == 0.0:
            return foot_positions

        x = np.zeros((4,1))
        z = np.zeros((4,1))

        #stance phase
        #use half sine equation as a trajectory during stance period
        x = np.where(
            stance_phase_signals > 0.0,
            (step_length / 2.0) * (1 - (2.0 * stance_phase_signals)),
            x
        )

        z = np.where(
            stance_phase_signals > 0.0,
            self._stance_depth * np.cos((np.pi * x) / step_length),
            z
        )

        return transform_trajectory_point(foot_positions, x, z, rotation)

    
