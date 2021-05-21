import numpy as np
import copy
from champ.geometry import *
from champ.utils import vectorize_knee_orientation

class Kinematics(object):
    def __init__(self, base, knee_orientation='>>'):
        self._half_pi = np.math.pi / 2.0

        #Y distance from the hip to the foot
        self._l0 = base.upper_legs.translation.y  +\
                   base.lower_legs.translation.y  +\
                   base.feet.translation.y

        lower_leg_x = base.lower_legs.translation.x
        lower_leg_z = base.lower_legs.translation.z

        feet_x = base.feet.translation.x
        feet_z = base.feet.translation.z

        #hypotenuse of upper leg link if the robot has x displacement
        #from upper leg to knee
        self._l1 = -np.sqrt(np.power(lower_leg_x , 2) + np.power(lower_leg_z, 2))
        #pre calculate square of l1
        self._l1_squared = np.power(self._l1, 2)
        #offset angle due to the x displacement from upper leg to knee
        self._ik_alpha = np.arccos(lower_leg_x / self._l1) - self._half_pi

        #hypotenuse of the lower leg link if the robot has x displacement
        #from knee to foot
        self._l2 = -np.sqrt(np.power(feet_x, 2) + np.power(feet_z, 2))
        #pre calculate square of l2
        self._l2_squared = np.power(self._l2, 2)
        #offset angle due to the x displacement from knee to fppt
        self._ik_beta = np.arccos(feet_x / self._l2) - self._half_pi

        #just a pre-calculated term for computing lower leg joint
        self._ll_arc_denom = (2.0 * self._l1 * self._l2)

        #translation from hip to upper leg
        self._upper_leg_trans = copy.deepcopy(base.upper_legs.translation.xyz)
        self._upper_leg_trans[:, 1] = 0.0

        self._knee_directions = vectorize_knee_orientation(knee_orientation)

    def inverse(self, foot_positions):
        temp_foot_pos = foot_positions
    
        y = temp_foot_pos[:, 1, np.newaxis] 
        z = temp_foot_pos[:, 2, np.newaxis] 

        hip_joints = -(np.arctan(y / z) - (self._half_pi - np.arccos(-self._l0 / np.sqrt(np.power(y, 2) + np.power(z, 2)))))
        
        #shift ref frame from base to hip
        temp_foot_pos = translate(temp_foot_pos, -self._upper_leg_trans)
        temp_foot_pos = rotate_x(temp_foot_pos, -hip_joints)
        
        x = temp_foot_pos[:, 0, np.newaxis] 
        z = temp_foot_pos[:, 2, np.newaxis] 
                
        lower_leg_joints = self._knee_directions * np.arccos((np.power(z, 2) + np.power(x, 2) - self._l1_squared - self._l2_squared) / self._ll_arc_denom)
        upper_leg_joints = (np.arctan(x / z) - np.arctan((self._l2 * np.sin(lower_leg_joints)) / (self._l1 + (self._l2 * np.cos(lower_leg_joints)))))
        
        #add the precalculated angle offsets due to x displacement in upper leg
        #and lower link
        lower_leg_joints += self._ik_beta - self._ik_alpha
        upper_leg_joints += self._ik_alpha
        
        return np.hstack((hip_joints, upper_leg_joints, lower_leg_joints)).flatten()
