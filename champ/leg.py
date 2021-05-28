import math
import copy
import numpy as np
from champ.joint import Joint
from champ.geometry import *
from champ.types import Point


class Leg(object):
    def __init__(self, parent):
        self._parent = parent

        self._zero_stances = np.zeros((4,3))
        self._zero_stances_vectorized = False

        self._center_to_nominal = 0.0
        self._center_to_nominal_calculated = False

        self._contact_states = np.zeros((4,1))

        self._joints_position = np.zeros(12)
        self._joints_velocity = np.zeros(12)

    @property
    def zero_stances(self):
        if not self._zero_stances_vectorized:
            self._zero_stances_vectorized = True
            self._zero_stances = translate(self._zero_stances, self._parent.hips.translation.xyz)
            self._zero_stances = translate(self._zero_stances, self._parent.upper_legs.translation.xyz)
            
            trans_z = self._parent.lower_legs.translation.z + self._parent.feet.translation.z
            
            self._zero_stances = translate_z(self._zero_stances, trans_z)
            self._zero_stances.flags.writeable = False

        return self._zero_stances

    @property
    def center_to_nominal(self):
        if not self._center_to_nominal_calculated:
            self._center_to_nominal_calculated = True
            x = self._parent.hips.translation.x[0,0] + self._parent.upper_legs.translation.x[0,0]
            y = self._parent.hips.translation.y[0,0] + self._parent.upper_legs.translation.y[0,0]
            self._center_to_nominal = math.sqrt((x * x) + (y * y))
        
        return self._center_to_nominal

    @property
    def jacobian(self):
        q = self._joints_position.reshape((4,3))

        jacobian = np.zeros((4,3,3))
        sin_lu = np.sin(q[:, 2] + q[:, 1])
        cos_lu = np.cos(q[:, 2] + q[:, 1])

        sin_h = np.sin(q[:, 0])
        cos_h = np.cos(q[:, 0])
        sin_u = np.sin(q[:, 1])
        cos_u = np.cos(q[:, 1])
    
        foot_x_sin_lu = self._parent.feet.translation.xyz[:, 0]  *  sin_lu
        foot_x_cos_lu = self._parent.feet.translation.xyz[:, 0]  *  cos_lu 
        foot_z_sin_lu = self._parent.feet.translation.xyz[:, 2]  *  sin_lu

        lower_leg_x_cos_u = self._parent.lower_legs.translation.xyz[:, 0]  *  cos_u
        lower_leg_z_sin_u = self._parent.lower_legs.translation.xyz[:, 2]  *  sin_u

        jacobian[:, 0, 0] = 0.0
        jacobian[:, 0, 1] = -foot_x_sin_lu + \
                            self._parent.feet.translation.xyz[:, 2] * cos_lu - \
                            self._parent.lower_legs.translation.xyz[:, 0] * sin_u + \
                            self._parent.lower_legs.translation.xyz[:, 2] * cos_u

        jacobian[:, 0, 2] = -foot_x_sin_lu + \
                            self._parent.feet.translation.xyz[:, 2] * cos_lu

        jacobian[:, 1, 0] = foot_x_sin_lu * cos_h - \
                            self._parent.feet.translation.xyz[:, 1] * sin_h - \
                            self._parent.feet.translation.xyz[:, 2] * cos_h * cos_lu + \
                            self._parent.lower_legs.translation.xyz[:, 0] * sin_u * cos_h - \
                            self._parent.lower_legs.translation.xyz[:, 1] * sin_h - \
                            self._parent.lower_legs.translation.xyz[:, 2] * cos_h * cos_u - \
                            self._parent.upper_legs.translation.xyz[:, 1] * sin_h - \
                            self._parent.upper_legs.translation.xyz[:, 2] * cos_h

        jacobian[:, 1, 1] = (foot_x_cos_lu + foot_z_sin_lu + \
                            lower_leg_x_cos_u + lower_leg_z_sin_u) * sin_h

        jacobian[:, 1, 2] = (foot_x_cos_lu + foot_z_sin_lu) * sin_h

        jacobian[:, 2, 0] = self._parent.feet.translation.xyz[:, 0] * sin_h * sin_lu + \
                            self._parent.feet.translation.xyz[:, 1] * cos_h - \
                            self._parent.feet.translation.xyz[:, 2] * sin_h * cos_lu + \
                            self._parent.lower_legs.translation.xyz[:, 0] * sin_h * sin_u + \
                            self._parent.lower_legs.translation.xyz[:, 1] * cos_h - \
                            self._parent.lower_legs.translation.xyz[:, 2] * sin_h * cos_u + \
                            self._parent.upper_legs.translation.xyz[:, 1] * cos_h - \
                            self._parent.upper_legs.translation.xyz[:, 2] * sin_h

        jacobian[:, 2, 1] = -(foot_x_cos_lu + foot_z_sin_lu + \
                            lower_leg_x_cos_u + lower_leg_z_sin_u) * cos_h
                            
        jacobian[: ,2, 2] = -(foot_x_cos_lu + foot_z_sin_lu) * cos_h
        
        return jacobian  

    @property
    def contact_states(self):
        return self._contact_states

    @contact_states.setter
    def contact_states(self, contact_states):
        self._contact_states[0, 0] = contact_states[0]
        self._contact_states[1, 0] = contact_states[1]
        self._contact_states[2, 0] = contact_states[2]
        self._contact_states[3, 0] = contact_states[3]

    @property
    def joint_states(self):
        return self._joints_position, self._joints_velocity

    @joint_states.setter
    def joint_states(self, joint_states):
        joint_positions, joints_velocity = joint_states
        if joint_positions is not None:
            self._joints_position = np.array(joint_positions)
            self._parent.hips.theta = self._joints_position[0::3, np.newaxis]
            self._parent.upper_legs.theta = self._joints_position[1::3, np.newaxis]
            self._parent.lower_legs.theta = self._joints_position[2::3, np.newaxis]

        if joints_velocity is not None:
            self._joints_velocity = np.array(joints_velocity)
            self._parent.hips.velocity = self._joints_velocity[0::3, np.newaxis]
            self._parent.upper_legs.velocity = self._joints_velocity[1::3, np.newaxis]
            self._parent.lower_legs.velocity = self._joints_velocity[2::3, np.newaxis]
