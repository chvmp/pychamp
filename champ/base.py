import copy
import numpy as np
from champ.leg import Leg
from champ.joint import JointVector
from champ.types import GaitConfig
from champ.geometry import translate, rotate_x
import champ.urdf as urdf

class Base(object):
    def __init__(self, robot_profile=None, load_from_server=False, link_names=None):
        self._joint_positions = 12 * [0]
        self.lf = Leg()
        self.rf = Leg()
        self.lh = Leg()
        self.rh = Leg()

        self.legs = [self.lf, self.rf, self.lh, self.rh]

        self.hips = JointVector(self, 0)
        self.upper_legs = JointVector(self, 1)
        self.lower_legs = JointVector(self, 2)
        self.feet = JointVector(self, 3)

        self.joints = [self.hips, 
                       self.upper_legs,
                       self.lower_legs,
                       self.feet]

        self._leg_zero_stances = np.zeros((4,3))
        self._leg_zero_stances_vectorized = False

        self._joint_positions = np.zeros(12)

        self._theta_vectors = []

        if link_names is not None:
            self.__load_translations(link_names)
        elif robot_profile is not None:
            self.__load_translations(robot_profile.link_names, 
                                     robot_profile.urdf)

    def init(self):
        self.hips.translation
        self.upper_legs.translation
        self.lower_legs.translation
        self.feet.translation
        self._leg_zero_stances

    @property
    def joint_positions(self):
        return self._joint_positions

    @joint_positions.setter
    def joint_positions(self, joint_positions):
        self._joint_positions = np.array((joint_positions))
        self.hips.theta = self._joint_positions[0::3, np.newaxis]
        self.upper_legs.theta = self._joint_positions[1::3, np.newaxis]
        self.lower_legs.theta = self._joint_positions[2::3, np.newaxis]
        for i in range(4):
            index = i * 3
            self.legs[i].hip.theta = joint_positions[index]
            self.legs[i].upper_leg.theta = joint_positions[index + 1]
            self.legs[i].lower_leg.theta = joint_positions[index + 2]

    @property
    def zero_stances(self):
        if not self._leg_zero_stances_vectorized:
            self._leg_zero_stances_vectorized = True
            self._leg_zero_stances = translate(self._leg_zero_stances, self.hips.translation)
            self._leg_zero_stances = translate(self._leg_zero_stances, self.upper_legs.translation)
            
            lower_trans = np.zeros((4,3))
            lower_trans[:, 2] = self.lower_legs.translation[:, 2] + self.feet.translation[:, 2]
            
            self._leg_zero_stances = translate(self._leg_zero_stances, lower_trans)
            self._leg_zero_stances.flags.writeable = False

        return self._leg_zero_stances

    def transform_to_hip(self, foot_positions):
        foot_positions -= self.hips.translation
        foot_positions = rotate_x(foot_positions, -self.hips.theta)
        return foot_positions
    
    def transform_to_base(self, foot_positions):
        foot_positions = rotate_x(foot_positions, self.hips.theta)
        foot_positions = translate(foot_positions, self.hips.translation)

        return foot_positions

    def __get_knee_direction(self, direction):
        if direction == '>':
            return -1
        elif direction == '<':
            return 1
        else:
            return -1

    def __load_translations(self, link_names, urdf_path=None):
        translations = urdf.get_translations(link_names, urdf_path)

        for i in range(4):
            base_idx = i * 4
            self.legs[i].joints[0].set_origin(translations[base_idx][0], 
                                              translations[base_idx][1],
                                              translations[base_idx][2])

            self.legs[i].joints[1].set_origin(translations[base_idx+1][0], 
                                              translations[base_idx+1][1],
                                              translations[base_idx+1][2])

            self.legs[i].joints[2].set_origin(translations[base_idx+2][0], 
                                              translations[base_idx+2][1],
                                              translations[base_idx+2][2])

            self.legs[i].joints[3].set_origin(translations[base_idx+3][0], 
                                              translations[base_idx+3][1],
                                              translations[base_idx+3][2])

        self.init()