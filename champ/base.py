import copy
import math
import numpy as np
from champ.leg import Leg
from champ.joint import Joint
from champ.types import GaitConfig
from champ.geometry import translate, translate_x, translate_y, translate_z, rotate_x
import champ.urdf as urdf

class Base(object):
    def __init__(self, robot_profile=None, load_from_server=False, link_names=None):        
        self.legs = Leg(self)

        self.hips = Joint(self, 0)
        self.upper_legs = Joint(self, 1)
        self.lower_legs = Joint(self, 2)
        self.feet = Joint(self, 3)

        self.joints = [self.hips, 
                       self.upper_legs,
                       self.lower_legs,
                       self.feet]

        self._joint_positions = np.zeros(12)

        if link_names is not None:
            self.__load_translations(link_names)
        elif robot_profile is not None:
            self.__load_translations(robot_profile.link_names, 
                                     robot_profile.urdf)

    def init(self):
        self.legs.zero_stances

    @property
    def joint_positions(self):
        return self._joint_positions

    @joint_positions.setter
    def joint_positions(self, joint_positions):
        self._joint_positions = np.array((joint_positions))
        self.hips.theta = self._joint_positions[0::3, np.newaxis]
        self.upper_legs.theta = self._joint_positions[1::3, np.newaxis]
        self.lower_legs.theta = self._joint_positions[2::3, np.newaxis]

    def transform_to_hip(self, foot_positions, leg_id=None):
        if leg_id is None:
            foot_positions = translate(foot_positions, -self.hips.translation.xyz)
        else:
            foot_positions = translate(foot_positions, -self.hips.translation.xyz[leg_id, :])

        return foot_positions
    
    def transform_to_base(self, foot_positions, leg_id=None):
        if leg_id is not None:
            foot_positions = translate(foot_positions, self.hips.translation.xyz)
        else:
            foot_positions = translate(foot_positions, self.hips.translation.xyz[leg_id, :])

        return foot_positions

    def __load_translations(self, link_names, urdf_path=None):
        translations = urdf.get_translations(link_names, urdf_path)

        for i in range(4):
            base_idx = i * 4

            self.hips.set_origin(
                i,
                translations[base_idx][0], 
                translations[base_idx][1],
                translations[base_idx][2]
            )

            self.upper_legs.set_origin(
                i,
                translations[base_idx+1][0], 
                translations[base_idx+1][1],
                translations[base_idx+1][2]
            )

            self.lower_legs.set_origin(
                i,
                translations[base_idx+2][0], 
                translations[base_idx+2][1],
                translations[base_idx+2][2]
            )

            self.feet.set_origin(
                i,
                translations[base_idx+3][0], 
                translations[base_idx+3][1],
                translations[base_idx+3][2]
            )

        self.init()