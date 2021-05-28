import copy
import math
import numpy as np
from champ.types import TwistVector, PoseVector
from champ.leg import Leg
from champ.joint import Joint
from champ.geometry import translate, translate_x, translate_y, translate_z, rotate_x
import champ.urdf as urdf


class Base(object):
    def __init__(self, robot_profile=None):        
        self.legs = Leg(self)

        self.hips = Joint(self, 0)
        self.upper_legs = Joint(self, 1)
        self.lower_legs = Joint(self, 2)
        self.feet = Joint(self, 3)

        self.joints = [self.hips, 
                       self.upper_legs,
                       self.lower_legs,
                       self.feet]

        self._mass = 0.0
        self._moment_of_inertia = np.zeros((3,3))
        self._dimensions = (0, 0, 0)
        self._position = np.zeros((3,1))
        self.velocity = TwistVector()
        self.pose = PoseVector()

        if robot_profile is not None:
            if len(robot_profile.link_names) > 0 and len(robot_profile.urdf) > 0:
                self.set_origins_from_urdf(
                    robot_profile.link_names, 
                    robot_profile.urdf
                )

    def init(self):
        self.legs.zero_stances

    @property
    def mass(self):
        return self._mass

    @mass.setter
    def mass(self, mass):
        self._mass = mass
        self._update_inertial_matrix()

    @property
    def dimensions(self):
        return self._dimensions

    @dimensions.setter
    def dimensions(self, dimensions):
        self._dimensions = dimensions
        self._update_inertial_matrix()

    @property
    def moment_of_inertia(self):
        return self._moment_of_inertia

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

    def set_origins_from_urdf(self, link_names, urdf_path=None):
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

    def _update_inertial_matrix(self):
        length, width, height = self._dimensions
        print(type(length))       

        self._moment_of_inertia[0, 0] = (1 / 12) * self._mass * (width * width + \
                                        height * height)
        self._moment_of_inertia[1, 1] = (1 / 12) * self._mass * (length * length + \
                                        height * height)
        self._moment_of_inertia[2, 2] = (1 /12) * self._mass * (length * length + \
                                        width * width)