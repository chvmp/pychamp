import math
import copy
import numpy as np
from champ.joint import Joint
from champ.geometry import *
from champ.types import Point, GaitConfig

class Leg(object):
    def __init__(self):
        self._no_of_links = 0.0
        self._id = 0
        self._last_touchdown = 0.0
        self._in_contact = 1
        self._gait_phase = 1

        self._zero_stance = np.zeros((1, 3))

        self.hip = Joint()
        self.upper_leg = Joint()
        self.lower_leg = Joint()
        self.foot = Joint()

        self.joints = [self.hip,
                       self.upper_leg,
                       self.lower_leg,
                       self.foot]

        self._center_to_nominal = 0.0
        self._center_to_nominal_calculated = False

        self._zero_stance = np.zeros([1,3])
        self._zero_stance_calculated = False

    @property
    def upper_leg_from_hip(self):
        return copy.deepcopy(self.upper_leg.translation)

    @property
    def lower_leg_from_hip(self):
        lower_leg_position = np.zeros((1,3))
        for i in range(2, 0, -1):
            lower_leg_position = translate(lower_leg_position, self.joints[i].translation)
            if i > 1:
                lower_leg_position = rotate_y(lower_leg_position, self.joints[i-1].theta)
        
        return lower_leg_position

    @property
    def foot_from_hip(self):
        foot_position = np.zeros((1,3))
        for i in range(3, 0, -1):
            foot_position = translate(foot_position, self.joints[i].translation)
            if i > 1:
                foot_position = rotate_y(foot_position, self.joints[i-1].theta)
        
        return foot_position

    @property
    def hip_from_base(self):
        return copy.deepcopy(self.hip.translation)

    @property
    def upper_leg_from_base(self):
        return self.transform_to_base(self.upper_leg_from_hip)

    @property
    def lower_leg_from_base(self):
        return self.transform_to_base(self.lower_leg_from_hip)

    @property
    def foot_from_base(self):
        return self.transform_to_base(self.foot_from_hip)

    def transform_to_hip(self, foot_position):
        foot_position[0, 0] -= self.hip.translation[0, 0]
        foot_position[0, 1] -= self.hip.translation[0, 1]

        return foot_position

    def transform_to_base(self, foot_position):
        foot_positions = translate(foot_position, self.hip.translation)
        foot_positions = rotate_x(foot_position, self.hip.theta)

        return foot_position

    @property
    def zero_stance(self):
        if not self._zero_stance_calculated:
            self._zero_stance_calculated = True
            self._zero_stance = translate(self._zero_stance, self.hip.translation)
            self._zero_stance = translate(self._zero_stance, self.upper_leg.translation)
            self._zero_stance[0, 2] = self.lower_leg.z + self.foot.z
            
            self._zero_stance.flags.writeable = False

        return self._zero_stance

    @property
    def center_to_nominal(self):
        if not self._center_to_nominal_calculated:
            self._center_to_nominal_calculated = True
            x = self.hip.x + self.upper_leg.x
            y = self.hip.y + self.upper_leg.y
            self._center_to_nominal = math.sqrt((x * x) + (y * y))
        
        return self._center_to_nominal

    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, id):
        self._id = id

    @property
    def last_touchdown(self):
        return self._last_touchdown

    @last_touchdown.setter
    def last_touchdown(self, current_time):
        self._last_touchdown = current_time

    @property
    def in_contact(self):
        return self._in_contact
    
    @in_contact.setter
    def in_contact(self, in_contact):
        self._in_contact = in_contact

    @property
    def gait_phase(self, phase):
        return self._gait_phase

    @gait_phase.setter
    def gait_phase(self):
        self._gait_phase = phase
    