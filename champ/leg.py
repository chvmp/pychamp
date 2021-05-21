import math
import copy
import numpy as np
from champ.joint import Joint
from champ.geometry import *
from champ.types import Point, GaitConfig


class Leg(object):
    def __init__(self, parent):
        self._parent = parent
        
        self._zero_stances = np.zeros((4,3))
        self._zero_stances_vectorized = False

        self._center_to_nominal = 0.0
        self._center_to_nominal_calculated = False

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