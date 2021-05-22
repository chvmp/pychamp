import time
import numpy as np

SECONDS_TO_MICROS = 1000000.0


class TDEvent(object):
    def __init__(self, stance_duration=0.25, swing_duration=0.25):
        self._swing_phase_period = swing_duration * SECONDS_TO_MICROS
        self._stance_phase_period = stance_duration  * SECONDS_TO_MICROS
        self._stride_period = self._stance_phase_period + self._swing_phase_period
        
        self._elapsed_time_ref = 0.0
        self._last_touch_down = int(round(time.perf_counter() * 1000000))
        self._leg_clocks = np.zeros((4,1))
        self._stance_phase_signal =  np.zeros((4,1))
        self._swing_phase_signal = np.zeros((4,1))

        self._has_swung = False

        self._phase_delay_trot= np.array([[0.0], [0.5], [0.5], [0.0]])
        self._phase_delay_zero = np.array([[0.5], [0.0], [0.0], [0.5]])
        self._delta_delay =  self._phase_delay_zero - self._phase_delay_trot
        self._phase_delay= np.array([[0.0], [0.5], [0.5], [0.0]])
        self._init_transit_time = 0.0

    def run(self, step_length, time):
        if step_length == 0.0:
            self._has_swung = False
            return self._stance_phase_signal, self._swing_phase_signal
   
        if time - self._last_touch_down >= self._stride_period:
            self._last_touch_down = time

        # elif self._init_transit_time == 0.0:
        #     self._init_transit_time = time
        # elif self._last_touch_down == 0:  
        #     print('hi')
        #     sat_x = np.where(self._delta_delay > 0,
        #                      (time - self._init_transit_time) / self._stride_period,
        #                      self._delta_delay)
        #     sat = np.where(sat_x <= 1,
        #                    sat_x,
        #                    1)
        #     self._phase_delay = self._phase_delay_zero + (self._delta_delay * sat)
        #     print(self._phase_delay)
        
        if self._elapsed_time_ref > self._stride_period:
            self._elapsed_time_ref = self._stride_period
        else:
            self._elapsed_time_ref = time - self._last_touch_down

        self._leg_clocks = self._elapsed_time_ref - (self._phase_delay * self._stride_period)

        self._stance_phase_signal = np.where((self._leg_clocks > 0) & (self._leg_clocks < self._stance_phase_period), 
                                              self._leg_clocks / self._stance_phase_period,
                                              0.0)
        
        self._swing_phase_signal = np.where((self._leg_clocks > -self._swing_phase_period) & (self._leg_clocks < 0), 
                                            (self._leg_clocks + self._swing_phase_period) / self._swing_phase_period,
                                             0.0)

        self._swing_phase_signal = np.where((self._leg_clocks > self._stance_phase_period) & (self._leg_clocks < self._stride_period), 
                                            (self._leg_clocks - self._stance_phase_period) / self._swing_phase_period,
                                             self._swing_phase_signal)

        if not self._has_swung and self._stance_phase_signal[0,0] < 0.5:
            self._stance_phase_signal[0,0] = 0.5
            self._stance_phase_signal[3,0] = 0.5
            self._swing_phase_signal[1,0] = 0.5
            self._swing_phase_signal[2,0] = 0.5
        else:
            self._has_swung = True
        
        return self._swing_phase_signal, self._stance_phase_signal