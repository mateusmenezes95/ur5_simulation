from time import time
from joint_trajectory_planner import * 
import numpy as np

from ur5_simulation.joint_trajectory_planner import calculate_displacement

class LSPBPlanner(JointTrajectoryPlanner):
    def __init__(self, pos_i, pos_f, desired_velocity, time_i = 0.0,
                 time_step = 0.1, time_f = None, movement_duration = None):
        # type: (float, float, float, float, float, float, float) -> None

        self._movement_duration = movement_duration
        self._time_f = time_f
        if movement_duration is None and time_f is None:
            raise AttributeError("Either final time or movement duration " + 
                                 "shall be passed in " +
                                 "JointTrajectoryPlanner constructor")
        elif movement_duration is None:
            self._movement_duration = time_f - time_i
        elif time_f is None:
            self._time_f = time_i + movement_duration
        self._displacement = calculate_displacement(pos_i, pos_f)

        self._desired_velocity = None
        if self._displacement >= 0:
            self._desired_velocity = desired_velocity
        else:
            self._desired_velocity = - desired_velocity

        if self._desired_velocity >= 0:
            minimum_velocity = self._displacement/self._movement_duration
            maximum_velocity = 2*(self._displacement)/self._movement_duration
        else:
            maximum_velocity = self._displacement/self._movement_duration
            minimum_velocity = 2*(self._displacement)/self._movement_duration

        print("Max vel = {}, Min vel = {}".format(maximum_velocity, minimum_velocity))

        if self._desired_velocity >= maximum_velocity or \
           self._desired_velocity < minimum_velocity:
            print("Desired velocity is not possible")
            self._desired_velocity = (maximum_velocity + minimum_velocity) / 2
        
        self._blend_time = (pos_i - pos_f + 
                            self._desired_velocity * self._movement_duration) \
                            / self._desired_velocity

        self._acceleration = self._desired_velocity / self._blend_time
        self._pos_i = pos_i
        self._pos_f = pos_f

        super(LSPBPlanner, self).__init__(time_i, time_step, time_f, 
                                          self._movement_duration)
        

    def generate_position_profile(self):
        position_profile = []
        for t_current in self._timespan:
            t = t_current - self._time_i 
            t_final = self._time_f - self._time_i 
            if t < self._blend_time:
                position_profile.append(
                    self._pos_i + (self._acceleration * t**2) / 2)
            elif t >= self._blend_time and t <= t_final - self._blend_time:
                position_profile.append(
                    (self._pos_i + self._pos_f - self._desired_velocity *
                     t_final) / 2 + (self._desired_velocity * t))
            elif t > t_final - self._blend_time and t <= t_final:
                position_profile.append(
                    self._pos_f - 
                    ((self._acceleration * t_final ** 2) / 2) +
                    (self._acceleration * t_final * t) - 
                    ((self._acceleration / 2) * t ** 2))
        position_profile = np.around(position_profile, 8)
        return position_profile


    def generate_velocity_profile(self):
        velocity_profile = []
        for t_current in self._timespan:
            t = t_current - self._time_i  
            t_final = self._time_f - self._time_i 
            if t < self._blend_time:
                velocity_profile.append(self._acceleration * t)
            elif t >= self._blend_time and t <= t_final - self._blend_time:
                velocity_profile.append(self._desired_velocity)
            elif t > t_final - self._blend_time and t <= t_final:
                velocity_profile.append(
                    self._desired_velocity - 
                    (t - (self._movement_duration - self._blend_time)) *
                    self._acceleration)
        velocity_profile = np.around(velocity_profile, 8)
        return velocity_profile


    def generate_acceleration_profile(self):
        acceleration_profile = []
        for t_current in self._timespan:
            t = t_current - self._time_i  
            t_final = self._time_f - self._time_i 
            if t < self._blend_time:
                acceleration_profile.append(self._acceleration)
            elif t >= self._blend_time and t <= t_final - self._blend_time:
                acceleration_profile.append(0)
            elif t > t_final - self._blend_time and t <= t_final:
                acceleration_profile.append(-self._acceleration)
        acceleration_profile = np.around(acceleration_profile, 8)
        return acceleration_profile
