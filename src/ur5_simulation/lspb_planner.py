from joint_trajectory_planner import JointTrajectoryPlanner 
import numpy as np

class LSPBPlanner(JointTrajectoryPlanner):
    def __init__(self, pos_i, pos_f, desired_velocity, time_i = 0.0,
                 time_step = 0.1, time_f = None, movement_duration = None):

        self._movement_duration = time_f - time_i
        displacement = pos_f - pos_i
        minumum_velocity = displacement/self._movement_duration
        maximum_velocity = 2*(-displacement)/self._movement_duration
        if desired_velocity >= maximum_velocity and \
           desired_velocity < minumum_velocity:
            raise AttributeError("Velocity shall be less then {} and higher " +
                                 "than {}.".format(maximum_velocity, 
                                                   minumum_velocity))
        
        self._blend_time = (pos_i - pos_f + 
                            desired_velocity * self._movement_duration) \
                            / desired_velocity

        self._desired_velocity = desired_velocity
        self._acceleration = desired_velocity / self._blend_time
        self._pos_i = pos_i
        self._pos_f = pos_f

        super(LSPBPlanner, self).__init__(time_i, time_step, time_f, 
                                          self._movement_duration)
        

    def generate_position_profile(self):
        position_profile = []
        for t in self._timespan:
            if t < self._blend_time:
                position_profile.append(
                    self._pos_i + (self._acceleration * t**2) / 2)
            elif t >= self._blend_time and t <= self._time_f - self._blend_time:
                position_profile.append(
                    (self._pos_i + self._pos_f - self._desired_velocity *
                     self._time_f) / 2 + (self._desired_velocity * t))
            elif t > self._time_f - self._blend_time and t <= self._time_f:
                position_profile.append(
                    self._pos_f - 
                    ((self._acceleration * self._time_f ** 2) / 2) +
                    (self._acceleration * self._time_f * t) - 
                    ((self._acceleration / 2) * t ** 2))
        position_profile = np.around(position_profile, 8)
        return position_profile


    def generate_velocity_profile(self):
        velocity_profile = []
        for t in self._timespan:
            if t < self._blend_time:
                velocity_profile.append(self._acceleration * t)
            elif t >= self._blend_time and t <= self._time_f - self._blend_time:
                velocity_profile.append(self._desired_velocity)
            elif t > self._time_f - self._blend_time and t <= self._time_f:
                velocity_profile.append(
                    self._desired_velocity - 
                    (t - (self._movement_duration - self._blend_time)) *
                    self._acceleration)
        velocity_profile = np.around(velocity_profile, 8)
        return velocity_profile


    def generate_acceleration_profile(self):
        acceleration_profile = []
        for t in self._timespan:
            if t < self._blend_time:
                acceleration_profile.append(self._acceleration)
            elif t >= self._blend_time and t <= self._time_f - self._blend_time:
                acceleration_profile.append(0)
            elif t > self._time_f - self._blend_time and t <= self._time_f:
                acceleration_profile.append(-self._acceleration)
        acceleration_profile = np.around(acceleration_profile, 8)
        return acceleration_profile
