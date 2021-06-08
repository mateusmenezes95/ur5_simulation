import abc
import rospy
import numpy as np
import matplotlib.pyplot as plt

class JointTrajectoryPlanner(object):
    def __init__(self, time_i = 0.0, time_step = 0.1, time_f = None,
                 movement_duration = None):
        self._time_step = time_step
        self._time_i = time_i
        self._time_f = time_f
        self._movement_duration = movement_duration
        print('Final time is ', time_f)
        print('Movement duration is ', movement_duration)

        if movement_duration is None and time_f is None:
            raise AttributeError("Either final time or movement duration " + 
                                 "shall be passed in " +
                                 "JointTrajectoryPlanner constructor")
        elif movement_duration is None:
            self._movement_duration = time_f - time_i
        elif time_f is None:
            self._time_f = time_i + movement_duration

        breakpoint_qty = int(self._movement_duration / self._time_step + 1)
        self._timespan = np.linspace(self._time_i, self._time_f, breakpoint_qty)

        (self._position_profile, self._velocity_profile,
         self._acceleration_profile) = self.generate_profiles()

    def generate_profiles(self):
        return (self.generate_position_profile(),
                self.generate_velocity_profile(), 
                self.generate_acceleration_profile())

    # @abc.abstractmethod
    def generate_position_profile(self):
        pass

    # @abc.abstractmethod
    def generate_velocity_profile(self):
        pass

    # @abc.abstractmethod
    def generate_acceleration_profile(self):
        pass
    
    def __get_info_at(self, time, vector):
        if time < self._time_i:
            rospy.logwarn("Requested trajectory information before its " +
                          "beginning")
            return time[0]
        elif time > self._time_f:
            rospy.logwarn("Requested trajectory information after its " +
                          "ending")
            return time[-1]
        else:
            return np.interp(time, self._timespan, vector)

    def get_position_at(self, time):
        return self.__get_info_at(time, self._position_profile)

    def get_velocity_at(self, time):
        return self.__get_info_at(time, self._velocity_profile)

    def get_acceleration_at(self, time):
        return self.__get_info_at(time, self._acceleration_profile)

    def plot_position_profile(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(self._timespan, self._position_profile)
        ax.set_title('Position Trajectory')
        ax.set_xlabel('Time / s')
        ax.set_ylabel('Position / rad')
        fig.tight_layout()
        fig.show()

    def plot_velocity_profile(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(self._timespan, self._velocity_profile)
        ax.set_title('Velocity Trajectory')
        ax.set_xlabel('Time / s')
        ax.set_ylabel('Velocity / rad/s')
        fig.tight_layout()
        fig.show()
    
    def plot_acceleration_profile(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(self._timespan, self._acceleration_profile)
        ax.set_title('Acceleration Trajectory')
        ax.set_xlabel('Time / s')
        ax.set_ylabel('Acceleration / rad/s\u00b2')
        fig.tight_layout()
        fig.show()
        