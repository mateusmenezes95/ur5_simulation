from joint_trajectory_planner import JointTrajectoryPlanner 
import numpy as np

class CubicPolynomialPlanner(JointTrajectoryPlanner):
    def __init__(self, pos_i, pos_f, vel_i, vel_f, movement_duration = None,
                 time_f = None, time_i = 0.0, time_step = 0.1):
        if movement_duration is None and time_f is None:
            raise AttributeError("Either final time or movement duration " + 
                                 "shall be passed in " +
                                 "JointTrajectoryPlanner constructor")
        elif movement_duration is None:
            self._movement_duration = time_f - time_i
        elif time_f is None:
            self._time_f = time_i + movement_duration

        linear_system = np.array([[1., time_i, time_i**2, time_i**3],
                                  [0., 1., 2*time_i, 3*time_i**2],
                                  [1., time_f, time_f**2, time_f**3],
                                  [0., 1., 2*time_f, 3*time_f**2]])  
        results = np.array([pos_i, vel_i, pos_f, vel_f])        
        self._a = np.linalg.solve(linear_system, results)

        super(CubicPolynomialPlanner, self).__init__(time_i, time_step, 
                                                     self._time_f, 
                                                     self._movement_duration)
        

    def generate_position_profile(self):
        position_profile = []
        for t_current in self._timespan:
            t = t_current - self._time_i
            position_profile.append(
                self._a[0] + self._a[1]*t + self._a[2]*t**2 + self._a[3] *t**3)
        position_profile = np.around(position_profile, 8)
        return position_profile


    def generate_velocity_profile(self):
        velocity_profile = []
        for t_current in self._timespan:
            t = t_current - self._time_i
            velocity_profile.append(
                self._a[1] + 2*self._a[2]*t + 3*self._a[3]*t**2)
        velocity_profile = np.around(velocity_profile, 8)
        return velocity_profile


    def generate_acceleration_profile(self):
        acceleration_profile = []
        for t_current in self._timespan:
            t = t_current - self._time_i
            acceleration_profile.append(2*self._a[2] + 6*self._a[3]*t)
        acceleration_profile = np.around(acceleration_profile, 8)
        return acceleration_profile
