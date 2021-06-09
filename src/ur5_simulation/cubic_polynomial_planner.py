from joint_trajectory_planner import JointTrajectoryPlanner 
import numpy as np

class CubicPolynomialPlanner(JointTrajectoryPlanner):
    def __init__(self, pos_i, pos_f, vel_i, vel_f, movement_duration,
                 time_i = 0.0, time_step = 0.1):

        time_f = time_i + movement_duration

        linear_system = np.array([[1., time_i, time_i**2, time_i**3],
                                  [0., 1., 2*time_i, 3*time_i**2],
                                  [1., time_f, time_f**2, time_f**3],
                                  [0., 1., 2*time_f, 3*time_f**2]])  
        results = np.array([pos_i, vel_i, pos_f, vel_f])        
        self._a = np.linalg.solve(linear_system, results)

        super(CubicPolynomialPlanner, self).__init__(time_i, time_step, time_f, 
                                                     movement_duration)
        

    def generate_position_profile(self):
        position_profile = []
        for t in self._timespan:
            position_profile.append(
                self._a[0] + self._a[1]*t + self._a[2]*t**2 + self._a[3] *t**3)
        position_profile = np.around(position_profile, 8)
        return position_profile


    def generate_velocity_profile(self):
        velocity_profile = []
        for t in self._timespan:
            velocity_profile.append(
                self._a[1] + 2*self._a[2]*t + 3*self._a[3]*t**2)
        velocity_profile = np.around(velocity_profile, 8)
        return velocity_profile


    def generate_acceleration_profile(self):
        acceleration_profile = []
        for t in self._timespan:
            acceleration_profile.append(2*self._a[2] + 6*self._a[3]*t)
        acceleration_profile = np.around(acceleration_profile, 8)
        return acceleration_profile
