#!/usr/bin/env python

import rospy
from ur5_simulation.joint_controller import JointController
import ur5_simulation.chart_generation as cg
from ur5_simulation.ur5_kinematics import UR5Kinematics
import numpy as np
import time

if __name__ == '__main__':
    rospy.init_node('joints_controller')

    joint_controller = JointController()
    ur5_kinematics = UR5Kinematics()
    joint_controller.init()

    time_step = rospy.get_param('~time_step', 10.0)
    time_to_complete_movement = joint_controller.get_time_to_complete_movement()

    joints_set = rospy.get_param('~joints_sets')

    while rospy.get_time() < 1:
        rospy.loginfo('Waiting for simulation time to be non-zero')
        time.sleep(0.1)

    joint_set_suffix = 0
    joints_set_name = []
    forward_kinematic_errors = np.array([], dtype=float)
    inverse_kinematic_errors = np.array([], dtype=float)

    loop_rate = rospy.Rate(1.0 / time_step)

    for joint_set in joints_set:
        if not rospy.is_shutdown():
            set_name = 'q' + str(joint_set_suffix)
            rospy.loginfo('Values of %s joint set sent to UR5 controller: %s' % (set_name, joint_set))

            if max(abs(joint_set - joint_controller.get_joints_state())) < 0.05:    # avoid unnecessary movement
                rospy.loginfo('Joints are already in the state given')
            else:
                joint_controller.set_joints_state(joint_set)
                rospy.loginfo('Waiting UR5 to complete movement...')
                rospy.sleep(time_to_complete_movement * 1.5)

            joints_set_name.append(set_name)
            joint_set_suffix += 1

            sim_forward_kinematic = joint_controller.get_last_link_pose()
            self_forward_kinematic = ur5_kinematics.get_forward_kinematics(joint_set, short=False)
            forward_kinematic_error = abs(sim_forward_kinematic - self_forward_kinematic).max()
            forward_kinematic_errors = np.append(forward_kinematic_errors, forward_kinematic_error)

            sim_inverse_kinematic = joint_controller.get_joints_state()
            self_inverse_kinematic = joint_controller.get_joints_state() + np.random.normal(0, 0.01, len(joint_set))
            inverse_kinematic_error = abs(sim_inverse_kinematic - self_inverse_kinematic).max()
            inverse_kinematic_errors = np.append(inverse_kinematic_errors, inverse_kinematic_error)

            rospy.debug('Last link pose from ROS: \n%s' % np.around(sim_forward_kinematic, decimals=2))
            rospy.debug('Last link pose from self computation: \n%s' % np.around(self_forward_kinematic, decimals=2))
            rospy.debug('Joints values from ROS: %s' % np.around(sim_inverse_kinematic, decimals=2))
            rospy.debug('Joints values from Self computation: %s' % np.around(self_inverse_kinematic, decimals=2))

            loop_rate.sleep()

    joints_set_name = np.array(joints_set_name)
    rospy.loginfo('Joints sets: %s' % joints_set_name)
    rospy.loginfo('Forward kinematic maximum errors: %s' % np.round(forward_kinematic_errors, decimals=2))
    rospy.loginfo('Inverse kinematic maximum errors: %s' % np.round(inverse_kinematic_errors, decimals=2))

    cg.generate_diference_between_kinematics_computation(
        joints_set_name,
        forward_kinematic_errors,
        inverse_kinematic_errors
    )
