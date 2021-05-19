#!/usr/bin/env python

import rospy
from ur5_simulation.joint_controller import JointController
import ur5_simulation.chart_generation as cg
import numpy as np
import time

if __name__ == '__main__':
    rospy.init_node('joints_controller')

    joint_controller = JointController()
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
            rospy.loginfo('Command sent: %s' % joint_set)

            if max(abs(joint_set - joint_controller.get_joints_state())) < 0.05:    # avoid unnecessary movement
                rospy.loginfo('Joints are already in the state given')
            else:
                joint_controller.set_joints_state(joint_set)
                rospy.sleep(time_to_complete_movement * 1.5)

            joints_set_name.append('q' + str(joint_set_suffix))
            joint_set_suffix += 1

            sim_forward_kinematic = joint_controller.get_last_link_pose()
            self_forward_kinematic = sim_forward_kinematic + np.random.normal(0, 0.01, sim_forward_kinematic.shape)
            forward_kinematic_error = abs(sim_forward_kinematic - self_forward_kinematic).max()
            forward_kinematic_errors = np.append(forward_kinematic_errors, forward_kinematic_error)

            sim_inverse_kinematic = joint_controller.get_joints_state()
            self_inverse_kinematic = joint_controller.get_joints_state() + np.random.normal(0, 0.01, len(joint_set))
            inverse_kinematic_error = abs(sim_inverse_kinematic - self_inverse_kinematic).max()
            inverse_kinematic_errors = np.append(inverse_kinematic_errors, inverse_kinematic_error)

            joints_state_round = np.around(sim_inverse_kinematic, decimals=2)
            rospy.loginfo('Last link pose: \n%s' % np.around(sim_forward_kinematic, decimals=2))
            rospy.loginfo('Joints states: %s' % joints_state_round)

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
