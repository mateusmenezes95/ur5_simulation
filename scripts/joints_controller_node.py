#!/usr/bin/env python

import rospy
from ur5_simulation.joint_controller import JointController
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

    previous_time = rospy.get_time()
    actual_time = previous_time
    while actual_time < previous_time + 1:
        actual_time = rospy.get_time()
        rospy.loginfo('Waiting for steady simulation time')
        time.sleep(1)

    loop_rate = rospy.Rate(1.0 / time_step)
    complete_movement_rate = rospy.Rate(1.0 / time_to_complete_movement)

    for joint_set in joints_set:
        if not rospy.is_shutdown():
            rospy.loginfo('Command sent: %s' % joint_set)

            if max(abs(joint_set - joint_controller.get_joints_state())) < 0.05:    # avoid unnecessary movement
                rospy.loginfo('Joints are already in the state given')
            else:
                joint_controller.set_joints_state(joint_set)
                complete_movement_rate.sleep()
                
            homogenous_matrix = joint_controller.get_last_link_pose()
            joints_state_round = np.around(joint_controller.get_joints_state(), decimals=2)

            rospy.loginfo('Last link pose: \n%s' % np.around(homogenous_matrix, decimals=2))
            rospy.loginfo('Joints states: %s' % joints_state_round)

            loop_rate.sleep()
