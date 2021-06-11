#!/usr/bin/env python

import rospy
import rospkg
from ur5_simulation.joint_position_controller import JointPositionController
import ur5_simulation.chart_generation as cg
from ur5_simulation.ur5_kinematics import UR5Kinematics
import numpy as np
import time
import os.path

if __name__ == '__main__':
    rospy.init_node('joints_position_controller')

    joint_controller = JointPositionController()
    ur5_kinematics = UR5Kinematics()
    joint_controller.init()

    time_step = rospy.get_param('~time_step', 10.0)
    time_to_complete_movement = joint_controller.get_time_to_complete_movement()
    
    set_picked = 'set_' + str(rospy.get_param('~set_to_pick'))
    joint_controller_type = str(rospy.get_param('~joint_controller'))
    dataset_pam = rospy.get_param('~datasets')
    shoulder_position = dataset_pam[set_picked]['shoulder_position']
    elbow_position = dataset_pam[set_picked]['elbow_position']
    wrist_position = dataset_pam[set_picked]['wrist_position']
    joints_set_list = dataset_pam[set_picked]['joints_set_list']
    poses_set_list = dataset_pam[set_picked]['poses_set_list']
    joint_velocity = dataset_pam[set_picked]['velocity']
    joint_movement_duration = dataset_pam[set_picked]['movement_duration']
    sample_period = dataset_pam[set_picked]['sample_period']

    while rospy.get_time() < 1:
        rospy.loginfo('Waiting for simulation time to be non-zero')
        time.sleep(0.1)

    loop_rate = rospy.Rate(1.0 / time_step)

    previous_time = rospy.get_time()
    actual_time = previous_time
    while actual_time < previous_time + 1:
        actual_time = rospy.get_time()
        rospy.loginfo('Waiting for steady simulation time')
        time.sleep(1)

    joint_controller.open_gripper()

    for pose_set_index, pose_set in enumerate(poses_set_list):
        pose_set = np.array(pose_set)
        if not rospy.is_shutdown():
            rospy.loginfo('Values of pose set %s sent to UR5 controller: %s' % (pose_set_index, pose_set))

            # if max(abs(pose_set - joint_controller.get_last_link_pose())) < 0.05:    # avoid unnecessary movement
            #     rospy.loginfo('Joints are already in the state given')
            # else:
            joint_set = ur5_kinematics.calculate_inverse_kinematics(pose_set, short=False,
                                                                    shoulder=shoulder_position,
                                                                    elbow=elbow_position,
                                                                    wrist=wrist_position)
            rospy.loginfo('Joint = %s' % np.round(joint_set, decimals=2))
            if pose_set_index == 4:
                joint_controller.close_gripper()
                rospy.sleep(3)
            joint_controller.set_joints_state(joint_set, joint_movement_duration,
                                                joint_velocity, sample_period, joint_controller_type)
            rospy.loginfo('Waiting UR5 to complete movement...')
            



            # rospy.loginfo('Last link pose from ROS: \n%s' % np.around(sim_forward_kinematic, decimals=2))

            loop_rate.sleep()
