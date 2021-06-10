#!/usr/bin/env python

import rospy
import rospkg
from ur5_simulation.joint_controller import JointController
import ur5_simulation.chart_generation as cg
from ur5_simulation.ur5_kinematics import UR5Kinematics
import numpy as np
import time
import os.path

if __name__ == '__main__':
    rospy.init_node('joints_controller')

    joint_controller = JointController()
    ur5_kinematics = UR5Kinematics()
    joint_controller.init()

    time_step = rospy.get_param('~time_step', 10.0)
    time_to_complete_movement = joint_controller.get_time_to_complete_movement()

    is_to_plot_charts = rospy.get_param('~plot_charts')
    is_to_write_poses_file = rospy.get_param('~write_poses_list_file')
    
    set_picked = 'set_' + str(rospy.get_param('~set_to_pick'))
    dataset_pam = rospy.get_param('~datasets')
    shoulder_position = dataset_pam[set_picked]['shoulder_position']
    elbow_position = dataset_pam[set_picked]['elbow_position']
    wrist_position = dataset_pam[set_picked]['wrist_position']
    joints_set_list = dataset_pam[set_picked]['joints_set_list']
    poses_set_list = dataset_pam[set_picked]['poses_set_list']

    while rospy.get_time() < 1:
        rospy.loginfo('Waiting for simulation time to be non-zero')
        time.sleep(0.1)

    forward_kinematic_max_errors = np.array([], dtype=float)
    inverse_kinematic_max_errors = np.array([], dtype=float)
    inverse_kinematic_errors = np.zeros(shape=(len(joints_set_list), len(joints_set_list[0])))
    forward_kinematic_translation_errors = np.zeros(shape=(len(joints_set_list), 3))
    end_effector_poses = []

    loop_rate = rospy.Rate(1.0 / time_step)

    previous_time = rospy.get_time()
    actual_time = previous_time
    while actual_time < previous_time + 1:
        actual_time = rospy.get_time()
        rospy.loginfo('Waiting for steady simulation time')
        time.sleep(1)

    joint_controller.open_gripper()

    for joint_set_index, joint_set in enumerate(joints_set_list):
        if not rospy.is_shutdown():
            rospy.loginfo('Values of joint set %s sent to UR5 controller: %s' % (joint_set_index, joint_set))

            if max(abs(joint_set - joint_controller.get_joints_state())) < 0.05:    # avoid unnecessary movement
                rospy.loginfo('Joints are already in the state given')
            else:
                joint_controller.set_joints_state(joint_set)
                rospy.loginfo('Waiting UR5 to complete movement...')
                rospy.sleep(time_to_complete_movement * 1.5)

            sim_forward_kinematic = joint_controller.get_last_link_pose()
            end_effector_poses.append(sim_forward_kinematic.tolist())
            self_forward_kinematic = ur5_kinematics.calculate_forward_kinematics(joint_set, short=False)
            forward_kinematic_max_error = (abs(sim_forward_kinematic) - abs(self_forward_kinematic)).max()
            forward_kinematic_max_errors = np.append(forward_kinematic_max_errors, forward_kinematic_max_error)
            forward_kinematic_translation_error = (sim_forward_kinematic[0:3, 3] - self_forward_kinematic[0:3, 3]).T
            forward_kinematic_translation_errors[joint_set_index] = forward_kinematic_translation_error

            sim_inverse_kinematic = joint_controller.get_joints_state()
            self_inverse_kinematic = ur5_kinematics.calculate_inverse_kinematics(sim_forward_kinematic, short=False,
                                                                                 shoulder=shoulder_position,
                                                                                 elbow=elbow_position,
                                                                                 wrist=wrist_position)
            inverse_kinematic_error = np.cos(sim_inverse_kinematic) - np.cos(self_inverse_kinematic)
            inverse_kinematic_errors[joint_set_index] = inverse_kinematic_error
            inverse_kinematic_max_error = abs(inverse_kinematic_error).max()
            inverse_kinematic_max_errors = np.append(inverse_kinematic_max_errors, inverse_kinematic_max_error)

            rospy.logdebug('Joints errors of set %s: %s' % (joint_set_index, inverse_kinematic_error))
            rospy.logdebug('Last link pose from ROS: \n%s' % np.around(sim_forward_kinematic, decimals=2))
            rospy.logdebug('Last link pose from self computation: \n%s' % np.around(self_forward_kinematic, decimals=2))
            rospy.logdebug('Joints values from ROS: %s' % np.around(sim_inverse_kinematic, decimals=2))
            rospy.logdebug('Joints values from Self computation: %s' % np.around(self_inverse_kinematic, decimals=2))

            loop_rate.sleep()

    rospy.loginfo('Tranlation x errors %s' % np.round(forward_kinematic_translation_errors[:,0], decimals=5))
    rospy.loginfo('Tranlation y errors %s' % np.round(forward_kinematic_translation_errors[:,1], decimals=5))
    rospy.loginfo('Tranlation z errors %s' % np.round(forward_kinematic_translation_errors[:,2], decimals=5))
    rospy.loginfo('Forward kinematic maximum errors: %s' % np.round(forward_kinematic_max_errors, decimals=2))
    rospy.loginfo('Inverse kinematic maximum errors: %s' % np.round(inverse_kinematic_max_errors, decimals=2))

    cg.generate_diference_between_kinematics_computation( forward_kinematic_max_errors, inverse_kinematic_max_errors)
    cg.generate_translation_errors(forward_kinematic_translation_errors)
    cg.generate_joints_errors(inverse_kinematic_errors)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('ur5_simulation')

    if is_to_write_poses_file:
        with open(os.path.join(pkg_path, 'config/poses_list.txt') , 'w') as poses_list_txt:
            for pose_num, end_effector_pose in enumerate(end_effector_poses):
                poses_list_txt.write('- [')
                for row_num, row in enumerate(end_effector_pose):
                    if row_num == 0:
                        poses_list_txt.write(str(row) + ',\n')
                    elif row_num == 3:
                        poses_list_txt.write('   ' + str(row) + '] # ' + str(pose_num) + '\n')
                    else:
                        poses_list_txt.write('   ' + str(row) + ',\n')

    if is_to_plot_charts:
        cg.show_charts()
