import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cubic_polynomial_planner import CubicPolynomialPlanner
from lspb_planner import LSPBPlanner
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import sys

class JointPositionController:
    def __init__(self):
        self.__joints_cmd_pub = rospy.Publisher('/joint_group_position_controller/command', Float64MultiArray, queue_size=1)
        self.__gripper_cmd_pub = rospy.Publisher('/gripper_position_controller/command', JointTrajectory, queue_size=1)
        self.__tf_buffer = tf2_ros.Buffer()
        self.__tf_listener = tf2_ros.TransformListener(self.__tf_buffer)

        self.__joints_state = JointState()
        self.__joints_index = {}
        self.__joints_name = []

    def init(self):
        self.__base_link = rospy.get_param('~base_link_name', 'base_link')
        self.__last_link = rospy.get_param('~last_link', 'link6')
        self.__time_to_complete_movement = rospy.get_param('~time_to_complete_movement', 1.0)
        joints_map = rospy.get_param('~joints_remap', None)
        for joint_map in joints_map:
            self.__joints_index[joint_map.keys()[0]] = joint_map.values()[0]

    def get_time_to_complete_movement(self):
        return self.__time_to_complete_movement
        
    def __tf_to_homogeneous_matrix(self, tf):
        rotation = tf2_geometry_msgs.transform_to_kdl(tf).M
        translation = tf2_geometry_msgs.transform_to_kdl(tf).p

        H = np.zeros((4, 4), dtype=float)

        for i, j in np.ndindex((3,3)):
            H[i][j] = rotation[(i, j)]

        for i in range(3):
            H[i][3] = translation[i]

        H[3][3] = 1.0

        return H

    def __get_joints_state_ordered(self):
        joints_state_ordered = JointState()
        try:
            joints_state_ordered = rospy.wait_for_message('/joint_states', JointState, timeout=0.5)
        except rospy.ROSException as error:
            rospy.logerr('It was not possible to get joint states: ' + str(error))
            sys.exit(0)

        joints_name = []
        joints_position = []

        for i in range(len(joints_state_ordered.name)):
            if joints_state_ordered.name[i] in self.__joints_index:
                joints_name.append(joints_state_ordered.name[i])
                joints_position.append(joints_state_ordered.position[i])

        joints_name_len = len(joints_name)

        joints_state_ordered.name = [''] * joints_name_len
        joints_state_ordered.position = [0] * joints_name_len

        for (joint_name, joint_position) in zip(joints_name, joints_position):
            joints_state_ordered.name[self.__joints_index[joint_name]] = joint_name
            joints_state_ordered.position[self.__joints_index[joint_name]] = joint_position

        return joints_state_ordered

    def set_joints_state(self, q, movement_duration, joint_velocity, sample_period, joint_controller_type):
        joints_name = self.__get_joints_state_ordered().name
        time_i = rospy.Time.now().to_sec()
        current_joint_states = self.get_joints_state()
        q_len = len(q)
        planners = [None]*q_len
        for joint_index, joint in enumerate(q):
            if (joint_controller_type == "lspb"):
                planners[joint_index] = LSPBPlanner(pos_i = current_joint_states[joint_index],
                                                    pos_f = joint, 
                                                    desired_velocity =joint_velocity,
                                                    time_i = time_i, 
                                                    movement_duration = movement_duration,
                                                    time_step = sample_period)
            elif (joint_controller_type == "cubic_polynomial"):
                planners[joint_index] = CubicPolynomialPlanner(pos_i = current_joint_states[joint_index],
                                                               pos_f = joint, 
                                                               vel_i =0.0, vel_f =0.0,
                                                               time_i = time_i, 
                                                               movement_duration = movement_duration,
                                                               time_step = sample_period)
               

            # print("pos_i = {}".format(current_joint_states[joint_index])) 
            # print("pos_f = {}".format(joint)) 
            # print("desired_velocity = {}".format(joint_velocity)) 
            # print("time_i = {}".format(time_i)) 
            # print("movement_duration = {}".format(movement_duration)) 
            # print("time_step = {}".format(sample_period)) 
            # planners[joint_index].plot_position_profile()
        
        if joint_controller_type == "lspb" or joint_controller_type == "cubic_polynomial":
            while rospy.Time.now().to_sec() < time_i + movement_duration:
                current_time = rospy.Time.now().to_sec()
                joints_cmd = Float64MultiArray() 
                for planner in planners:
                    joints_cmd.data.append(planner.get_position_at(current_time))
                self.__joints_cmd_pub.publish(joints_cmd)
                rospy.sleep(sample_period)
        else:
            joints_cmd = Float64MultiArray() 
            joints_cmd.data = q
            self.__joints_cmd_pub.publish(joints_cmd)
            rospy.sleep(self.__time_to_complete_movement)


    def get_joints_state(self):
        return np.array(self.__get_joints_state_ordered().position)

    def get_homogeneous_tf(self, source_frame, target_frame):
        try:
            tf = self.__tf_buffer.lookup_transform(source_frame, target_frame, rospy.Time(0))
            return self.__tf_to_homogeneous_matrix(tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            rospy.logerr('It was not possible get tf between ' 
                          + target_frame + ' and ' + source_frame + ':' + str(error))
            sys.exit(0)


    def get_last_link_pose(self):
        return self.get_homogeneous_tf(self.__base_link, self.__last_link)

    def cmd_gripper(self, percentage):
        gripper_cmd_msg = JointTrajectory()
        gripper_cmd_msg.header.stamp = rospy.Time.now()
        gripper_cmd_msg.joint_names = ['finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [percentage/100.]
        point.time_from_start.secs = 3.0

        gripper_cmd_msg.points.append(point)

        self.__gripper_cmd_pub.publish(gripper_cmd_msg)

    def open_gripper(self):
        self.cmd_gripper(1)

    def close_gripper(self):
        self.cmd_gripper(10)