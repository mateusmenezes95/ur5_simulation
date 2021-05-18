import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import sys

class JointController:
    def __init__(self):
        self.__joints_cmd_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)
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

    def set_joints_state(self, q):
        joints_name = self.__get_joints_state_ordered().name

        q_len = len(q)
        num_joints = len(joints_name)

        if q_len != num_joints:
            rospy.logwarn('Joint set not accepted: Number of joints to control is diferent of robot''s joints number')
            rospy.logwarn('Required %d joints. %d was given ' % (num_joints, q_len))
            return

        joints_cmd = JointTrajectory()
        joints_cmd.header.stamp = rospy.Time.now()
        joints_cmd.joint_names = joints_name

        point = JointTrajectoryPoint()
        point.positions = q
        point.time_from_start = rospy.Duration(self.__time_to_complete_movement)

        joints_cmd.points.append(point)

        self.__joints_cmd_pub.publish(joints_cmd)

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
