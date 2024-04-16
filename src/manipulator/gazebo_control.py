#!/usr/bin/env python3

from gazebo_msgs.srv import SetModelConfiguration
from std_msgs.msg import Float64
import rospy

class GazeboControl():
    """docstring for GazeboControl."""

    def __init__(self):

        self.rate = rospy.Rate(10)

        self.model_name = "robot"
        self.urdf_param_name = "robot"
        self.joint_names = ["joint_torso_to_head","joint_torso_to_torso_sensor_plat"]
        self.joint_positions = [0.0,0.0]

        rospy.Subscriber('/control/head', Float64, self.callback_head)
        rospy.Subscriber('/control/kinect_plat', Float64, self.callback_kinect_plat)

        rospy.loginfo('Waiting for "/gazebo/set_model_configuration" service')
        rospy.wait_for_service('/gazebo/set_model_configuration')
        self.model_config = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        rospy.loginfo('control/head is ready!')
        rospy.loginfo('control/kinect_plat is ready!')

        while not rospy.is_shutdown():
            self.rate.sleep()
            self.model_config(self.model_name, self.urdf_param_name, self.joint_names, self.joint_positions)

    def callback_head(self, msg):
        self.joint_positions[0] = msg.data
    def callback_kinect_plat(self, msg):
        self.joint_positions[1] = msg.data


if __name__ == "__main__":
    rospy.init_node('gazebo_control_hera')
    gc = GazeboControl()
