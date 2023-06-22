#!/usr/bin/env python3

from __future__ import division
import sys
import rospy
import moveit_commander
from std_msgs.msg import Float32MultiArray, Float32, Float64
from hera_control.srv import Head_service


class Head:
    def __init__(self):
        self.goal = Float32()
        rospy.Service('head_interface', Head_service, self.handler)
        moveit_commander.roscpp_initialize(sys.argv)
        # self.pub = rospy.Publisher("/zed_controller/command", Float64, queue_size=10)
        self.head = moveit_commander.MoveGroupCommander('zed')
        self.head.set_max_acceleration_scaling_factor(1.0)
        self.head.set_max_velocity_scaling_factor(1.0)
    
        self.execute_pose('up')

    def handler(self, request,):
        function_name = request.type.lower()
        pose = request.goal

        functions = {
            'up': lambda pose=None: self.execute_pose('up'),
            'down': lambda pose=None: self.execute_pose('down'),
        }

        try:
            result = functions[function_name](pose)
            return str(result)

        except KeyError:
            rospy.logerr('Invalid function name %s' % function_name)
            return "Invalid function name: {}".format(function_name)
    
    def execute_pose(self, pose_name):
        head = self.head
        head.set_named_target(pose_name)
        success = head.go(wait=True)
        head.stop()
        return success


if __name__ == '__main__':
    rospy.init_node('head_interface', log_level=rospy.INFO)
    Head()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
