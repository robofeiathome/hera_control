#!/usr/bin/env python3

from __future__ import division
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from hera_control.srv import Head_service
from dynamixel_workbench_msgs.srv import DynamixelCommand


class Head:
    def __init__(self):
        self.goal = Float32()
        rospy.Service('head', Head_service, self.handler)
        self.head = rospy.ServiceProxy('/dynamixel_controller/dynamixel_command', DynamixelCommand)
        rospy.loginfo('Head ready!')
        self.reset()
        
    def reset(self):
        success = self.head('', 9, 'Goal_Position', 2048)
        return success

    def handler(self, request):
        type = request.type.lower()
        goal = request.goal

        goal.z = (2048 * goal.z) / 3.14

        if type == 'reset':
            success = self.reset()
        else:
            success = self.head('', 9, 'Goal_Position', int(goal.z))

        if success:
            return "succeeded"
        else:
            return "failed"


if __name__ == '__main__':
    rospy.init_node('head_interface', log_level=rospy.INFO)
    Head()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
