#!/usr/bin/env python3

from __future__ import division
import math
import rospy
import sys
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Point, Pose
import moveit_commander
from hera_control.srv import Head
from dynamixel_workbench_msgs.srv import DynamixelCommand


class Head:

    def __init__(self):
        self.goal = Float32()        

        rospy.Service('head_service', Head, self.handler)
        self.head = rospy.ServiceProxy('/dynamixel_controller/dynamixel_command', DynamixelCommand)
	    
        rospy.loginfo('Head ready!')
        self.reset()
        
    def reset(self, timeout = 0):

	    return self.head('', 9, 'Goal_Position', 2320)
    	
    def handler(self, request):
        type = request.type.lower()
        goal = request.goal

        goal.z = (2048*goal.z)/3.14

        if type == '':
            if goal.z != 0:
                success = self.head('', 9, 'Goal_Position', int(goal.z))
            else:
                rospy.loginfo('Para utilizar 0.0 digite "reset"')

        elif type == 'reset':
            success = self.reset()

        if success != None:
            return 'SUCCEEDED'
        else:
            return 'FAILED'

if __name__ == '__main__':

    rospy.init_node('head_service', log_level=rospy.INFO)
    Head()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
