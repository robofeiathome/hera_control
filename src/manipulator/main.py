#!/usr/bin/env python3

import sys
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject

class Manipulator:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        self.arm.set_pose_reference_frame('manip_base_link')
        self.hand = moveit_commander.MoveGroupCommander('gripper')
        self.hand.set_max_acceleration_scaling_factor(1.0)
        self.hand.set_max_velocity_scaling_factor(1.0)
        self.head = moveit_commander.MoveGroupCommander('head')
        self.head.set_max_acceleration_scaling_factor(1.0)
        self.head.set_max_velocity_scaling_factor(1.0)

        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty_srv)
        self.display_trajectory_publisher = rospy.Publisher("/move_group_arm/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self._pub = rospy.Publisher('collision_object', CollisionObject, queue_size=10)

        self.tf = tf.TransformListener()
        self.tf.waitForTransform('manip_base_link', 'torso', rospy.Time(), rospy.Duration(1.0))

        self.execute_pose(self.head, 'up')
        self.execute_pose(self.arm, 'home')
        self.execute_pose(self.hand, 'open')

        self.box_name = "box"
        self.eef_link = self.arm.get_end_effector_link()

    def execute_pose(self, group, pose_name):
        group.set_named_target(pose_name)
        success = group.go(wait=True)
        return success

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manipulator', log_level=rospy.ERROR)
    manipulator = Manipulator()
    Poses(manipulator)
    Joints(manipulator)
    Furniture(manipulator)
    Motions(manipulator)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
