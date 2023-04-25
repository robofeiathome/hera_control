#!/usr/bin/env python3

from sre_constants import SUCCESS
import sys
import copy
import math
import rospy
import tf
from tf import transformations 
import moveit_commander
import moveit_python
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import DisplayTrajectory
from dynamixel_workbench_msgs.srv import DynamixelCommand
from sensor_msgs.msg import JointState
from hera_control.srv import Manip
from hera_control.srv import Manip_poses, Manip_posesResponse
from std_srvs.srv import Empty as Empty_srv
from enum import Enum

class ManipPoses(Enum):
    reset = 0
    home = 1
    open = 2
    close = 3
    attack = 4
    place = 5
    point = 6
    ready_to_pick = 7

class Manipulator:

    black_gripper = False 

    if (black_gripper):
        #id 8
        LEFT_GRIP_OPENED = 2300
        LEFT_GRIP_CLOSED = 1850
        LEFT_GRIP_HALF_CLOSED = 2003

        #id 7
        RIGHT_GRIP_OPENED = 1800
        RIGHT_GRIP_CLOSED = 2250
        RIGHT_GRIP_HALF_CLOSED = 2139
    else:
        #id 8
        LEFT_GRIP_OPENED = 2040
        LEFT_GRIP_CLOSED = 1499
        LEFT_GRIP_HALF_CLOSED = 2003

        #id 7
        RIGHT_GRIP_OPENED = 2067
        RIGHT_GRIP_CLOSED = 2584
        RIGHT_GRIP_HALF_CLOSED = 2139

    def __init__(self):

        # Initialize ROS node and moveit_commander
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.group.set_planning_time(10)
        self.group.set_pose_reference_frame('manip_base_link')
        self.hand = moveit_commander.MoveGroupCommander('gripper')
        self.hand.set_planning_time(10)
        self.hand.set_pose_reference_frame('manip_base_link')

        self.joint_trajectory = rospy.Publisher('/dynamixel_controller/joint_trajectory', JointTrajectory, queue_size=10)
        self.update_start_state = rospy.Publisher('/rviz/moveit/update_start_state', Empty, queue_size=10)

        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.display_planned_path_callback)
        rospy.Subscriber('/dynamixel_controller/feedback', Empty, self.feedback_callback)
        rospy.Subscriber('/dynamixel_controller/joint_states', JointState , self.effort_callback)


        self.gripper = rospy.ServiceProxy('/dynamixel_controller/dynamixel_command', DynamixelCommand)
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty_srv)

        rospy.Service('manipulator', Manip, self.handler)
        rospy.Service('manipulator_poses', Manip_poses, self.pub_poses)

        self.tf = tf.TransformListener()
        self.tf.waitForTransform('manip_base_link', 'torso', rospy.Time(), rospy.Duration(1.0))

        self.is_moving = False
        self.plan = None
        self.gripper_effort_right = 0.0
        self.gripper_effort_left = 0.0

        rospy.loginfo('Going Home in 2 seconds...')
        rospy.sleep(1)
        self.execute_pose('home')
        self.open_gripper()

        def handler(self, request):

            function_name = request.type.lower()
            coordinates = request.goal

            quaternion = tf.transformations.quaternion_from_euler(coordinates.rx, coordinates.ry, coordinates.rz)
            pose = Pose()
            pose.position.x = coordinates.x
            pose.position.y = coordinates.y
            pose.position.z = coordinates.z
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            functions = {
                'reset': lambda: self.execute_pose('reset'),
                'home': lambda: self.execute_pose('home'),
                'attack': lambda: self.execute_pose('attack'),
                'pick': lambda pose: self.pick(pose),
                'place': lambda: self.place(),
                'open': lambda: self.open_gripper(),
                'half_close': lambda: self.half_close_gripper(),
                'close': lambda: self.close_gripper(),
                'ready_to_pick': lambda: self.ready_to_pick(),
                'giro': lambda pose: self.base_orientation(pose.position.y, pose.position.x),
                'point': lambda pose: self.point(pose.position.x),
                'point_people': lambda pose: self.point_people(pose),
                '': lambda pose: self.go_to_coordinates(pose),
            }

            try:
                result = functions[function_name](*coordinates)
                return result
            except KeyError:
                rospy.logerr('Invalid function name %s' % function_name)
                return False


    def add_box(self,name,pose):
        scene2 = moveit_commander.PlanningSceneInterface(synchronous=True)
        pose_target = PoseStamped() # Create a new pose for the box
        pose_target.header.frame_id = "manip_base_link"
        pose_target.pose.position.x = pose.position.x + 0.1 #
        pose_target.pose.position.y = pose.position.y
        pose_target.pose.position.z = pose.position.z
        pose_target.pose.orientation = pose.orientation
        success = scene2.add_box(name, pose_target, size=(0.1, 0.1, 0.18))
        rospy.sleep(2)
        if success != None:
            return True
        else:
            return False

    def attach_box(self,name):
        box_name = name
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        eef_link = self.group.get_end_effector_link()
        grasping_group = "gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        success = scene.attach_box(eef_link, box_name, touch_links=touch_links)
        if success != None:
            return True
        else:
            return False

    def base_orientation(self, y, x):
        x = x + 0.17
        y = y - 0.06
        angle = math.atan2(y,x)
        x = 652*(angle+1.57)+1024
        if (652*(angle+1.57)+1024) > 2048:
            x+=90
        self.gripper('', 1, 'Goal_Position',int(x))

    def calibrate_position(self,pose):
        pose.position.x -= 0.15
        pose.position.y += 0.12
        pose.position.z += 0.1
        return pose

    def close_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_CLOSED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_CLOSED)
        if abs(self.gripper_effort_right) > 1000 or abs(self.gripper_effort_left > 1000):
            print("peguei alguma coisa")
            success = True
        else:
            print("nada")
            success = False
        return success

    def display_planned_path_callback(self, data):
        self.plan = data.trajectory[0].joint_trajectory

    def effort_callback(self, data):
        self.gripper_effort_left = 0
        self.gripper_effort_right = 0
        self.gripper_effort_left = data.effort[2]
        self.gripper_effort_right = data.effort[3]

    def execute(self):
        self.joint_trajectory.publish(self.plan)
        self.is_moving = False

    def execute_cartesian_plan(self, waypoints, avoid_col=True):
        plan, fraction = self.group.compute_cartesian_path(waypoints, 0.001, 0, avoid_collisions)
        rospy.loginfo('Cartesian path fraction: %.2f.' % fraction)
        rospy.sleep(2)
        if self.plan != None and fraction > 0.9:
            self.execute()
            while self.is_moving:
                pass
            rospy.sleep(5)
            self.plan = None
            return True
        else:
            self.plan = None
            return False

    def execute_plan(self):
        self.group.plan()
        rospy.sleep(2)
        if self.plan != None:
            self.execute()
            while self.is_moving:
                pass
            rospy.sleep(2)
            self.plan = None
            return True
        else:
            return False

    def execute_pose(self, pose_name):
        self.group.set_named_target(pose_name)
        success = self.execute_plan()
        return success

    def feedback_callback(self, data):
        self.is_moving = False
        if self.update_start_state.get_num_connections() > 0:
            self.update_start_state.publish()

    def go_to_coordinates(self,pose):
        self.group.set_pose_target(pose)
        success = self.execute_plan()

    def half_close_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_HALF_CLOSED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_HALF_CLOSED)
        rospy.sleep(5)
        return True
    
    def open_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_OPENED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_OPENED)
        return True

    def pick(self,pose):
        pose = self.calibrate_position(pose)
        self.clear_octomap()
        self.execute_pose('home')
        self.open_gripper()
        self.execute_pose('attack')
        self.base_orientation(pose.position.y, pose.position.x)
        target_pose = copy.deepcopy(pose)
        self.group.set_pose_target(target_pose)
        if pose.orientation.z > 0:
            pose.position.x += 0.05 * math.cos(goal.rz)
            pose.position.y += 0.05 * math.sin(goal.rz)
        else:
            pose.position.x += 0.05 * math.cos(goal.rz)
            pose.position.y -= 0.05 * math.sin(goal.rz)
        pose.position.z -= 0.15
        pose.position.y -= 0.03
        self.add_box('box', pose)
        rospy.sleep(2)
        self.clear_octomap()
        self.attach_box('box')
        success = self.execute_plan()
        rospy.sleep(4)
        self.close_gripper()
        # self.close_gripper()
        rospy.sleep(1)
        self.remove_box('box')
        rospy.sleep(1)
        self.execute_pose('attack')

    def place(self):
        success = self.execute_pose('place')
        rospy.sleep(2)
        if (success):
            rospy.sleep(2)
            self.open_gripper()
            self.execute_pose('attack')
        else:
            return False

    def point(self,angle):
        self.close_gripper()
        angle = (2048 * angle) / 1
        if (angle < 1024):
            angle = 1024
        elif (angle > 3072):
            angle = 3072

        self.gripper('', 2, 'Goal_Position', 1450)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 600)
        self.gripper('', 6, 'Goal_Position', 2200)
        self.gripper('', 1, 'Goal_Position', angle)
        return True

    def point_people(self, angle):
        angle = 2356 + (-0.96 * pixel)
        self.close_gripper()
        if (int(angle) < 1741):
            angle = 1741
        elif (int(angle) > 2355):
            angle = 2355
        success = self.point(int(angle))
        rospy.sleep(1)
        self.gripper('', 2, 'Goal_Position', 2000)
        self.gripper('', 6, 'Goal_Position', 2800)

    def pub_poses(self, request):
        pose_names = [n.name for n in list(ManipPoses)]
        return Manip_posesResponse(pose_names)

    def ready_to_pick(self):
        self.gripper('', 1, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2422)
        self.gripper('', 3, 'Goal_Position', 2043)
        self.gripper('', 4, 'Goal_Position', 1108)
        self.gripper('', 5, 'Goal_Position', 2047)
        self.gripper('', 6, 'Goal_Position', 2803)
        rospy.sleep(4)

    def remove_box(self,name):
        box_name = name
        scene = moveit_commander.PlanningSceneInterface()
        eef_link = self.group.get_end_effector_link()
        obj = moveit_python.PlanningSceneInterface("manip_base_link")
        scene.remove_attached_object(eef_link, name=box_name)
        success = obj.removeCollisionObject(name)
        if success is not None:
            return True
        else:
            return False


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manipulator', log_level=rospy.INFO)
    Manipulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
