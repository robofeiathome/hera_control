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
    pick_lixo = 4
    guardar_lixo = 5
    tirar_lixo = 6
    attack = 7
    place = 8
    point = 9
    find_trash = 10
    ready_to_pick = 11

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

        rospy.Service('manipulator', Manip3, self.handler)
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

    def display_planned_path_callback(self, data):
        self.plan = data.trajectory[0].joint_trajectory

    def execute(self):
        self.joint_trajectory.publish(self.plan)
        self.is_moving = False

    def effort_callback(self, data):
        self.gripper_effort_left = 0
        self.gripper_effort_right = 0
        self.gripper_effort_left = data.effort[2]
        self.gripper_effort_right = data.effort[3]  

    def feedback_callback(self, data):
        self.is_moving = False
        if self.update_start_state.get_num_connections() > 0:
            self.update_start_state.publish()

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

    def execute_cartesian_plan(self, waypoints, avoid_col = True):
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

    def execute_pose(self, pose_name):
        self.group.set_named_target(pose_name)
        success = self.execute_plan()
        return success

    def place(self):
        sucess = self.execute_pose('place3')
        if (success):
            rospy.sleep(2)
            self.open_gripper()
            self.execute_pose('attack')
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

    def half_close_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_HALF_CLOSED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_HALF_CLOSED)
        rospy.sleep(5)
        return True
    
    def open_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_OPENED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_OPENED)
        return True

    def close_gripper(self):
        self.gripper('', 8, 'Goal_Position', Manipulator.LEFT_GRIP_CLOSED)
        self.gripper('', 7, 'Goal_Position', Manipulator.RIGHT_GRIP_CLOSED)
        if abs(self.gripper_effort_right)>1000 or abs(self.gripper_effort_left>1000):
            print("peguei alguma coisa")
            success = True
        else:
            print("nada")
            success = False
        return success

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
       
    def remove_box(self,name):
        box_name = name
        scene = moveit_commander.PlanningSceneInterface()
        eef_link = self.group.get_end_effector_link()
        obj = moveit_python.PlanningSceneInterface("manip_base_link")
        scene.remove_attached_object(eef_link, name=box_name)
        success = obj.removeCollisionObject(name)
        if success != None:
            return True
        else:
            return False
    
    def pick_trash(self):
        self.open_gripper()
        self.gripper('', 1, 'Goal_Position', 1973)
        self.gripper('', 2, 'Goal_Position', 2432)
        self.gripper('', 3, 'Goal_Position', 2127)
        self.gripper('', 4, 'Goal_Position', 893)
        self.gripper('', 5, 'Goal_Position', 2069)
        self.gripper('', 6, 'Goal_Position', 1945)

    def pick_lixo(self):
        self.gripper('', 4, 'Goal_Position', 890)
        self.gripper('', 2, 'Goal_Position', 2700)
        self.gripper('', 6, 'Goal_Position', 2350)
        rospy.sleep(2)
        x = 2700
        y = 890
        z = 2200
        for i in range(3):
            x += 170
            y += 90
            z += 130
            self.gripper('', 2, 'Goal_Position', x)
            rospy.sleep(1)
            self.gripper('', 4, 'Goal_Position', y)
            self.gripper('', 6, 'Goal_Position', z)
        rospy.sleep(3)
        self.close_gripper()

    def find_cima(self):
        self.execute_pose('attack')
        self.gripper('', 1, 'Goal_Position', 2095)
        self.gripper('', 2, 'Goal_Position', 2200)
        self.gripper('', 3, 'Goal_Position', 2023)
        self.gripper('', 4, 'Goal_Position', 1000)
        self.gripper('', 5, 'Goal_Position', 2017)
        self.gripper('', 6, 'Goal_Position', 1700)

    def drop_lixo(self):
        self.gripper('', 1, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2300)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 800)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 6, 'Goal_Position', 2048)
        rospy.sleep(1)
        self.gripper('', 2, 'Goal_Position',  3200)
        rospy.sleep(1)
        self.gripper('', 6, 'Goal_Position', 2800)
        self.gripper('', 1, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 3000)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 1200)
        self.gripper('', 5, 'Goal_Position', 2300)
        self.gripper('', 6, 'Goal_Position', 2200)
        self.gripper('', 2, 'Goal_Position', 3400)
        rospy.sleep(1)
        self.gripper('', 4, 'Goal_Position', 1000)
        rospy.sleep(1)
        self.gripper('', 4, 'Goal_Position', 1600)

    def guardar_lixo(self):
        self.gripper('', 6, 'Goal_Position', 2048)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 800)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2300)
        self.gripper('', 1, 'Goal_Position', 2048)
        rospy.sleep(1)
        self.gripper('', 1, 'Goal_Position', 700)
        self.gripper('', 3, 'Goal_Position', 1771)
        self.gripper('', 4, 'Goal_Position', 1500)
        self.gripper('', 6, 'Goal_Position', 1500)
        rospy.sleep(4)
        self.open_gripper()
        rospy.sleep(2)
        self.gripper('', 6, 'Goal_Position', 2048)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2300)
        self.gripper('', 1, 'Goal_Position', 2048)
        rospy.sleep(3)
        self.execute_pose('home')   

    def point(self,ang):
        self.gripper('', 2, 'Goal_Position', 1450)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 600)
        self.gripper('', 6, 'Goal_Position', 2200)
        self.gripper('', 1, 'Goal_Position', ang)
        return True
        
    def tirar_lixo(self):
        self.open_gripper()
        self.gripper('', 6, 'Goal_Position', 2048)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 800)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2300)
        self.gripper('', 1, 'Goal_Position', 2048)
        rospy.sleep(4)
        self.gripper('', 1, 'Goal_Position', 500)
        self.gripper('', 4, 'Goal_Position', 1200)
        self.gripper('', 6, 'Goal_Position', 1700)
        rospy.sleep(4)   
        self.gripper('', 4, 'Goal_Position', 900)
        self.gripper('', 6, 'Goal_Position', 1900)
        rospy.sleep(2)
        self.close_gripper()
        self.gripper('', 6, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 1500)
        self.gripper('', 1, 'Goal_Position', 2048)
    
    def ready_to_pick(self):
        self.gripper('', 1, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2422)
        self.gripper('', 3, 'Goal_Position', 2043)
        self.gripper('', 4, 'Goal_Position', 1108)
        self.gripper('', 5, 'Goal_Position', 2047)
        self.gripper('', 6, 'Goal_Position', 2803)
        rospy.sleep(4)

    def table_place(self):
        self.gripper('', 1, 'Goal_Position', 2048)
        self.gripper('', 2, 'Goal_Position', 2800)
        self.gripper('', 3, 'Goal_Position', 2048)
        self.gripper('', 4, 'Goal_Position', 1500)
        self.gripper('', 5, 'Goal_Position', 2048)
        self.gripper('', 6, 'Goal_Position', 2700)
        rospy.sleep(4)
    

    def pub_poses(self, request):
        pose_names = [n.name for n in list(ManipPoses)]
        return Manip_posesResponse(pose_names)

    def calibrate_position(self,pose):
        pose.position.x -= 0.15
        pose.position.y += 0.12
        pose.position.z  += 0.1
        return pose
        
    def handler(self, request):
        type = request.type.lower()
        goal = request.goal
        quaternion = tf.transformations.quaternion_from_euler(goal.rx, goal.ry, goal.rz)
        pose = Pose()
        pose.position.x = goal.x
        pose.position.y = goal.y
        pose.position.z = goal.z
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        
        if type == 'reset':
            success = self.execute_pose('reset')
        elif type == 'home':
            success = self.execute_pose('home')
        elif type == 'open':
            success = self.open_gripper()
        elif type == 'half_close':
            success = self.half_close_gripper()
        elif type == 'close':
            success = self.close_gripper()
        elif type == 'pick_lixo':
            success = self.pick_lixo()
        elif type == 'pick_trash':
            success = self.pick_trash()
        elif type == 'find_trash':
            self.execute_pose('attack')
            success = self.execute_pose('find_trash')
        elif type == 'drop_lixo':
            success = self.drop_lixo()
        elif type == 'guardar_lixo':
            success = self.guardar_lixo() 
        elif type == 'tirar_lixo':
            success = self.tirar_lixo()  
        elif type == 'ready_to_pick':
            success = self.ready_to_pick()
        elif type == 'find_cima':
            success = self.find_cima()
        elif type == 'table_place':
            success = self.table_place()
        elif type == 'take_out_trash_lid':
            self.execute_pose('find_trash')
            self.close_gripper()  
            self.execute_pose('attack')
            success = self.execute_pose('storing_trash_lid')            
        elif type == 'attack':
            success = self.execute_pose('attack')   
        elif type == 'giro':
            success = self.base_orientation(pose.position.y, pose.position.x)
        elif type == 'pick':
            pose = self.calibrate_position(pose)
            self.clear_octomap()
            self.execute_pose('home')
            self.open_gripper() 
            self.execute_pose('attack')
            self.base_orientation(pose.position.y,pose.position.x)            
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
            self.add_box('box',pose)
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
    
        elif type == 'place_moveit':
            success = self.execute_pose('place')
            rospy.sleep(2)
            self.open_gripper()

        elif type == 'place':
            pose = self.calibrate_position(pose)
            tcp_pose = self.tf.transformPose('manip_base_link', self.group.get_current_pose()).pose
            tcp_pose.position.x += 0.1
            self.add_box('box',tcp_pose)
            rospy.sleep(2)
            self.clear_octomap()
            self.attach_box('box')
            pose.position.z += 0.15
            target_pose = copy.deepcopy(pose)
            self.group.set_pose_target(target_pose)
            success = self.execute_plan()
            if success:
                pose.position.z -= 0.15
                target_pose = copy.deepcopy(pose)
                self.group.set_pose_target(target_pose)
                success = self.execute_plan()
                if success:
                    rospy.sleep(2)
                    self.open_gripper() 
                    rospy.sleep(1)
                    self.remove_box('box') 
                    rospy.sleep(1)
                    self.execute_pose('attack')

        elif type == 'point':
            angle = (2048*angle)/1
            self.close_gripper()
            if (angle < 1024):
                angle = 1024
            elif (angle > 3072):
                angle = 3072
            success = self.point(int(angle)) 

        elif type == 'point_people':
            angle = 2356 + (-0.96*pixel)
            self.close_gripper()
            if (int(angle) < 1741):
                angle = 1741
            elif (int(angle) > 2355):
                angle = 2355
            success = self.point(int(angle))
            rospy.sleep(1)
            self.gripper('', 2, 'Goal_Position', 2000)
            self.gripper('', 6, 'Goal_Position', 2800)
        
        else:
            self.group.set_pose_target(pose)
            success = self.execute_plan()
            
        if success:
            return "success"
        else:
            return "failed"

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manipulator', log_level=rospy.INFO)
    Manipulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
