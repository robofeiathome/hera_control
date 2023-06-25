#!/usr/bin/env python3

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from hera_control.srv import Manip_service
from std_srvs.srv import Empty as Empty_srv
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane


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
        self.head = moveit_commander.MoveGroupCommander("zed")
        self.head.set_max_acceleration_scaling_factor(1.0)
        self.head.set_max_velocity_scaling_factor(1.0)

        self._objects = dict()


        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty_srv)
        self.display_trajectory_publisher = rospy.Publisher("/move_group_arm/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)  
        self._pub = rospy.Publisher('collision_object',
                                        CollisionObject,
                                        queue_size=10)


        rospy.Service('manipulator', Manip_service, self.handler)
        self.tf = tf.TransformListener()
        self.tf.waitForTransform('manip_base_link', 'torso', rospy.Time(), rospy.Duration(1.0))

        self.execute_pose(self.arm,'home')
        self.execute_pose(self.hand,'open')
 

        self.box_name = "box"
        self.eef_link = self.arm.get_end_effector_link()

    def handler(self, request):
        function_name = request.type.lower()
        self.coordinates = request.goal

        
        quaternion = tf.transformations.quaternion_from_euler(self.coordinates.rx, self.coordinates.ry, self.coordinates.rz)
        quaternion[0] = 0.0
        quaternion[1] = 0.0
        quaternion[2] = 0.0
        quaternion[3] = 1.0
        pose = Pose(position=Point(self.coordinates.x, self.coordinates.y, self.coordinates.z), orientation=Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))


        functions = {
            'reset': lambda pose=None: self.execute_pose(self.arm,'reset'),
            'home': lambda pose=None: self.execute_pose(self.arm,'home'),
            'attack': lambda pose=None: self.execute_pose(self.arm,'attack'),
            'open': lambda pose=None: self.execute_pose(self.hand,'open'),
            'close': lambda pose=None: self.execute_pose(self.hand,'close'),
            'up': lambda pose=None: self.execute_pose(self.head,'up'),
            'down': lambda pose=None: self.execute_pose(self.head,'down'),
            'sg_place_1': lambda pose=None: self.execute_pose(self.arm, 'place'),
            'pick': lambda pose: self.pick(pose),
            'place': lambda pose: self.place(pose),
            'cartesian_path': lambda pose: self.cartesian_path(pose),
            '': lambda pose: self.go_to_coordinates(pose),
        }

        try:
            result = functions[function_name](pose)
            return str(result)

        except KeyError:
            rospy.logerr('Invalid function name %s' % function_name)
            return "Invalid function name: {}".format(function_name)


    def add_box(self,pose):
        box_name = self.box_name
        scene = self.scene
        box_pose = PoseStamped()
        box_pose.pose = pose
        box_pose.header.frame_id = "manip_base_link"
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.15))
        return self.wait_for_state_update(box_is_known=True, timeout=4)
    
    def makeSolidPrimitive(self, name, solid, pose):
        o = CollisionObject()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = "manip_base_link"
        o.id = name
        o.primitives.append(solid)
        o.primitive_poses.append(pose)
        o.operation = o.ADD
        return o
    
    
    def addSolidPrimitive(self, name, solid, pose):
        o = self.makeSolidPrimitive(name, solid, pose)
        self._objects[name] = o
        self._pub.publish(o)
 
    def addCylinder(self, name, height, radius, x, y, z):
        s = SolidPrimitive()
        s.dimensions = [height, radius]
        s.type = s.CYLINDER

        ps = PoseStamped()
        ps.header.frame_id = "manip_base_link"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0

        self.addSolidPrimitive(name, s, ps.pose)
    
    def attach_box(self):
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        grasping_group = "gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=4)

    def detach_box(self):
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, name=box_name)
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=4)

    def execute_pose(self, group, pose_name):
        group.set_named_target(pose_name)
        success = group.go(wait=True)
        return success

    def go_to_coordinates(self, pose):
        self.arm.set_pose_target(pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def pick(self,pose):
        self.execute_pose(self.hand,'open')
        self.clear_octomap()
        self.addCylinder(self.box_name, 0.15, 0.025, (self.coordinates.x), self.coordinates.y, self.coordinates.z)
        rospy.sleep(2)
        self.execute_pose(self.head, "up")
        pose.position.x -= 0.13
        target_pose = copy.deepcopy(pose)
        self.arm.set_pose_target(target_pose)
        success = self.arm.go(wait=True)
        if success:
            self.attach_box()
            self.execute_pose(self.hand,'close')
            self.execute_pose(self.arm,'attack')
            self.execute_pose(self.arm,'hold')
        return success
    
    def place(self,pose):
        self.execute_pose(self.arm,'hold')
        pose.position.x -= 0.15
        pose.position.z += 0.1
        rospy.sleep(2)
        self.clear_octomap()
        target_pose = copy.deepcopy(pose)
        self.arm.set_pose_target(target_pose)
        success = self.arm.go(wait=True)
        if success:
            self.detach_box()
            self.remove_box()
            self.execute_pose(self.hand,'open')
            self.execute_pose(self.arm,'home')
        return success


    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=4)

    
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('manipulator', log_level=rospy.INFO)
    Manipulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
