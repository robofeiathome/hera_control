#!/usr/bin/env python3

import sys
import copy
import rospy
import tf
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from hera_control.srv import Manip_service, Joint_service, Furniture, Look_for_person
from std_srvs.srv import Empty as Empty_srv
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane
from hera_face.srv import face_list
import math


def law_cosines(a, angle, c):
    return math.sqrt(a**2 + c**2 - 2*a*c*math.cos(angle))


def law_sines(a, b, c):
    return math.asin(a * math.sin(b) / c)


def sine_to_rad(sine):
    if sine < -1 or sine > 1:
        raise ValueError("O valor do seno deve estar entre -1 e 1.")

    theta1 = math.asin(sine)
    theta2 = math.pi - theta1
    menor_angulo = min(abs(theta1), abs(theta2))

    return menor_angulo

class Manipulator:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.arm.set_pose_reference_frame('manip_base_link')
        self.hand = moveit_commander.MoveGroupCommander('gripper')
        self.hand.set_max_acceleration_scaling_factor(1.0)
        self.hand.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)
        self.head = moveit_commander.MoveGroupCommander("camera")
        self.head.set_max_acceleration_scaling_factor(1.0)
        self.head.set_max_velocity_scaling_factor(1.0)

        self._objects = dict()


        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty_srv)
        self.recog_face = rospy.ServiceProxy('/face_recog', face_list)

        self.display_trajectory_publisher = rospy.Publisher("/move_group_arm/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)  
        self._pub = rospy.Publisher('collision_object',CollisionObject,queue_size=10)

        rospy.Subscriber('/joint_states', JointState, self.callback_positions)

        rospy.Service('manipulator', Manip_service, self.handler)
        rospy.Service('joint_command', Joint_service, self.joints_handler)
        rospy.Service('adding_furniture', Furniture, self.adding_furniture)
        rospy.Service('look_for_person',Look_for_person,self.look_handler)
        self.tf = tf.TransformListener()
        self.tf.waitForTransform('manip_base_link', 'torso', rospy.Time(), rospy.Duration(1.0))

        self.execute_pose(self.head,'up')
        self.execute_pose(self.arm,'home')
        self.execute_pose(self.hand,'open')
 

        self.box_name = "box"
        self.eef_link = self.arm.get_end_effector_link()

    def callback_positions(self, msg):
        self.joint_positions = msg.position

    def handler(self, request):
        function_name = request.type.lower()
        self.coordinates = request.goal

        pose = Pose(position=Point(self.coordinates.x, self.coordinates.y, self.coordinates.z), orientation=Quaternion(0.0,0.0,0.0,1.0))


        functions = {
            function_name: lambda pose=None: self.execute_pose(self.arm,function_name),
            'open': lambda pose=None: self.execute_pose(self.hand,'open'),
            'close': lambda pose=None: self.execute_pose(self.hand,'hard_close'),
            'serve_close': lambda pose=None: self.execute_pose(self.hand,'serve_close'),
            'soft_close': lambda pose=None: self.execute_pose(self.hand,'soft_close'),
            'ground': lambda pose=None: self.execute_pose(self.head,'ground'),
            'look_for_person': lambda pose=None: self.look_for_person(function_name),
            'bottom_shelf': lambda pose=None: self.execute_pose(self.arm,'place_bottom_shelf'),
            'center_shelf': lambda pose=None: self.execute_pose(self.arm,'pick_center_shelf'),
            'add_shelfs': lambda pose: self.add_shelfs(pose.position.x),
            'head_up': lambda pose=None: self.execute_pose(self.head,'up'),
            'way_up': lambda pose=None: self.move_joint(9, 0.3),
            'center_shelf': lambda pose=None: self.execute_pose(self.head,'center_shelf'),
            'head_down': lambda pose=None: self.execute_pose(self.head,'down'),
            'head_receptionist': lambda pose=None: self.execute_pose(self.head,'head_receptionist'),
            'head_stickler': lambda pose=None: self.execute_pose(self.head,'head_stickler'),
            'way_down': lambda pose=None: self.execute_pose(self.head,'way_down'),
            'serving_right': lambda pose=None: self.serving('right'),
            'serving_cereal_right': lambda pose=None: self.serving_cereal('right'),
            'serving_cereal_left': lambda pose=None: self.serving_cereal('left'),
            'serving_left': lambda pose=None: self.serving('left'),
            'pick': lambda pose: self.pick(pose),
            'pick_bottom': lambda pose: self.pick_bottom(pose),
            'place_with_pose': lambda pose: self.place_with_pose(pose),
            'close_with_box': lambda pose=None: self.close_with_box(),
            'place': lambda pose=None: self.place('place'),
            'place_dinner_table': lambda pose=None: self.place('place_dinner_table'),
            'place_bottom_shelf': lambda pose=None: self.execute_pose(self.arm,'place_bottom'),
            '': lambda pose: self.go_to_coordinates(pose),
        }

        try:
            result = functions[function_name](pose)
            return str(result)

        except KeyError:
            rospy.logerr('Invalid function name %s' % function_name)
            return "Invalid function name: {}".format(function_name)
        
    def joints_handler(self, request):
        function_name = request.type.lower()
        id = request.goal.id
        position = request.goal.x
        
        functions = {
            '': lambda id, position: self.move_joint(id, position),
            'point_rad': lambda id,position: self.point_rad(position),
            'point_pixel': lambda id,position: self.point_pixel(position),
            'add_shelfs': lambda id, position: self.add_shelfs(position),
        }

        try:
            result = functions[function_name](id,position)
            return str(result)

        except KeyError:
            rospy.logerr('Invalid function name %s' % function_name)
            return "Invalid function name: {}".format(function_name)
        
    def look_handler(self, request):
        name = request.name
        try:
            result = self.look_for_person(name)
            return result

        except KeyError:
            rospy.logerr('Invalid function request')
            return False
        
    def adding_furniture(self, request):
        function_name = request.type
        num = request.num
        height = request.height
        self.coordinates = request.goal

        pose = Pose(position=Point(self.coordinates.x, self.coordinates.y, self.coordinates.z), orientation=Quaternion(0.0,0.0,0.0,1.0))

        functions = {
            'add_box': lambda pose=None: self.add_box_object("table", [2.0, 0.79, height], [self.coordinates.x, self.coordinates.y, self.coordinates.z, 0, 0, 0, 1], "table"),
            'add_bookcase': lambda pose: self.add_bookcase(num, height, pose),
            'remove_all_objects': lambda pose=None: self.remove_all_objects(),
            'remove_bookcase': lambda pose=None: self.remove_bookcase(num),
            'remove_table': lambda pose=None: self.remove_table(),
            'detach': lambda pose=None: self.detach_box()
        }

        try:
            result = functions[function_name](pose)
            return str(result)

        except KeyError:
            rospy.logerr('Invalid function name %s' % function_name)
            return "Invalid function name: {}".format(function_name)

    def add_box(self):
        box_name = self.box_name
        scene = self.scene
        box_pose = PoseStamped()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.2
        box_pose.header.frame_id = "wrist_pan_link"
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.15))
        return self.wait_for_state_update(box_is_known=True, timeout=4)
    
    def add_box_object(self, name, dimensions, pose, frame="bookcase"):
        p = PoseStamped()
        p.header.frame_id = frame
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]
        p.pose.orientation.x = pose[3]
        p.pose.orientation.y = pose[4]
        p.pose.orientation.z = pose[5]
        p.pose.orientation.w = pose[6]

        self.scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))
    
    def add_bookcase(self, num, height, pose):
        largura = 0.85
        espessura = 0.03
        profundidade = 0.4

        self.shelf_dimensions = [profundidade, largura, espessura]
        shelves_heights = 0.1
        for i in range(num+1):
            self.shelf_pose = [pose.position.x, pose.position.y, shelves_heights, 0, 0, 0, 1]
            self.add_box_object("shelf{}"+format(i), self.shelf_dimensions, self.shelf_pose)
            shelves_heights += (height/num)
        
        self.wall_dimensions = [profundidade, espessura, height, 0, 0, 0, 1]

        self.wall1_pose = [self.shelf_pose[0], self.shelf_pose[1] - self.shelf_dimensions[1]/2, height/2 , 0, 0, 0, 1]
        self.wall2_pose = [self.shelf_pose[0], self.shelf_pose[1] + self.shelf_dimensions[1]/2, height/2 , 0, 0, 0, 1]        
        self.add_box_object("wall1", self.wall_dimensions, self.wall1_pose)
        self.add_box_object("wall2", self.wall_dimensions, self.wall2_pose)
        return True
    
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

    def move_joint(self,id,position):
        if id == 9 or id == 10:
            id -= 9
            group = self.head
            if id == 9:
                position = 0.3 if position >= 0.3 else position
                position = -2.0 if position <= -2.0 else position
            
        elif id == 7 or id == 8:
            id-=1
            group = self.hand
        else:
            id-=1
            group = self.arm

        values = group.get_current_joint_values()
        values[id] = position
        group.set_joint_value_target(values)
        success = group.go(wait=True)
        return success
    
    def look_for_person(self, name):
        self.move_joint(10, 0.0)

        MOTOR_POSITIONS = [0.0, 0.3, -0.3]
        person_found = False

        i = 0
        while not person_found:
            self.move_joint(10, MOTOR_POSITIONS[i])
            resp = self.recog_face(name)
            rospy.loginfo("Looking for {}".format(name))
            if resp.centers:
                center = resp.centers[0]
                self.point_pixel(center)
                person_found = True
                break
            else:
                i = i + 1 if i < 2 else 0

        return person_found


    def close_with_box(self):
        self.clear_octomap()
        self.addCylinder(self.box_name, 0.18, 0.025, (self.coordinates.x), self.coordinates.y, self.coordinates.z)
        self.attach_box()
        self.execute_pose(self.hand,'close')
        return True

    def pick(self,pose):
        self.execute_pose(self.hand,'open')
        # self.execute_pose(self.head, 'down')
        self.clear_octomap()
        rospy.sleep(2)

        self.addCylinder(self.box_name, 0.17, 0.013, (self.coordinates.x + 0.02), (self.coordinates.y + 0.04), self.coordinates.z)
        pose.position.x -= 0.1
        pose.position.y += 0.02
        # rospy.sleep(2)

        target_pose = copy.deepcopy(pose)
        self.arm.set_pose_target(target_pose)
        self.execute_pose(self.head, 'way_up')
        rospy.sleep(1)
        self.clear_octomap()
        success = self.arm.go(wait=True)
        if success:
            # tirei o clear antes do attach, pois estava demorando muito
            # self.clear_octomap()
            # rospy.sleep(1)
            self.attach_box()
            self.clear_octomap()
            success2 = self.execute_pose(self.hand,'hard_close')
            # self.execute_pose(self.arm,'attack')
            return success2
        else:
            self.detach_box()
            self.remove_box()
        return success
    
    def pick_bottom(self,pose):
        self.execute_pose(self.hand,'open')
        self.execute_pose(self.head, 'way_down')
        # rospy.sleep(2)
        self.clear_octomap()

        self.addCylinder(self.box_name, 0.17, 0.0125, (self.coordinates.x), (self.coordinates.y), self.coordinates.z)
        pose.position.x -= 0.10
        pose.position.y += 0.06
        # rospy.sleep(1)
        target_pose = copy.deepcopy(pose)
        self.arm.set_pose_target(target_pose)
        self.execute_pose(self.head, 'up')
        # rospy.sleep(1)
        success = self.arm.go(wait=True)
        if success:
            self.attach_box()
            success2 = self.execute_pose(self.hand,'hard_close')
            # self.execute_pose(self.arm,'attack')
            return success2
        else:
            self.detach_box()
            self.remove_box()
        return success

    def place(self, manip_pose):
        self.clear_octomap()
        success = self.execute_pose(self.arm, manip_pose)
        # rospy.sleep(2)
        if success:
            self.detach_box()
            self.remove_box()
            self.execute_pose(self.hand,'open')
        return success
    
    def place_with_pose(self, pose):

        self.clear_octomap()
        #rospy.sleep(2)
        pose.position.x -= 0.07
        # pose.position.z = 0.13
        target_pose = copy.deepcopy(pose)
        self.arm.set_pose_target(target_pose)
        #self.execute_pose(self.head, 'head_receptionist')
        # rospy.sleep(1)
        success = self.arm.go(wait=True)
        if success:
            self.detach_box()
            self.remove_box()
            success2 = self.execute_pose(self.hand,'open')
            return success2
        else:
            self.detach_box()
            self.remove_box()
        return success

    def point_pixel(self, pixel):
        manip_cam_dist = 0.25
        point_dist = 3
        cam_x_pixel = 1280

        pixel_rad = ((-1.4/cam_x_pixel)*pixel) + 0.825
        # manip_point_dist = law_cosines(manip_cam_dist, pixel_rad, point_dist)
        # manip_point_rad = law_sines(point_dist, pixel_rad, manip_point_dist)
        # print(manip_point_rad)
        # manip_rad = sine_to_rad(manip_point_rad)
        #
        # manip_rad = manip_rad if pixel_rad > 0 else -manip_rad
        #
        # # self.execute_pose(self.hand, 'w')
        # self.execute_pose(self.arm, 'point')
        self.move_joint(10, pixel_rad)
        return True

    def point_rad(self, angle):
        self.execute_pose(self.hand, 'hard_close')
        self.execute_pose(self.arm, 'point')
        self.move_joint(1, angle)
        return True    

    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=4)

    def remove_bookcase(self, num):
        for i in range(num+1):
            self.scene.remove_world_object("shelf{}"+format(i))
            
        self.scene.remove_world_object("cabinet")    
        self.scene.remove_world_object("wall1")
        self.scene.remove_world_object("wall2")
        return True
    
    def remove_table(self):

        self.scene.remove_world_object("table")
        return True
    
    def remove_all_objects(self):
        self.scene.clear()
        return True
    
    def serving(self, side):
        self.execute_pose(self.arm, 'serve_up')
        if side == 'left':
            pre = -1
        elif side == 'right':
            pre = 1
        self.move_joint(5,0.75*pre)
        self.move_joint(5,1.2*pre)
        x = 1.2*pre
        for i in range (1):
            x += 0.3*pre
            self.move_joint(5,x)
            rospy.sleep(0.5)
        for i in range (1):
            x -= 0.3*pre
            self.move_joint(5,x)
            rospy.sleep(0.5)
        self.execute_pose(self.arm, 'serve_up')
        self.execute_pose(self.arm, 'attack')
        return
    
    def serving_cereal(self, side):
        self.execute_pose(self.arm, 'serve_up')
        if side == 'left':
            pre = -1
        elif side == 'right':
            pre = 1
        self.move_joint(5,0.75*pre)
        self.move_joint(5,1.2*pre)
        x = 1.2*pre
        for i in range (4):
            x += 0.3*pre
            self.move_joint(5,x)
            rospy.sleep(0.5)
        x -= 0.3*pre
        self.move_joint(5,x)
        rospy.sleep(0.5)
        self.execute_pose(self.arm, 'serve_up')
        self.execute_pose(self.arm, 'attack')
        return
    
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
    rospy.init_node('manipulator', log_level=rospy.ERROR)
    Manipulator()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
