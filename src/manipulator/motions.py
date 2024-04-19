#!/usr/bin/env python3
from hera_control.srv import Manip_service

    class Motions:
        def __init__(self, manipulator):
            self.manipulator = manipulator
            rospy.Service('manipulator', Manip_service, self.handler)

        def handler(self, request):
                function_name = request.type.lower()
                self.coordinates = request.goal

                pose = Pose(position=Point(self.coordinates.x, self.coordinates.y, self.coordinates.z), orientation=Quaternion(0.0,0.0,0.0,1.0))

                functions = {
                    function_name: lambda pose=None: self.execute_pose(self.arm,function_name),
                    'serving_right': lambda pose=None: self.serving('right'),
                    'serving_cereal_right': lambda pose=None: self.serving_cereal('right'),
                    'serving_cereal_left': lambda pose=None: self.serving_cereal('left'),
                    'serving_left': lambda pose=None: self.serving('left'),
                    'pick_hard_close': lambda pose: self.pick(pose,'hard_close'),
                    'pick_soft_close': lambda pose: self.pick(pose,'soft_close'),
                    'pick_super_soft_close': lambda pose: self.pick(pose,'super_soft_close'),
                    'place': lambda pose=None: self.place('place'),
                    'place_kitchen_table': lambda pose=None: self.place('place_kitchen_table'),
                    'place_counter': lambda pose=None: self.place('place_counter'),
                    'place_dinner_table': lambda pose=None: self.place('place_dinner_table'),
                    'place_livingroom_table': lambda pose=None: self.place('place_livingroom_table'),
                    'place_bottom_shelf': lambda pose=None: self.place('place_bottom_shelf'),
                    '': lambda pose: self.go_to_coordinates(pose),
                }

                try:
                    result = functions[function_name](pose)
                    return str(result)

                except KeyError:
                    rospy.logerr('Invalid function name %s' % function_name)
                    return "Invalid function name: {}".format(function_name)

        def go_to_coordinates(self, pose):
            self.arm.set_pose_target(pose)
            success = self.arm.go(wait=True)
            self.arm.stop()
            self.arm.clear_pose_targets()
            return success

        def pick(self,pose,hand_pose):
            self.execute_pose(self.hand,'open')
            self.clear_octomap()
            self.addCylinder(self.box_name, 0.18, 0.025, (self.coordinates.x), self.coordinates.y, self.coordinates.z)
            rospy.sleep(2)
            self.execute_pose(self.head, 'down')
            pose.position.x -= 0.10
            target_pose = copy.deepcopy(pose)
            self.arm.set_pose_target(target_pose)
            self.execute_pose(self.head, 'up')
            rospy.sleep(1)
            success = self.arm.go(wait=True)
            if success:
                self.attach_box()
                success2 = self.execute_pose(self.hand, hand_pose)
                # self.execute_pose(self.arm,'attack')
                return success2
            return success

        def place(self, manip_pose):
            self.clear_octomap()
            success = self.execute_pose(self.arm, manip_pose)
            rospy.sleep(2)
            if success:
                self.detach_box()
                self.remove_box()
                self.execute_pose(self.hand,'half_open')
                rospy.sleep(1)
                self.execute_pose(self.hand,'open')

            return success
        
        def point_pixel(self, pixel):
            self.execute_pose(self.hand, 'close')
            self.execute_pose(self.arm, 'point')
            x = ((-1.55/1920)*pixel) + 0.775
            self.move_joint(1, x)
            return True

        def point_rad(self,angle):
            self.execute_pose(self.hand, 'close')
            self.execute_pose(self.arm, 'point')
            self.move_joint(1, angle)
            return True    
        
        def serving(self, side):
            self.execute_pose(self.arm, 'serve')
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
            self.execute_pose(self.arm, 'serve')
            self.execute_pose(self.arm, 'attack')
            return
        
        def serving_cereal(self, side):
            self.execute_pose(self.arm, 'serve')
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
            self.execute_pose(self.arm, 'serve')
            self.execute_pose(self.arm, 'attack')
            return
    
    class Objects:
        def __init__(self, manipulator):
            
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
    rospy.init_node('motions', log_level=rospy.ERROR)
    manipulator = Manipulator()
    Motions(manipulator)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass