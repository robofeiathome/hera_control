#!/usr/bin/env python3
from hera_control.srv import Furniture

class Furniture:
    def __init__(self, manipulator):
        self.manipulator = manipulator
        rospy.Service('adding_furniture', Furniture, self.adding_furniture)""
    
    def handler(self, request):
        function_name = request.type
        num = request.num
        height = request.height
        self.coordinates = request.goal

        pose = Pose(position=Point(self.coordinates.x, self.coordinates.y, self.coordinates.z), orientation=Quaternion(0.0,0.0,0.0,1.0))

        functions = {
            'add_bookcase': lambda pose: self.add_bookcase(num, height, pose),
            'remove_all_objects': lambda pose=None: self.remove_all_objects(),
            'remove_bookcase': lambda pose=None: self.remove_bookcase(num)
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
    
    def add_box_object(self, name, dimensions, pose):
        p = PoseStamped()
        p.header.frame_id = "bookcase"
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
        largura = 0.93
        espessura = 0.03
        profundidade = 0.27

        self.shelf_dimensions = [profundidade, largura, espessura]
        shelves_heights = 0
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
    
    def remove_bookcase(self, num):
        for i in range(num+1):
            self.scene.remove_world_object("shelf{}"+format(i))
        self.scene.remove_world_object("wall1")
        self.scene.remove_world_object("wall2")
        return True
    
    def remove_all_objects(self):
        self.scene.clear()
        return True
    
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motions', log_level=rospy.ERROR)
    manipulator = Manipulator()
    Furniture(manipulator)
    Objects(manipulator)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass