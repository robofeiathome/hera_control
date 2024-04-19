#!/usr/bin/env python3
from hera_control.srv import Manip_service

    class Poses:
        def __init__(self, manipulator):
            self.manipulator = manipulator
            rospy.Service('manipulator', Manip_service, self.handler)

        def handler(self, request):
                function_name = request.type.lower()
                self.coordinates = request.goal

                pose = Pose(position=Point(self.coordinates.x, self.coordinates.y, self.coordinates.z), orientation=Quaternion(0.0,0.0,0.0,1.0))

                functions = {
                    function_name: lambda pose=None: self.execute_pose(self.arm,function_name),
                    'open': lambda pose=None: self.execute_pose(self.hand,'open'),
                    'half_open': lambda pose=None: self.execute_pose(self.hand,'half_open'),
                    'soft_close': lambda pose=None: self.execute_pose(self.hand,'soft_close'),
                    'hard_close': lambda pose=None: self.execute_pose(self.hand,'hard_close'),
                    'super_soft_close': lambda pose=None: self.execute_pose(self.hand,'super_soft_close'),
                    'ground': lambda pose=None: self.execute_pose(self.head,'ground'),
                    'bottom_shelf': lambda pose=None: self.execute_pose(self.arm,'place_bottom_shelf'),
                    'center_shelf': lambda pose=None: self.execute_pose(self.arm,'pick_center_shelf'),
                    'add_shelfs': lambda pose: self.add_shelfs(pose.position.x),
                    'head_up': lambda pose=None: self.execute_pose(self.head,'up'),
                    'center_shelf': lambda pose=None: self.execute_pose(self.head,'center_shelf'),
                    'head_down': lambda pose=None: self.execute_pose(self.head,'down'),
                    'way_down': lambda pose=None: self.execute_pose(self.head,'way_down'),
                    'serving_right': lambda pose=None: self.serving('right'),
                    'serving_cereal_right': lambda pose=None: self.serving_cereal('right'),
                    'serving_cereal_left': lambda pose=None: self.serving_cereal('left'),
                    'serving_left': lambda pose=None: self.serving('left'),
                    'pick_hard_close': lambda pose: self.pick(pose,'hard_close'),
                    'pick_soft_close': lambda pose: self.pick(pose,'soft_close'),
                    'pick_super_soft_close': lambda pose: self.pick(pose,'super_soft_close'),
                    'close_with_box': lambda pose=None: self.close_with_box(),
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

        def execute_pose(self, group, pose_name):
            group.set_named_target(pose_name)
            success = group.go(wait=True)
            return success
        
if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('poses', log_level=rospy.ERROR)
    manipulator = Manipulator()
    Poses(manipulator)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass