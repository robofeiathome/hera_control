#!/usr/bin/env python3
from hera_control.srv import Joint_service

class Joints:
    def __init__(self, manipulator):
        self.manipulator = manipulator
        rospy.Service('joint_command', Joint_service, self.joints_handler)

    def handler(self, request):
        function_name = request.type.lower()
        id = request.goal.id
        position = request.goal.x

        functions = {
            '': lambda id, position: self.move_joint(id, position),
            'point_rad': lambda id,position: self.point_rad(position),
            'point_pixel': lambda id,position: self.point_pixel(position),
        }

        try:
            result = functions[function_name](id,position)
            return str(result)

        except KeyError:
            rospy.logerr('Invalid function name %s' % function_name)
            return "Invalid function name: {}".format(function_name)

    def move_joint(self,id,position):
        if id == 9:
            id = 0
            group = self.head
            position = 0 if position >= 0.0 else position
            position = -1.5 if position <= -1.5 else position
            
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

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motions', log_level=rospy.ERROR)
    manipulator = Manipulator()
    Joints(manipulator)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass  