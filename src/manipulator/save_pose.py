#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import rospkg
import sys


class Poses():
    def __init__(self):
        self.group = str(input("Group:"))
        self.pose_name = str(input("Pose_name:"))

        path = rospkg.RosPack().get_path('hera_moveit_config')
        self.file_path = (path + '/config/hera.srdf')
        
        rospy.Subscriber('/joint_states', JointState, self.callback_positions)

        # Initialize self.pose_message
        self.pose_message = ""
        self.file_written = False

    def writing_file(self):
        # Open the original file for reading
        with open(self.file_path, 'r') as original_file:
            # Read the content of the original file
            original_content = original_file.read()

        # Specify the position where you want to insert new content
        position = original_content.find("<group_state")  # Replace with your desired string

        if position != -1:  # Check if the string is found in the file
            # Create a new file for writing
            with open('new_file.txt', 'w') as new_file:
                # Write the original content up to the insertion point
                new_file.write(original_content[:position])

                # Write the new content
                new_file.write(self.pose_message)

                # Write the remaining original content after the insertion point
                new_file.write(original_content[position:])

            # Replace the original file with the new file
            import shutil
            shutil.move('new_file.txt', self.file_path)
            print('Success')
            self.file_written = True

        else:
            print("String not found in the file")


        # The file will be automatically closed when the 'with' block exits
        new_file.close()

    def callback_positions(self, msg):
        if self.group == 'arm':
            self.pose_message = '<group_state name="'+ self.pose_name +'" group="'+ self.group +'">\n\
        <joint name="elbow_lift_joint" value="'+ str(msg.position[0]) +'"/>\n\
        <joint name="elbow_pan_joint" value="'+ str(msg.position[1]) +'"/>\n\
        <joint name="shoulder_lift_joint" value="'+ str(msg.position[6]) +'"/>\n\
        <joint name="shoulder_pan_joint" value="'+ str(msg.position[7]) +'"/>\n\
        <joint name="wrist_lift_joint" value="'+ str(msg.position[8]) +'"/>\n\
        <joint name="wrist_pan_joint" value="'+ str(msg.position[9]) +'"/>\n\
    </group_state>\n    '
        elif self.group == 'gripper':
            self.pose_message = '<group_state name="'+ self.pose_name +'" group="'+ self.group +'">\n\
        <joint name="gripper_left_joint" value="'+ str(msg.position[2]) +'"/>\n\
        <joint name="gripper_right_joint" value="'+ str(msg.position[3]) +'"/>\n\
    </group_state>\n    '
        elif self.group == 'camera':
            self.pose_message = '<group_state name="'+ self.pose_name +'" group="'+ self.group +'">\n\
        <joint name="joint_camera_tilt" value="'+ str(msg.position[5]) +'"/>\n\
        <joint name="joint_camera_pan" value="'+ str(msg.position[4]) +'"/>\n\
    </group_state>\n    '
        else:
            print("Invalid group, please try again. The existing groups are:\narm\ngripper\ncamera")
        if not self.file_written: self.writing_file()

if __name__ == "__main__":
    rospy.init_node('save_poses')
    p = Poses()
    rospy.spin()