#!/usr/bin/env python3
import sys
import rospy

class Raposo:

    def __init__(self):
        self.objects = rospy.ServiceProxy('/objects', FindObject)
        rospy.Service('RaposoNaoPegue', Raposo_service, self.object_selection)

        self.allObjects = [
                            ('English_Sauce', 4), 
                            ('Tonic', 0),
                            ('Coke', 5), 
                            ('Potato_Chips', 5),
                            ('Tomato_Sauce', 1), 
                            ('Mustard', 5)]


    def object_selection(self):
        resp = self.objects("all")
        coordinates = resp.position
        names = resp.taken_object
        for i in names:
            if names[i] == allObjects[i][0] and allObjects[i][1] == 5:
                ObjectsToPickCoordinates.append(coordinates[i])
                ObjectsToPickName.append(names[i])
            if len(ObjectsToPickCoordinates) == 3:
                for i in ObjectsToPickCoordinates:
                    print(ObjectsToPickName[i])
                return ObjectsToPickCoordinates,ObjectsToPickName

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('raposo', log_level=rospy.INFO)
    Raposo()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
