#!/usr/bin/env python3
import sys
import rospy
from hera_control.srv import Raposo_service
from hera_objects.srv import FindObject

class Raposo:

    def __init__(self):
        self.objects = rospy.ServiceProxy('/objects', FindObject)
        rospy.Service('RaposoNaoPegue', Raposo_service, self.object_selection)

        self.allObjects = [
                            ('English_Sauce0', 4), 
                            ('Tonic0', 0),
                            ('Coke0', 5), 
                            ('Potato_Chips0', 5),
                            ('Tomato_Sauce0', 1), 
                            ('Mustard0', 5)]


    def object_selection(self,req):
        resp = self.objects("all")
        coordinates = resp.position
        names = resp.taken_object
        self.ObjectsToPickCoordinates = []
        self.ObjectsToPickName = []
        for i in range (len(names)):
            for j in range (len(self.allObjects)):
                if names[i] == self.allObjects[j][0] and self.allObjects[j][1] == 5:
                    self.ObjectsToPickCoordinates.append(coordinates[i])
                    self.ObjectsToPickName.append(names[i])
            if len(self.ObjectsToPickCoordinates) == 3:
                return self.ObjectsToPickCoordinates,self.ObjectsToPickName

if __name__ == '__main__':
    rospy.init_node('raposo', log_level=rospy.INFO)
    Raposo()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
