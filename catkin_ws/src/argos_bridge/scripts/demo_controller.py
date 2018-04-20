#!/usr/bin/env python

"""
Demo controller for controlling an ARGoS simulated robot via argos_bridge.
The algorithm implemented here is a simple state machine designed to push
pucks to the walls.  If the proximity sensor detects an obstacle (other robot
or wall) then it moves away from it for a fixed period.
"""
import rospy
from argos_bridge.msg import State
from argos_bridge.msg import Haptic
from argos_bridge.msg import Flocking

class DemoController:

    statePub = None

    def __init__(self):
        self.statePub = rospy.Publisher('State', State, queue_size=1)
        rospy.Subscriber('Haptic', Haptic, self.haptic_callback)
        # rospy.Subscriber('Flocking', Flocking, self.flocking_callback)

    def haptic_callback(self, haptic):
        print "haptic call back python"
    
    # def flocking_callback(self, flocking_msg):
    #     print "Flocking call back python"
        

if __name__ == '__main__':
    rospy.init_node("demo_controller")
    controller = DemoController()
    rospy.spin()
