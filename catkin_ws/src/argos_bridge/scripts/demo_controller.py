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

class DemoController:

    statePub = None
    xForce = None
    yForce = None
    zForce = None

    def __init__(self):
        self.statePub = rospy.Publisher('State', State, queue_size=1)
        rospy.Subscriber('Haptic', Haptic, self.haptic_callback)

    def haptic_callback(self, haptic):
        self.xForce = haptic.x_value
        self.yForce = haptic.y_value
        self.zForce = haptic.z_value

if __name__ == '__main__':
    rospy.init_node("demo_controller")
    controller = DemoController()
    rospy.spin()
