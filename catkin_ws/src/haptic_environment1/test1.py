#!/usr/bin/env python

import sys
import rospy
from argos_bridge.msg import State
def update_from_bot(state):
    prin5 state

if __name__ == "__main__":
    rospy.Subscriber("/bot0/state", State, update_from_bot)
    rospy.spin()
