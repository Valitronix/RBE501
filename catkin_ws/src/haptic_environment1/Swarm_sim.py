import sys
import pygame
from pygame.locals import *
import math
import helper
import numpy as np
from numpy import sin, cos, pi
from operator import sub
import time
import rospy
from sensor_msgs.msg import JointState
from geomagic_control.msg import OmniFeedback, PhantomButtonEvent
from argos_bridge.msg import Haptic, State, Flocking
import tf
import signal

class sim_swarm:
    def __init__(self, num_bots):
        self.swarm_state = np.asarray([[0.] * num_bots,[0.] * num_bots,[0] * num_bots,[0] * num_bots,[0] * num_bots,[0] * num_bots])
        self.swarm_heading = [0] * num_bots
        rospy.init_node("swarm_simulator")

        self.bot_0_pub = rospy.Publisher("/bot0/state", State, queue_size=1)
        self.bot_1_pub = rospy.Publisher("/bot1/state", State, queue_size=1)
        self.bot_2_pub = rospy.Publisher("/bot2/state", State, queue_size=1)
        self.bot_3_pub = rospy.Publisher("/bot3/state", State, queue_size=1)

    def publish(self):
        bot_state = State()
        bot_state.x = self.swarm_state[0,0]
        bot_state.y = self.swarm_state[1,0]
        #bot_state.yaw = 0.0
        self.bot_0_pub.publish(bot_state)

        bot_state.x = self.swarm_state[0, 1]
        bot_state.y = self.swarm_state[1, 1]
        self.bot_1_pub.publish(bot_state)

        bot_state.x = self.swarm_state[0, 2]
        bot_state.y = self.swarm_state[1, 2]
        self.bot_2_pub.publish(bot_state)

        bot_state.x = self.swarm_state[0, 3]
        bot_state.y = self.swarm_state[1, 3]
        self.bot_3_pub.publish(bot_state)


    def shiftleft(self):
        num_bots = self.swarm_state.shape[1]
        shift = np.matrix([[-.1]*num_bots,[0]*num_bots,[0]*num_bots,[0]*num_bots,[0]*num_bots,[0]*num_bots])
        self.swarm_state += shift
        self.swarm_state[0,:] = np.clip(self.swarm_state[0,:], 0, helper.windowWidth)

    def shiftright(self):
        num_bots = self.swarm_state.shape[1]
        shift = np.matrix(
            [[.1] * num_bots, [0] * num_bots, [0] * num_bots, [0] * num_bots, [0] * num_bots, [0] * num_bots])
        self.swarm_state += shift
        self.swarm_state[0,:] = np.clip(self.swarm_state[0, :], 0, helper.windowWidth)

    def shiftup(self):
        num_bots = self.swarm_state.shape[1]
        shift = np.matrix([[0]*num_bots,[-.1]*num_bots,[0]*num_bots,[0]*num_bots,[0]*num_bots,[0]*num_bots])
        self.swarm_state += shift
        self.swarm_state[1,:] = np.clip(self.swarm_state[1, :], 0, helper.windowHeight)

    def shiftdown(self):
        num_bots = self.swarm_state.shape[1]
        shift = np.matrix([[0]*num_bots,[.1]*num_bots,[0]*num_bots,[0]*num_bots,[0]*num_bots,[0]*num_bots])
        self.swarm_state += shift
        self.swarm_state[1,:] = np.clip(self.swarm_state[1, :], 0, helper.windowHeight)

if __name__=="__main__":
    our_sim_swarm = sim_swarm(4)
    while not rospy.is_shutdown():
        selection = np.random.choice(4)
        if selection == 0:
            our_sim_swarm.shiftdown()
        if selection == 1:
            our_sim_swarm.shiftleft()
        if selection == 2:
            our_sim_swarm.shiftright()
        if selection == 3:
            our_sim_swarm.shiftup()
        our_sim_swarm.publish()
        rospy.sleep(1.)
