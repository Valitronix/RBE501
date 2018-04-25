#!/usr/bin/env python

import sys
import pygame
from pygame.locals import *
import math
import helper
import numpy as np
from numpy import sin, cos
from operator import sub
import time
import rospy
from sensor_msgs.msg import JointState
from geomagic_control.msg import OmniFeedback, PhantomButtonEvent
from argos_bridge.msg import Haptic
from argos_bridge.msg import State
import tf
import signal


class Game:

    def __init__(self):

        # Controller Settings
        self.action_boundary = 100
        self.gains = {'K_input': 1,
                      'V_input': 0,
                      'K_output': 0.05,
                      }
        # PyGame Initialization
        self.player = Rect((helper.windowWidth*0.5, helper.windowHeight*0.5, helper.PLAYERSIZE_X, helper.PLAYERSIZE_Y) )
        self.swarmbot = Rect((helper.windowWidth*0.5, helper.windowHeight*0.5, helper.BLOCKSIZE_X, helper.BLOCKSIZE_Y) )
        self.display_surf = pygame.display.set_mode((helper.windowWidth, helper.windowHeight))
        pygame.display.set_caption('Use the haptic device to pull the swarm bot')

        pygame.init()
        pygame.font.init()


        # Variable Initialization
        self.ee_state = np.asarray([[0], [0], [0], [0], [0], [0]])
        self.swarm_state = np.asarray([[300],[300],[0],[0],[0],[0]])
        self.swarm_heading = 0
        self.F = [0,0,0]
        self.haptic_output = OmniFeedback()
        self.swarm_output = Haptic()
        self.time0 = time.time()

        # ROS Initialization
        rospy.init_node("python_controller")
        rospy.Subscriber("/bot0/state", State, self.update_from_bot)
        rospy.Subscriber("/Geomagic/end_effector_pose", JointState, self.get_input_haptic)
        rospy.Subscriber("/Geomagic/button", PhantomButtonEvent, self.button_callback)
        self.one_bot_pub = rospy.Publisher("/bot0/haptic", Haptic, queue_size=1)
        self.haptic_pub = rospy.Publisher("/Geomagic/force_feedback", OmniFeedback, queue_size=1)



    def get_input_mouse(self):
        '''
        Gets input from mouse. To be used when haptic device input is not available.
        '''
        pygame.event.pump()
        (in_x, in_y) = pygame.mouse.get_pos()
        self.ee_state = [[in_x], [in_y], [0], [0], [0], [0]]
        self.player.center = (self.ee_state[0][0], self.ee_state[1][0])

    def get_input_haptic(self, js):
        '''
        ROS callback to update current position of haptic device. Do not put print
        statements in this callback, as performance will become unacceptably slow.
        :param js: current end-effector position of the haptic device in the haptic
        device's base frame
        '''
        if not rospy.is_shutdown():
            self.ee_state[0] = self.remap(js.position[0], -150, 150, 0, helper.windowWidth)
            self.ee_state[1] = self.remap(js.position[1], 90, -90, 0, helper.windowHeight)
            self.ee_state[2] = 0

    def button_callback(self, button):
        '''
        ROS callback for using buttons on haptic device for input.
        :param button: information from Geomagic Touch describing the new state of the
        buttons (True or False
        '''
        pass


    def remap(self, x, in_min, in_max, out_min, out_max):
        # Simple function for linear scaling
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def update_player(self):
        '''
        Simple physics engine for when the swarm is not running. Take calculated force vectors
        (which would otherwise be sent via ROS topics to either the ARGoS simulation or the
        real swarm) and applies them to a simple, arbitrarily-massed swarm simulation.
        :return:
        '''
        swarm_mass = 5
        dt = time.clock() - self.time0
        xdd = np.array(self.F).reshape(3, 1) / swarm_mass
        B = np.zeros(shape=(6, 3))
        A = np.identity(6)

        B[3, 0] = dt
        B[4, 1] = dt
        B[5, 2] = dt

        A[0, 3] = dt
        A[1, 4] = dt
        A[2, 5] = dt

        self.swarm_state = np.dot(A, self.swarm_state) + np.dot(B, xdd)

        self.swarmbot.center = (self.swarm_state[0][0], self.swarm_state[1][0])
        self.time0 = time.clock()

    def update_from_bot(self, state):
        '''
        ROS callback to update current position of swarm robot(s). Do not
        put print statements in this function, as performance will become
        unacceptably slow.
        :param state: current position of the swarm robot in the swarm world frame
        '''
        x_virtual = self.remap(state.x, -2.5, 2.5, 0, helper.windowWidth)
        x_dot_virtual = self.remap(state.dot_x, -2.5, 2.5, 0, helper.windowWidth)
        y_virtual = self.remap(state.y, 2.5, -2.5, 0, helper.windowHeight)
        y_dot_virtual = self.remap(state.dot_y, 2.5, -2.5, 0, helper.windowHeight)
        self.swarm_state = np.asarray([[x_virtual], [y_virtual], [0], [x_dot_virtual], [y_dot_virtual], [0]])
        self.player.center = (self.ee_state[0][0], self.ee_state[1][0])
        self.swarmbot.center = (self.swarm_state[0][0], self.swarm_state[1][0])
        self.swarm_heading = -state.yaw

    def update_force_pull(self):
        """
        Given the current haptic device and swarm position(s), determine the forces
        or commands to be sent to the haptic feedback and swarm.
        :return:
        """
        e = np.subtract(self.ee_state[0:3], self.swarm_state[0:3])
        ed = np.subtract(self.ee_state[3:], self.swarm_state[3:])
        d = np.linalg.norm(e)
        if d < self.action_boundary:
            F = self.gains['K_input'] * e + self.gains['V_input'] * ed
        else:
            F = [0,0,0]
        self.F = np.round(F, 2)
        self.swarm_force(np.asarray(self.F))
        self.haptic_force(self.F)

    def swarm_force(self, F):
        R_mat = np.matrix([[cos(self.swarm_heading), sin(self.swarm_heading), 0],
                           [-sin(self.swarm_heading), cos(self.swarm_heading), 0],
                           [                       0,                       0, 1]])

        force_vector = R_mat*F.reshape(3,1)
        output_force = Haptic()
        output_force.x_value = force_vector[0,0]
        output_force.y_value = -force_vector[1, 0]
        self.one_bot_pub.publish(output_force)

    def haptic_force(self, F):
        self.haptic_output.force.x = max(min(-F[0] * self.gains['K_output'], 3), -3)
        self.haptic_output.force.y = max(min(F[1] * self.gains['K_output'], 3), -3)
        self.haptic_output.force.z = 0
        self.haptic_output.lock = [False, False, False]
        self.haptic_pub.publish(self.haptic_output)

    def update_GUI(self):
        self.display_surf.fill((0, 0, 0))
        self.player_draw()

        # Add text to screen
        text = "Current Force... X = %.2d, Y = %.2d, Z = %.2d" %(self.F[0], self.F[1], self.F[2])
        myfont = pygame.font.SysFont('Comic Sans MS', 18)
        textsurface = myfont.render(text, False, helper.WHITE)
        self.display_surf.blit(textsurface, (0,0))

        # Need to update to see changes
        pygame.display.update()


    def player_draw(self):
        """
        draws the player location
        :return:
        """
        pygame.draw.rect(self.display_surf, helper.WHITE, self.player, 0)
        pygame.draw.ellipse(self.display_surf, helper.DARK_BLUE, self.swarmbot, 0)

if __name__ == "__main__":
    game = Game()
    while not rospy.is_shutdown():
        #game.get_input() # Use this line if input is coming from mouse
        game.update_force_pull()
        #game.update_player() # Use this line if the swarm simulation is not running
        game.update_GUI()
