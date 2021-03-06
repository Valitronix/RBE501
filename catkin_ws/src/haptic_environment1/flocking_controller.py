#!/usr/bin/env python

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


class Game:

    def __init__(self, num_bots):
        #Controller Settings
        self.action_boundary = 300
        self.gains = {'K_input': 0.5,
                      'V_input': 0,
                      'K_z_feedback': 1.5,
                      'V_z_feedback': 0.5,
                      'K_output': 0.05,
                      }
        self.tdmin = 10
        self.tdmax = 100
        self.target_distance = 20

        #PyGame Initialization
        self.player = Rect((helper.windowWidth*0.5, helper.windowHeight*0.5, helper.PLAYERSIZE_X, helper.PLAYERSIZE_Y) )
        self.swarmbot = Rect((helper.windowWidth*0.5, helper.windowHeight*0.5, helper.BLOCKSIZE_X*0.5, helper.BLOCKSIZE_Y*0.5) )
        self.display_surf = pygame.display.set_mode((helper.windowWidth, helper.windowHeight))
        pygame.display.set_caption('Use the haptic device to move the swarm')
        pygame.init()
        pygame.font.init()

        #Variable Initialization
        self.ee_state = np.asarray([[0], [0], [0], [0], [0], [0]])
        self.swarm_state = np.matrix([[300] * num_bots,[300] * num_bots,[0] * num_bots,[0] * num_bots,[0] * num_bots,[0] * num_bots])
        self.swarm_center_state = np.asarray([[300],[300],[0],[0],[0],[0]])
        self.swarm_heading = [0] * num_bots
        self.F = [0,0,0]
        self.flockingx = 0
        self.flockingy = 0
        self.time0 = time.time()
        self.swarm_time = [time.time()] * num_bots

        # ROS Initialization
        rospy.init_node("python_controller")

        for i in range(10):
            topic_string = "/bot"+str(i)+"/State"
            rospy.Subscriber(topic_string, State, self.update_from_bot, (i))

        rospy.Subscriber("/Geomagic/end_effector_pose", JointState, self.get_input_haptic)
        rospy.Subscriber("/Geomagic/button", PhantomButtonEvent, self.button_callback)

        self.haptic_pub = rospy.Publisher("/Geomagic/force_feedback", OmniFeedback, queue_size=1)
        self.flock_pub = rospy.Publisher("/Flocking", Flocking, queue_size=1)
        self.multi_bot_pub = rospy.Publisher("/Haptic", Haptic, queue_size=1)




    def get_input_mouse(self):
        '''
        Gets input from mouse. To be used when haptic device input is not available.
        '''
        pygame.event.pump()
        (in_x, in_y) = pygame.mouse.get_pos()
        self.ee_state = [[in_x], [in_y], [0], [0], [0], [0]]
        self.player.center = (in_x, in_y)

    def get_input_haptic(self, js):
        '''
        ROS callback to update current position of haptic device. Do not put print
        statements in this callback, as performance will become unacceptably slow.
        :param js: current end-effector position of the haptic device in the haptic
        device's base frame
        '''
        if not rospy.is_shutdown():
            dt = max(time.time() - self.time0, 0.1)
            self.time0 = time.time()

            # Velocity and position in X axis
            temp_0 = self.remap(js.position[0], -150, 150, 0, helper.windowWidth)
            self.ee_state[3] = (temp_0 - self.ee_state[0])/dt
            self.ee_state[0] = temp_0

            # Velocity and position and Y axis
            temp_1 = self.remap(js.position[1], 90, -90, 0, helper.windowHeight)
            self.ee_state[4] = (temp_1 - self.ee_state[1]) / dt
            self.ee_state[1] = temp_1

            # Velocity and position in Z axis
            temp_2 = self.remap(js.position[2], -50, 50, self.tdmin, self.tdmax)
            self.ee_state[2] = min(self.tdmax, max(self.tdmin, temp_2))


    def button_callback(self, button):
        '''
        ROS callback for using buttons on haptic device for input.
        :param button: information from Geomagic Touch describing the new state of the
        buttons (True or False
        '''
        # Define step size in target distance
        step = 10
        # Grey button increments up by step, white button decrements down by step
        if button.grey_button:
            self.target_distance = min(100, self.target_distance + step)
        if button.white_button:
            self.target_distance = max(10, self.target_distance - step)


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
        dt = time.clock() - self.timedt
        xdd = np.array(self.F).reshape(3, 1) / swarm_mass
        B = np.zeros(shape=(6, 3))
        A = np.identity(6)

        B[3, 0] = dt
        B[4, 1] = dt
        B[5, 2] = dt

        A[0, 3] = dt
        A[1, 4] = dt
        A[2, 5] = dt

        self.swarm_center_state = np.dot(A, self.swarm_center_state) + np.dot(B, xdd)
        self.time0 = time.clock()

    def update_from_bot(self, state, bot_no):
        '''
        ROS callback to update current position of swarm robot(s). Do not
        put print statements in this function, as performance will become
        unacceptably slow.
        :param state: current position of the swarm robot in the swarm world frame
        :param bot_no: the ID of the bot being updated
        '''
        dt = time.time() - self.swarm_time[bot_no]

        # Remap positions and velocities to simulation frame
        x_virtual = self.remap(state.x, -15, 15, 0, helper.windowWidth)
        y_virtual = self.remap(state.y, 15, -15, 0, helper.windowHeight)
        x_dot_virtual = (x_virtual - self.swarm_state[0,bot_no])/dt
        y_dot_virtual = (y_virtual - self.swarm_state[1,bot_no])/dt

        # Update state variables
        state_temp = np.zeros((6,1))
        state_temp[0] = x_virtual
        state_temp[1] = y_virtual
        self.swarm_state[:, bot_no] = state_temp.reshape(6,1)
        self.swarm_heading[bot_no] = -state.yaw

        # Recalculate swarm center
        self.swarm_center_state = np.mean(self.swarm_state, 1)

        # Update swarm center rectangle
        self.swarmbot.center = (self.swarm_center_state[0], self.swarm_center_state[1])

        self.swarm_time[bot_no] = time.time()

    def update_force_pull(self):
        """
        Given the current haptic device and swarm position(s), determine the forces
        or commands to be sent to the haptic feedback and swarm.
        :return:
        """
        # Update player rectangle
        self.player.center = (self.ee_state[0],self.ee_state[1])

        # Get position error between haptic device cursor and swarm center
        e = np.subtract(self.ee_state[0:2], self.swarm_center_state[0:2])
        #ToDo incorporate velocity back in IF all velocities above make sense
        #ToDo check ee_state, swarm_state, and swarm_center_state velocities
        # Get velocity error between haptic device cursor and swarm center
        #ed = np.subtract(self.ee_state[3:], self.swarm_state[3:])
        # Determine the magnitude of the distance between the two entities
        d = np.linalg.norm(e)
        F = [0,0,0]
        # If the haptic device cursor is close enough, update the force based on the distance
        if d < self.action_boundary:
            F[0:2] = self.gains['K_input'] * e #+ self.gains['V_input'] * ed
        else:
            pass

        # Adjust z force to encourage user back to middle of pre-defined range
        td_error = self.ee_state[2] - np.mean([self.tdmax, self.tdmin])
        F[2] = self.gains['K_z_feedback'] * td_error # + self.gains['V_z_feedback'] * self.ee_state[5]
        self.F = F
        self.haptic_force(self.F) # Send feedback to haptic device

        self.target_distance = self.ee_state[2]
        flocking_x = self.remap(self.ee_state[0], 0, helper.windowWidth, -15, 15)
        flocking_y = self.remap(self.ee_state[1], 0, helper.windowHeight, 15, -15)
        self.swarm_force(np.asarray(self.F), flocking_x, flocking_y, self.target_distance)


    def swarm_force(self, F, x_cmd, y_cmd, target_distance):
        '''
        Publish new position and flocking commands to the swarm
        :param F: not used
        :param x_cmd: desired X position
        :param y_cmd: desired Y position
        :param target_distance: desired target distance
        '''
        flocking_msg = Flocking()
        flocking_msg.distance = target_distance
        flocking_msg.y = y_cmd
        flocking_msg.x = x_cmd
        self.flock_pub.publish(flocking_msg)

    def haptic_force(self, F):
        haptic_output = OmniFeedback()
        haptic_output.force.x = max(min(-F[0] * self.gains['K_output'], 3), -3)
        haptic_output.force.y = max(min(F[1] * self.gains['K_output'], 3), -3)
        haptic_output.force.z = max(min(-F[2] * self.gains['K_output'], 3), -3)
        haptic_output.lock = [False, False, False]
        self.haptic_pub.publish(haptic_output)

    def update_GUI(self):

        self.display_surf.fill((0, 0, 0))
        self.player_draw()

        #Add text to top left corner
        text = "Current target distance: %.1f |-- %.1f --| %.1f" %(self.tdmin, self.target_distance, self.tdmax)
        myfont = pygame.font.SysFont('Comic Sans MS', 18)
        textsurface = myfont.render(text, False, helper.WHITE)
        self.display_surf.blit(textsurface, (0,0))

        #Always last step: update display
        pygame.display.update()


    def player_draw(self):
        """
        draws the player location
        """
        pygame.draw.rect(self.display_surf, helper.WHITE, self.player, 0)
        pygame.draw.ellipse(self.display_surf, helper.DARK_BLUE, self.swarmbot, 0)
        for bot in range(len(self.swarm_time)):
            endpoint = (self.swarm_state[0, bot], self.swarm_state[1, bot])
            pygame.draw.line(self.display_surf, helper.BLUE, self.swarmbot.center, endpoint )
            pygame.draw.circle(self.display_surf, helper.PEACH, endpoint, int(helper.PLAYERSIZE_X*0.5) )


if __name__ == "__main__":
    game = Game(10)
    while not rospy.is_shutdown():
        #game.get_input() # Use this line if input is coming from mouse
        game.update_force_pull()
        #game.update_player() # Use this line if the swarm simulation is not running
        game.update_GUI()

