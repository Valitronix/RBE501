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
        self.angle = 0

        pygame.display.set_caption('Use the cursor to move the swarm bot')

        rospy.init_node("python_controller")

        for i in range(10):
            topic_string = "/bot"+str(i)+"/State"
            rospy.Subscriber(topic_string, State, self.update_from_bot, (i))

        rospy.Subscriber("/Geomagic/end_effector_pose", JointState, self.get_input_haptic)
        rospy.Subscriber("/Geomagic/button", PhantomButtonEvent, self.button_callback)
        rospy.Subscriber("/Geomagic/joint_states", JointState, self.get_rotation_haptic)

        self.haptic_pub = rospy.Publisher("/Geomagic/force_feedback", OmniFeedback, queue_size=1)
        self.flock_pub = rospy.Publisher("/Flocking", Flocking, queue_size=1)
        self.multi_bot_pub = rospy.Publisher("/Haptic", Haptic, queue_size=1)


        pygame.init()
        pygame.font.init()

    # def get_input_mouse(self):
    #     # Gets input from mouse
    #     pygame.event.pump()
    #     (in_x, in_y) = pygame.mouse.get_pos()
    #     ## AVQuestion can we get velocity from the end effector, too?
    #     self.ee_state = [[in_x], [in_y], [0], [0], [0], [0]]
    #     self.player.center = (self.ee_state[0][0], self.ee_state[1][0])
    #     (bt1, bt2, bt3) = pygame.mouse.get_pressed()
    #     self.running = not bt1

    def get_rotation_haptic(self, rot):
        self.angle = (np.floor((rot.position[5] - np.pi)*(180/np.pi)/10))*np.pi/18
    def get_input_haptic(self, js):
        if not rospy.is_shutdown():
            dt = max(time.time() - self.time0, 0.1)
            self.time0 = time.time()
            temp_0 = self.remap(js.position[0], -150, 150, 0, helper.windowWidth)
            self.ee_state[3] = (temp_0 - self.ee_state[0])/dt
            self.ee_state[0] = temp_0
            temp_1 = self.remap(js.position[1], 90, -90, 0, helper.windowHeight)
            self.ee_state[4] = (temp_1 - self.ee_state[1]) / dt
            self.ee_state[1] = temp_1
            temp_2 = self.remap(js.position[2], -50, 50, self.tdmin, self.tdmax)
            self.ee_state[2] = min(self.tdmax, max(self.tdmin, temp_2))


    def button_callback(self, button):
        step = 10
        if button.grey_button:
            self.target_distance = min(100, self.target_distance + step)
        if button.white_button:
            self.target_distance = max(10, self.target_distance - step)


    def remap(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # def update_player(self):
    #     #For when the swarm simulation is not running
    #     """
    #     call_back when the player's location has changed, based on end-effector movement. The player's location in the
    #     task space, where (0,0) is in the middle of the task space, the x-axis is positive to the right, and the y-axis
    #     is positive going up, is converted to a location in the game, where (0,0) is in the top left corner, x-axis
    #     extends right, and y-axis points down.
    #     :param msg: x and y location of the player in the end effector space (actually x and z location in robot frame)
    #     :return:
    #     """
    #     dt = time.clock() - self.timedt
    #     xdd = np.array(self.F).reshape(3, 1) / self.swarm_mass
    #     B = np.zeros(shape=(6, 3))
    #     A = np.identity(6)
    #
    #     B[3, 0] = dt
    #     B[4, 1] = dt
    #     B[5, 2] = dt
    #
    #     A[0, 3] = dt
    #     A[1, 4] = dt
    #     A[2, 5] = dt
    #
    #     self.swarm_center_state = np.dot(A, self.swarm_center_state) + np.dot(B, xdd)
    #
    #     #self.swarmbot.center = (self.swarm_state[0][0], self.swarm_state[1][0])
    #     self.time0 = time.clock()

    def update_from_bot(self, state, bot_no):
        dt = time.time() - self.swarm_time[bot_no]
        #Gets current position from swarm robot
        x_virtual = self.remap(state.x, -15, 15, 0, helper.windowWidth)
        y_virtual = self.remap(state.y, 15, -15, 0, helper.windowHeight)
        x_dot_virtual = (x_virtual - self.swarm_state[0,bot_no])/dt
        y_dot_virtual = (y_virtual - self.swarm_state[1,bot_no])/dt
        state_temp = np.zeros((6,1))
        state_temp[0] = x_virtual
        state_temp[1] = y_virtual
        self.swarm_state[:, bot_no] = state_temp.reshape(6,1)
        self.swarm_center_state = np.mean(self.swarm_state, 1)
        self.swarmbot.center = (self.swarm_center_state[0], self.swarm_center_state[1])

        self.swarm_time[bot_no] = time.time()

    def update_force_pull(self):
        """
        :return:
        """
        self.player.center = (self.ee_state[0],self.ee_state[1])

        e = np.subtract(self.ee_state[0:2], self.swarm_center_state[0:2])
        #ToDo incorporate velocity back in IF all velocities above make sense
        #ToDo check ee_state, swarm_state, and swarm_center_state velocities
        #ed = np.subtract(self.ee_state[3:], self.swarm_state[3:])
        d = np.linalg.norm(e)
        F = [0,0,0]
        if d < self.action_boundary:
            F[0:2] = self.gains['K_input'] * e #+ self.gains['V_input'] * ed
        else:
            pass

        #ToDo adjust z force to encourage user back to middle of pre-defined range
        td_error = self.ee_state[2] - np.mean([self.tdmax, self.tdmin])
        F[2] = self.gains['K_z_feedback'] * td_error # + self.gains['V_z_feedback'] * self.ee_state[5]
        self.F = F
        self.haptic_force(self.F)

        self.target_distance = self.ee_state[2]
        flocking_x = self.remap(self.ee_state[0], 0, helper.windowWidth, -15, 15)
        flocking_y = self.remap(self.ee_state[1], 0, helper.windowHeight, 15, -15)
        self.swarm_force(np.asarray(self.F), flocking_x, flocking_y, self.target_distance)


    def swarm_force(self, F, x_cmd, y_cmd, target_distance):
        R_mat = np.matrix([[cos(self.swarm_heading), sin(self.swarm_heading), 0],
                           [-sin(self.swarm_heading), cos(self.swarm_heading), 0],
                           [                       0,                       0, 1]])

        force_vector = R_mat*F.reshape(3,1)
        output_force = Haptic()
        output_force.x_value = force_vector[0,0]
        output_force.y_value = -force_vector[1, 0]
        output_force.z_value = force_vector[2,0]
        flocking_msg = Flocking()
        flocking_msg.distance = target_distance
        flocking_msg.y = y_cmd
        flocking_msg.x = x_cmd
        self.flockingx = x_cmd
        self.flockingy = y_cmd
        self.multi_bot_pub.publish(output_force)
        self.flock_pub.publish(flocking_msg)

    def haptic_force(self, F):
        haptic_output = OmniFeedback()
        haptic_output.force.x = max(min(-F[0] * self.gains['K_output'], 3), -3)
        haptic_output.force.y = max(min(F[1] * self.gains['K_output'], 3), -3)
        haptic_output.force.z = max(min(-F[2] * self.gains['K_output'], 3), -3)
        haptic_output.lock = [False, False, False]
        self.haptic_pub.publish(haptic_output)

    def update_GUI(self):
        """
        refeshs the game on a timer callback
        :msg: not used
        :return:
        """

        self.display_surf.fill((0, 0, 0))
        self.player_draw()

        scoretext = "Current target distance: %.1f |-- %.1f --| %.1f" %(self.flockingx, self.flockingy,self.target_distance)
        myfont = pygame.font.SysFont('Comic Sans MS', 18)
        textsurface = myfont.render(scoretext, False, helper.WHITE)
        self.display_surf.blit(textsurface, (0,0))
        pygame.display.update()


    def player_draw(self):
        """
        draws the player location
        :return:
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
        #game.get_input()
        game.update_force_pull()
        #game.update_player()
        game.update_GUI()

