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
        #
        # self.goal_rec = None
        # self.start_rec = None
        # self.swarm_output = Haptic()
        # self.running = True
        # self.am_i_at_goal = False
        # self.am_i_at_start = False
        # self.swarm_mass = 5
        self.action_boundary = 1000
        self.player = Rect((helper.windowWidth*0.5, helper.windowHeight*0.5, helper.PLAYERSIZE_X, helper.PLAYERSIZE_Y) )
        self.swarmbot = Rect((helper.windowWidth*0.5, helper.windowHeight*0.5, helper.BLOCKSIZE_X, helper.BLOCKSIZE_Y) )
        self.display_surf = pygame.display.set_mode((helper.windowWidth, helper.windowHeight))
        self.ee_state = np.asarray([[0], [0], [0], [0], [0], [0]])
        self.swarm_state = np.asarray([[300] * num_bots,[300] * num_bots,[0] * num_bots,[0] * num_bots,[0] * num_bots,[0] * num_bots])
        self.swarm_center_state = np.asarray([[300],[300],[0],[0],[0],[0]])
        self.swarm_heading = [0] * num_bots
        self.F = [0,0,0]
        self.time0 = time.time()
        self.swarm_time = [time.time()] * num_bots
        self.gains = {'K_input': 1,
                      'V_input': 0,
                      'K_z_feedback': 1.2,
                      'V_z_feedback': 0.5,
                      'K_output': 0.05,
                      }
        self.tdmin = 0
        self.tdmax = 100
        pygame.display.set_caption('Use the cursor to move the swarm bot')
        #self.csv = open("/home/cibr-strokerehab/Documents/JointStatesRecording.csv", "w")

        rospy.init_node("python_controller")

        for i in range(10):
            topic_string = "/bot"+str(i)+"/state"
            rospy.Subscriber(topic_string, State, self.update_from_bot, (i))

        rospy.Subscriber("/Geomagic/end_effector_pose", JointState, self.get_input_haptic)
        rospy.Subscriber("/Geomagic/button", PhantomButtonEvent, self.button_callback)

        #self.one_bot_pub = rospy.Publisher("/bot0/haptic", Haptic, queue_size=1)
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


    def get_input_haptic(self, js):
        # Test: print ee_state and check velocity
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
        pass


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
    #     dt = time.clock() - self.time0
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
    #     self.swarm_state = np.dot(A, self.swarm_state) + np.dot(B, xdd)
    #
    #     self.swarmbot.center = (self.swarm_state[0][0], self.swarm_state[1][0])
    #     self.time0 = time.clock()

    def update_from_bot(self, state, bot_no):
        # Test: print swarm.state, swarmbot center, check that it starts in center and moves from there
        # Test: does velocity make sense in pixels/second?
        dt = time.time() - self.swarm_time[bot_no]
        #Gets current position from swarm robot
        x_virtual = self.remap(state.x, -2.5, 2.5, 0, helper.windowWidth)
        y_virtual = self.remap(state.y, 2.5, -2.5, 0, helper.windowHeight)
        x_dot_virtual = (x_virtual - self.swarm_state[0,bot_no])/dt
        y_dot_virtual = (y_virtual - self.swarm_state[1,bot_no])/dt
        #x_dot_virtual = self.remap(state.dot_x, -2.5, 2.5, 0, helper.windowWidth)
        #y_dot_virtual = self.remap(state.dot_y, 2.5, -2.5, 0, helper.windowHeight)
        state_temp = np.zeros(6)
        state_temp[0] = x_virtual
        state_temp[1] = y_virtual
        self.swarm_state[:, bot_no] = state_temp
        #self.swarm_heading[bot_no] = -state.yaw
        self.swarm_center_state = np.mean(self.swarm_state, 1)
        self.swarmbot.center = (self.swarm_center_state[0], self.swarm_center_state[1])

        self.swarm_time[bot_no] = time.time()

    def update_force_pull(self):
        """
        :return:
        """
        self.player.center = (self.ee_state[0],self.ee_state[1])

        # Test: Does the error still make sense?
        e = np.subtract(self.ee_state[0:2], self.swarm_center_state[0:2])
        print "ee_state", self.ee_state[0:2]
        print "swarm bot 0", self.swarm_state[:,0]
        print "swarm_center", self.swarm_center_state
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
        F[2] = self.gains['K_z_feedback'] * self.ee_state[2] + self.gains['V_z_feedback'] * self.ee_state[5]
        self.F = F
        #print "F", self.F
        self.haptic_force(self.F)

        target_distance = self.ee_state[3]
        self.swarm_force(np.asarray(self.F), 0, 0, target_distance)


    def swarm_force(self, F, velocity, heading, target_distance):
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
        flocking_msg.y = velocity
        flocking_msg.x = heading
        self.multi_bot_pub.publish(output_force)
        self.flock_pub.publish(flocking_msg)

    def haptic_force(self, F):
        haptic_output = OmniFeedback()
        haptic_output.force.x = max(min(-F[0] * self.gains['K_output'], 3), -3)
        haptic_output.force.y = max(min(F[1] * self.gains['K_output'], 3), -3)
        haptic_output.force.z = max(min(-F[2] * self.gains['K_output'], 3), -3)
        haptic_output.lock = [False, False, False]
        #print haptic_output.force
        #b self.haptic_pub.publish(haptic_output)

    def update_GUI(self):
        """
        refeshs the game on a timer callback
        :msg: not used
        :return:
        """

        #AVQuestion could we speed this up by only drawing the blocks around the player's position?
        self.display_surf.fill((0, 0, 0))
        #self.maze_draw()
        self.player_draw()
        # Test: how does this target distance text look? is it correct?

        scoretext = "Current target distance: %.1f |-- %.1f --| %.1f" %(self.tdmin, self.ee_state[2], self.tdmax)
        myfont = pygame.font.SysFont('Comic Sans MS', 18)
        textsurface = myfont.render(scoretext, False, helper.WHITE)
        self.display_surf.blit(textsurface, (0,0))
        pygame.display.update()


    # def maze_callback(self,msg):
    #     """
    #     Called when a new maze is published, identifies maze components and calls initialization function
    #     :param msg: occupany grid message
    #     :return:
    #     """
    #     self.maze = msg
    #     self.walls = []
    #     starts = []
    #     goals = []
    #     for index, pt in enumerate(self.maze.data):
    #         bx,by = maze_helper.get_i_j(self.maze,index)
    #         cell = maze_helper.check_cell(self.maze,index)
    #         if cell == 1:
    #             self.walls.append(pygame.Rect(bx * maze_helper.BLOCKSIZE_X, by * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
    #         elif cell == 2:
    #             starts.append(pygame.Rect(bx * maze_helper.BLOCKSIZE_X, by * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
    #             self.start_rec = starts[0].unionall(starts)
    #         elif cell == 3:
    #             goals.append(pygame.Rect(bx * maze_helper.BLOCKSIZE_X, by * maze_helper.BLOCKSIZE_Y, maze_helper.BLOCKSIZE_X, maze_helper.BLOCKSIZE_Y))
    #             self.goal_rec = goals[0].unionall(goals)
    #
    #     self.on_init()


    def player_draw(self):
        """
        draws the player location
        :return:
        """
        # if self.am_i_at_start:
        #     self.update_score()
        #     print "Score:", self.score
        pygame.draw.rect(self.display_surf, helper.WHITE, self.player, 0)
        pygame.draw.ellipse(self.display_surf, helper.DARK_BLUE, self.swarmbot, 0)
        for bot in range(len(self.swarm_time)):
            endpoint = (self.swarm_state[0, bot], self.swarm_state[1, bot])
            pygame.draw.line(self.display_surf, helper.BLUE, self.swarmbot.center, endpoint )
        # Test: Does this drawing make sense when the little bots are running around?

    # def maze_draw(self):
    #     """
    #     callback for the maze
    #     draws the maze
    #     :return: none
    #     """
    #     for wall in self.walls:
    #             pygame.draw.rect(self.display_surf, maze_helper.PURPLE, wall, 0)
    #     pygame.draw.rect(self.display_surf, maze_helper.RED, self.goal_rec, 0)
    #     pygame.draw.rect(self.display_surf, maze_helper.BLUE, self.start_rec, 0)
    #
    # def at_start(self):
    #     """
    #     checks if we are at the starting location, and if so, starts timer
    #     :return: boolean check if we are at the starting location
    #
    #     """
    #     state = Bool()
    #     state.data = self.start_rec.contains(self.player)
    #     if state.data:
    #         self.game_timer = time.time()
    #     return state.data
    #
    # def at_goal(self):
    #     """
    #     checks if we are at the goal, and if so, publishes a flag to generate a new maze, calculates the score, and
    #     begins the reset process.
    #     :return: boolean check if we are at goal
    #     """
    #     state = Bool()
    #     state.data = self.goal_rec.contains(self.player)
    #     if state.data:
    #         self.pub_goal.publish(state)
    #         self.running = False
    #         time_score = 20 - (time.time() - self.game_timer)
    #         self.score += time_score
    #
    #     return state.data


    # def update_score(self, assistance=3):
    #
    #     player_x = math.floor(float(self.player.centerx) / maze_helper.BLOCKSIZE_X)  # This is the (x,y) block in the grid where the center of the player is
    #     player_y = math.floor(float(self.player.centery) / maze_helper.BLOCKSIZE_Y)
    #     point_index = maze_helper.index_to_cell(self.maze, player_x, player_y)
    #     if maze_helper.check_cell(self.maze, int(point_index)) == 1:
    #         self.score -= 1
    #     for rec in self.solved_path:
    #         if rec.centerx == player_x and rec.centery == player_y:
    #             #print "On track"
    #             reward = math.floor(1./len(self.solved_path) * 100)
    #             self.score += reward
    #         # else:
    #         #     print "Player (x,y):", player_x, player_y
    #         #     print "Waypoint (x,y):", pose.pose.position.x, pose.pose.position.y


if __name__ == "__main__":
    game = Game(4)
    while not rospy.is_shutdown():
        #game.get_input()
        game.update_force_pull()
        #game.update_player()
        game.update_GUI()

