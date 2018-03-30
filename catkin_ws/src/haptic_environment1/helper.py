#!/usr/bin/env python

import sys
import pygame
import time
import numpy as np
import math

RED = (255,0,0)
PEACH = (255,100,100)
YELLOW = (200,200,0)
GREEN = (0,255,0)
MINT = (100,255,175)
AQUA = (0,150,150)
BLUE = (0,0,255)
DARK_BLUE = (0,0,128)
PINK = (255,200,200)
PURPLE = (255,150,255)
MAGENTA = (100,0,100)
WHITE = (255,255,255)
BLACK = (0,0,0)

BLOCKSIZE_X = 30
BLOCKSIZE_Y = 30
PLAYERSIZE_X = 10
PLAYERSIZE_Y = 10

windowWidth = 30*BLOCKSIZE_X
windowHeight = 18*BLOCKSIZE_Y

def invert(self):
    for index, row in enumerate(self.maze):
        self.maze[index] = row[::-1]

def rec_to_point(rec):
    pt = Point()
    pt.x = rec.centerx
    pt.y = rec.centery
    return pt

def get_i_j(maze,index):
    """
    converts 1D to 2D
    :param maze: occupancy grid mesage
    :param index: index of 1D array
    :return: 2 ints of the the 2D array indexs
    """
    N = maze.info.width
    j = index/N
    i = index % N

    return i,j

def check_cell(maze, pt):
    """
    Check if a cell is a wall or the goal (1 = wall or out of bounds, 2 = start, 3 = goal, 0 = path)
    :param maze:occupany grid message
    :param pt:index to check
    :return:what is at that index
    """
    N = maze.info.width
    M = maze.info.height

    if maze.data[pt] == 2:
        return 2
    elif maze.data[pt] == 3:
        return 3
    elif maze.data[pt] == 0:
        return 0
    else:
        return 1

def getStart(maze):
    """
    get the stating location
    :param maze: occupany grid message
    :return: 2D index of the starting location
    """
    start = maze.data.index(2)
    return get_i_j(maze,start)

def getGoal(maze):
    """
    gets the goal location
    :param maze: occupany grid message
    :return: 2D index of the goal location
    """
    goal = maze.data.index(3)
    return get_i_j(maze,goal)

def index_to_cell(maze,x,y):
    """
    converts 2D index to 1D index
    :param maze: occupancy grid message
    :param x: x index of 2D array
    :param y: y index of 2D array
    :return: index of 1D array
    """
    if x < 0 or x >= maze.info.width or y < 0 or y >= maze.info.height:
        return maze.data.index(1)
    else:
        return maze.info.width*y + x

def joint_to_game(x_range, y_range  ):

    (position, velocity, effort) = tools.helper.call_return_joint_states()
    # scales the input to the game
    EE_y = tools.helper.remap(round(position[0],5),-0.1,0.1,x_range[0],x_range[1] )
    EE_x = tools.helper.remap(round(position[2],5),0.6,1.9,y_range[0],y_range[1])
    return (EE_y,EE_x)

def task_to_game(x, y):

    EE_x = tools.helper.remap(x,-0.10,0.10,0, windowWidth )
    EE_x = max(0, min(EE_x, windowWidth))

    EE_y = tools.helper.remap(y,-0.08,-0.21,0, windowHeight)
    EE_y = max(0, min(EE_y, windowHeight))

    return (EE_x,EE_y)

def neighbors_euclidean(maze, loc_x, loc_y, looking_for = [0,2,3]):
    neighbors_in = [(loc_x - 1, loc_y - 1), (loc_x, loc_y - 1), (loc_x + 1, loc_y - 1),\
                    (loc_x - 1, loc_y),     (loc_x, loc_y),     (loc_x + 1, loc_y),\
                    (loc_x - 1, loc_y + 1), (loc_x, loc_y + 1), (loc_x + 1, loc_y + 1)]

    neighbors_out = []
    for option in neighbors_in:
        if check_cell(maze, index_to_cell(maze, option[0], option[1])) in looking_for:
            neighbors_out.append(option)

    return neighbors_out

def neighbors_manhattan(maze,loc_x, loc_y, looking_for = [0,2,3]):
    neighbors_in = [(loc_x - 1, loc_y), (loc_x, loc_y + 1), (loc_x + 1, loc_y), (loc_x, loc_y - 1)]
    neighbors_out = []
    for option in neighbors_in:
        if check_cell(maze, index_to_cell(maze, option[0], option[1])) in looking_for:
            neighbors_out.append(option)

    return neighbors_out

def collision_plane(maze,player):
    """
    Check for walls and obstacles in the player's vicinity. Looking for the closest points in the wall(s), so the
    repulsive force is perpendicular to the wall
    :param maze: Occupancy Grid
    :param player: Rect
    :return: points = array of Point() objects representing the closest point in the obstacles found
    """
    points = []
    loc_x = int(float(player.centerx)/BLOCKSIZE_X) # This is the (x,y) block in the grid where the top left corner of the player is
    loc_y = int(float(player.centery)/BLOCKSIZE_Y)
    looking_for = [1]

    option = (loc_x - 1, loc_y)

    if check_cell(maze, index_to_cell(maze, option[0], option[1])) in looking_for:
        point = project( (BLOCKSIZE_X*(option[0]+1), BLOCKSIZE_Y*option[1]), (BLOCKSIZE_X*(option[0]+1), BLOCKSIZE_Y*(option[1]+1)), player.center)
        points.append(make_point(point[0], point[1]))

    option = (loc_x , loc_y + 1)

    if check_cell(maze, index_to_cell(maze, option[0], option[1])) in looking_for:
        point = project( (BLOCKSIZE_X*(option[0]), BLOCKSIZE_Y*(option[1])), (BLOCKSIZE_X*(option[0]+1), BLOCKSIZE_Y*(option[1])), player.center)
        points.append(make_point(point[0], point[1]))

    option = (loc_x + 1 , loc_y )

    if check_cell(maze, index_to_cell(maze, option[0], option[1])) in looking_for:
        point = project( (BLOCKSIZE_X*(option[0]), BLOCKSIZE_Y*(option[1]+1)), (BLOCKSIZE_X*(option[0]), BLOCKSIZE_Y*(option[1])), player.center)
        points.append(make_point(point[0], point[1]))

    option = (loc_x , loc_y - 1)

    if check_cell(maze, index_to_cell(maze, option[0], option[1])) in looking_for:
        point = project( (BLOCKSIZE_X*(option[0]+1), BLOCKSIZE_Y*(option[1]+1)), (BLOCKSIZE_X*(option[0]), BLOCKSIZE_Y*(option[1] + 1)), player.center)
        points.append(make_point(point[0], point[1]))

    return points

def project(v, w, p):
    """
    "Project the player's "shadow" on the nearby walls.
    :param v: 1 of two nearest corners of a wall block to the player
    :param w: 1 of two nearest corners of a wall block to the player
    :param p: player location
    :return: projection = closest point on line segment between v and w to player p
    """
    v_vec = np.asarray( v, dtype=float )
    w_vec = np.asarray( w,dtype=float )
    p_vec = np.asarray( p,dtype=float )

    pv = p_vec - v_vec
    wv = w_vec - v_vec

    l = np.linalg.norm( wv )**2
    t = max( 0, min(1, np.dot(pv,wv)/l) )
    projection = v_vec + t*(wv)

    return projection


def check_collision_adaptive(player,maze):

    walls = []
    centers = []
    player_x = int(float(player.centerx)/BLOCKSIZE_X) # This is the (x,y) block in the grid where the top left corner of the player is
    player_y = int(float(player.centery)/BLOCKSIZE_Y)
    neighbor_walls = neighbors_manhattan(maze, player_x,player_y, [1])
    for neighbor in neighbor_walls:
        point = Point()
        wall_block = pygame.Rect((neighbor[0] * BLOCKSIZE_X, neighbor[1] * BLOCKSIZE_Y, BLOCKSIZE_X, BLOCKSIZE_Y))
        point.x = wall_block.x
        point.y = wall_block.y
        centers.append(point)
        walls.append(wall_block)

    return centers,walls


def goal_adaptive(start,goal,path):

    points = []
    goal = maze_helper.rec_to_point(goal)
    start = maze_helper.rec_to_point(start)
    points.append(goal)
    points.append(start)

    if path:
        for rec in path:
            pt = Point()
            pt.x = (rec.centerx * maze_helper.BLOCKSIZE_X) + math.floor(abs((maze_helper.BLOCKSIZE_X - maze_helper.PLAYERSIZE_X) * 0.5))
            pt.y = (rec.centery * maze_helper.BLOCKSIZE_Y) + math.floor(abs((maze_helper.BLOCKSIZE_Y - maze_helper.PLAYERSIZE_Y) * 0.5))
            points.append(pt)

    points.append(goal)

    return points

def make_point(x, y):
    point = Point()
    point.x = x
    point.y = y

    return point