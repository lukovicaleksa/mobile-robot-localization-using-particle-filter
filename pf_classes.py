#!/usr/bin/env python

import os
import numpy as np
import csv

from pf_functions import *

MAX_LIDAR_DISTANCE = 1.01 # maximum LIDAR distance in [m]
COLLISION_DISTANCE = 0.12 # collision distance in [m]

class Particle:
    """
    Particle class
    """
    def __init__(self, X, w):
        self.X = X  # [x, y, theta] coordinates
        self.w = w  # particle weight

    # for sorting purpose
    def get_weight(self):
        return self.w

class Wall:
    """
    Wall object class
    """
    def __init__(self, x_1, x_2, y_1, y_2, name):
        self.name = name

        # for loggin the map objects
        self.x_1 = x_1
        self.x_2 = x_2
        self.y_1 = y_1
        self.y_2 = y_2

        self.x = np.linspace(x_1, x_2, 1000)
        self.y = np.linspace(y_1, y_2, 1000)

class Map:
    """
    Map class
    """
    def __init__(self, name):
        self.name = name
        self.walls = []

    def extract_objects(self):
        """
        Extract objects in a map
        """
        model_properties = rosservice.call_service('/gazebo/get_model_properties', ['turtlebot3_square'])[1]

        for object_name in model_properties.body_names:
            if object_name[:4] == 'Wall':
                # Wall extraction
                wall = rosservice.call_service('/gazebo/get_link_state', [object_name, 'turtlebot3_square'])[1]

                x_center = wall.link_state.pose.position.x
                y_center = wall.link_state.pose.position.y
                theta = transform_orientation(wall.link_state.pose.orientation)
                length = 4.0  # Unreachable through rosservice, accessed from object_i -> collision -> geometry
                width = 0.15  # Unreachable through rosservice, accessed from object_i -> collision -> geometry
                name = wall.link_state.link_name

                # offset the objects center to collision spot
                x_center = x_center - width / 2 * np.sign(x_center) * np.abs(np.sin(theta))
                y_center = y_center - width / 2 * np.sign(y_center) * np.abs(np.cos(theta))

                # length of object is reduced by the overlaping area with other objects
                x_1 = x_center - (length / 2 - width) * np.cos(theta)
                x_2 = x_center + (length / 2 - width) * np.cos(theta)
                y_1 = y_center - (length / 2 - width) * np.sin(theta)
                y_2 = y_center + (length / 2 - width) * np.sin(theta)

                self.walls.append(Wall(x_1, x_2, y_1, y_2, name))

    def check_invalid_position(self, particle):
        """
        Checking if particle position on map is invalid
        """
        x_min, y_min, x_max, y_max = 100, 100, -100, -100

        for w in self.walls:
            x_min = np.min(np.append(w.x, x_min))
            x_max = np.max(np.append(w.x, x_max))
            y_min = np.min(np.append(w.y, y_min))
            y_max = np.max(np.append(w.y, y_max))

        x_min = x_min + COLLISION_DISTANCE
        x_max = x_max - COLLISION_DISTANCE
        y_min = y_min + COLLISION_DISTANCE
        y_max = y_max - COLLISION_DISTANCE

        if particle.X[0] < x_min or particle.X[0] > x_max or particle.X[1] < y_min or particle.X[1] > y_max:
            return True
        else:
            return False

    def save_objects(self):
        """
        Save map objects as .csv
        """
        for (i,w) in enumerate(self.walls):
            np.savetxt('Maps/' + self.name + '/' + w.name + '.csv', np.array([w.x_1, w.x_2, w.y_1, w.y_2]), delimiter = ' , ')

    def load_objects(self):
        """
        Load map objects from .csv
        """
        self.walls = []     # reinitialize map objects
        for object_name in os.listdir('Maps/' + self.name + '/'):
            (x_1, x_2, y_1, y_2) = np.genfromtxt('Maps/' + self.name + '/' + object_name, delimiter = ' , ')
            self.walls.append(Wall(x_1, x_2, y_1, y_2, object_name[:-4]))
