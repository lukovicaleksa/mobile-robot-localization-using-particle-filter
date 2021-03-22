#! /usr/bin/env python

import rospy

import os
import csv

from time import time
from time import sleep
from datetime import datetime
import matplotlib.pyplot as plt

import sys
DATA_PATH = '/home/maestro/catkin_ws/src/pf_localization/Data'
MODULES_PATH = '/home/maestro/catkin_ws/src/pf_localization/scripts'
sys.path.insert(0, MODULES_PATH)

from pf_classes import *

X_INIT = 1.3      # in [m]
Y_INIT = 0.0      # in [m]
THETA_INIT = 0.0  # in [degree]

SIM_TIME = 30.0   # Simulation time [s]

LOG_DIR = DATA_PATH + '/test'

if __name__ == '__main__':
    try:
        # Node initialization
        rospy.init_node('pf_algo_node', anonymous = False)
        rate = rospy.Rate(10)

        setPosPub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)

        # Set in initial position and orientation
        robot_in_pos = False

        while not robot_in_pos:
            (x_init, y_init, theta_init) = robot_set_pos(setPosPub, X_INIT, Y_INIT, radians(THETA_INIT))

            msgOdom = rospy.wait_for_message('/odom', Odometry)
            (x, y) = get_odom_position(msgOdom)     # x,y in [m]
            theta = get_odom_orientation(msgOdom)   # theta in [radians]

            # check init pos
            if abs(x - x_init) < 0.01 and abs(y - y_init) < 0.01:
                robot_in_pos = True
                print('Robot in pos\r\n')
                sleep(3)
            else:
                robot_in_pos = False
                print('Robot not in pos\r\n')

        # Initialize log data
        X_odom = []
        Y_odom = []
        THETA_odom = []
        DISTANCES_lidar = []
        ANGLES_lidar = []

        # Initialize time
        t_start = rospy.Time.now()
        print('SIMULATION HAS STARTED! \r\n')

        # main loop
        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - t_start).to_sec()

            # LIDAR measurements
            msgScan = rospy.wait_for_message('/scan', LaserScan)
            (distances, angles) = lidar_scan(msgScan, 1.01)       # distances in [m], angles in [radians]

            # Odometry measurements
            msgOdom = rospy.wait_for_message('/odom', Odometry)
            (x_odom, y_odom) = get_odom_position(msgOdom)   # x,y in [m]
            theta_odom = get_odom_orientation(msgOdom)      # theta in [radians]

            # Appending log data
            X_odom.append(x_odom)
            Y_odom.append(y_odom)
            THETA_odom.append(theta_odom)
            DISTANCES_lidar.append(distances)
            ANGLES_lidar.append(angles)

            crash = check_crash(distances)

            print('CRASH: ' + str(crash))
            print('ODOMETRY: (x,y,theta): (%.2f, %.2f, %.2f)' % (x_odom, y_odom, theta_odom))
            print('ELAPSED TIME %.2f in [s]' % elapsed_time)
            print('\r\n')

            # End of simulation
            if elapsed_time > SIM_TIME:
                # Log the data
                np.savetxt(LOG_DIR + '/X_odom.csv', np.array(X_odom), delimiter = ' , ')
                np.savetxt(LOG_DIR + '/Y_odom.csv', np.array(Y_odom), delimiter = ' , ')
                np.savetxt(LOG_DIR + '/THETA_odom.csv', np.array(THETA_odom), delimiter = ' , ')

                DIST_LIDAR = np.empty([len(DISTANCES_lidar), len(DISTANCES_lidar[0])])
                ANG_LIDAR = np.empty([len(DISTANCES_lidar), len(DISTANCES_lidar[0])])

                for i in range(len(DISTANCES_lidar)):
                    DIST_LIDAR[i,:] = DISTANCES_lidar[i]
                    ANG_LIDAR[i,:] = ANGLES_lidar[i]

                np.savetxt(LOG_DIR + '/DISTANCES_lidar.csv', DIST_LIDAR, delimiter = ' , ')
                np.savetxt(LOG_DIR + '/ANGLES_lidar.csv', ANG_LIDAR, delimiter = ' , ')
                break

    except rospy.ROSInterruptException:
        print('Simulation terminated!')
        pass
