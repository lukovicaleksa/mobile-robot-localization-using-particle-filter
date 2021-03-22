#! /usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from datetime import datetime
from time import sleep

import sys
MODULES_PATH = '/home/maestro/catkin_ws/src/pf_localization/scripts'
sys.path.insert(0, MODULES_PATH)

from pf_functions import *

MAX_SIMULATION_TIME = 180 # in seconds
X_INIT = -6.5
Y_INIT = -3
THETA_INIT = 90

max_sim = False

if __name__ == '__main__':
    try:
        rospy.init_node('scan_node', anonymous = False)
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

        now = datetime.now()
        dt_string_start = now.strftime("%d/%m/%Y %H:%M:%S")
        print('SCAN NODE START ==> ', dt_string_start ,'\r\n')

        scan_time = 0
        count = 0

        t_0 = rospy.Time.now()
        t_start = rospy.Time.now()

        # init timer
        while not (t_start > t_0):
            t_start = rospy.Time.now()

        t = t_start

        # Init figure - real time
        plt.style.use('seaborn-ticks')
        fig = plt.figure(1)
        ax = fig.add_subplot(1,1,1)

        # main loop
        while not rospy.is_shutdown():

            # Lidar measurements
            msgScan = rospy.wait_for_message('/scan', LaserScan)
            ( distances, angles ) = lidar_scan(msgScan, 3.5)  # distances in [m], angles in [degrees]

            # Odometry measurements
            msgOdom = rospy.wait_for_message('/odom', Odometry)
            (x_odom, y_odom) = get_odom_position(msgOdom)   # x,y in [m]
            theta_odom = get_odom_orientation(msgOdom)      # theta in [radians]

            scan_time = (rospy.Time.now() - t).to_sec()
            sim_time = (rospy.Time.now() - t_start).to_sec()
            count = count + 1

            print('\r\nScan cycle:', count , '\r\nScan time:', scan_time, 's')
            print('Simulation time:', sim_time, 's')
            t = rospy.Time.now()

            # Lidar measurements x-y plane
            distances_x = np.array([])
            distances_y = np.array([])

            for (dist, ang) in zip(distances, angles):
                distances_x = np.append(distances_x, x_odom + dist * np.cos(ang + theta_odom))
                distances_y = np.append(distances_y, y_odom + dist * np.sin(ang + theta_odom))

            # Plot
            ax.clear()

            # Plot Lidar measurements
            ax.plot(distances_x, distances_y, 'b.', markersize = 1.2, label = 'lidar')

            # Plot Robot position
            dir_robot_x = np.array([x_odom, x_odom + 0.25 * np.cos(theta_odom)])
            dir_robot_y = np.array([y_odom, y_odom + 0.25 * np.sin(theta_odom)])

            ax.plot(x_odom, y_odom, 'r.', markersize = 8, label = 'robot')
            ax.plot(dir_robot_x, dir_robot_y, color = 'red', lw = 1.5)

            plt.xlabel('x[m]')
            plt.ylabel('y[m]')
            plt.title('Scan node')
            plt.xlim([-10, 10])
            plt.ylim([-6, 6])
            plt.draw()
            plt.pause(0.0001)

            if sim_time > MAX_SIMULATION_TIME:
                now = datetime.now()
                dt_string_stop = now.strftime("%d/%m/%Y %H:%M:%S")
                print('\r\nSCAN NODE START ==> ', dt_string_start ,'\r\n')
                print('SCAN NODE STOP ==> ', dt_string_stop ,'\r\n')
                rospy.signal_shutdown('End of simulation')
                max_sim = True

            rate.sleep()

    except rospy.ROSInterruptException:

        if not max_sim:
            print('\r\nSIMULATION TERMINATED!')
            now = datetime.now()
            dt_string_stop = now.strftime("%d/%m/%Y %H:%M:%S")
            print('\r\nSCAN NODE START ==> ', dt_string_start ,'\r\n')
            print('SCAN NODE STOP ==> ', dt_string_stop ,'\r\n')
            rospy.signal_shutdown('End of simulation')

        pass
