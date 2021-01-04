#!/usr/bin/env python

import rospy
import rosservice

import numpy as np
import numpy.matlib as nm
from math import *

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState

from pf_classes import *

MAX_LIDAR_DISTANCE = 1.01 # maximum LIDAR distance in [m]
COLLISION_DISTANCE = 0.12 # collision distance in [m]

def resample_particles(particles, map):
    """
    Resample particles from distribution based on weights w_t
    """
    weights =  np.array([p.w for p in particles])
    resampled_particles = []

    while not (len(resampled_particles) == len(particles)):

        rnd = np.random.rand()           # unifrom distribution between 0-1
        cumsum = np.cumsum(weights)     # cumulative sum of particles weights

        # find index of selected particle
        ind = np.where(cumsum > rnd)[0][0]

        # resample new particle around selected particle
        mu_x = particles[ind].X[0]
        sigma_x = 0.01  # in [m]
        mu_y = particles[ind].X[1]
        sigma_y = 0.01  # in [m]
        mu_theta = particles[ind].X[2]
        sigma_theta = 0.06  # in [radians]

        X_res_particle = [np.random.normal(mu_x, sigma_x),
                          np.random.normal(mu_y, sigma_y),
                          np.random.normal(mu_theta, sigma_theta)]

        w_res_particle = 1 / len(particles)   # Not important at the moment, will be recalculated

        res_particle = Particle(X_res_particle, w_res_particle)

        # if resampled particle position is valid, add it to the list of resampled particles
        if not map.check_invalid_position(res_particle):
            resampled_particles.append(res_particle)

    return resampled_particles

def normalize_particle_weights(particles):
    """
    Normalize weights of particles to satisfy: sum(weights) = 1.0
    """
    cumsum = 0.0

    for p in particles:
        cumsum += p.w

    for p in particles:
        p.w /= (cumsum + 1e-6)

    return particles

def calculate_particle_weight(p, angles, distances, map):
    """
    Calculate particle weights based on measurements: w_t = p(z_t|x_t)
    """
    p_distances = np.array([])
    p_angles = angles

    for ang in angles:
        distances_to_obstacles = []

        for w in map.walls:
            # fit line parameters r and alpha
            (r, alpha) = fit_line(np.array([w.x, w.y]), np.array([p.X[0], p.X[1], p.X[2]]))

            # distance to object
            obst_dist = r / np.cos(ang - alpha)

            # object bounds
            x_min = np.min(w.x)
            x_max = np.max(w.x)
            y_min = np.min(w.y)
            y_max = np.max(w.y)

            # corners tolerance
            x_min -= (0.02 * np.sign(x_min)) * x_min
            x_max += (0.02 * np.sign(x_max)) * x_max
            y_min -= (0.02 * np.sign(y_min)) * y_min
            y_max += (0.02 * np.sign(y_max)) * y_max

            # intersection in (x,y) coordinates
            intersection_x = p.X[0] + obst_dist * np.cos(ang + p.X[2])
            intersection_y = p.X[1] + obst_dist * np.sin(ang + p.X[2])

            # object parallel to lidar beam
            if obst_dist < 100:
                parallel = False
            else:
                parallel = True

            # object behind lidar beam
            if obst_dist > 0:
                behind = False
            else:
                behind = True

            # lidar beam hits the object - bounded line
            if x_min < intersection_x < x_max and y_min < intersection_y < y_max:
                out_of_bounds = False
            else:
                out_of_bounds = True

            if not out_of_bounds and not parallel and not behind:
                distances_to_obstacles.append(obst_dist)

        # if lidar beam hits near the corner it will intersect 2 obstacles -> choose closest
        min_obst_dist = min(distances_to_obstacles)

        # limit measurement
        if min_obst_dist > MAX_LIDAR_DISTANCE:
            min_obst_dist = MAX_LIDAR_DISTANCE

        p_distances = np.append(p_distances, min_obst_dist)

    # euc dist between estimated lidar measurements of particle and ground truth
    lidar_est_dist = np.linalg.norm(distances - p_distances)

    # max euc dist between estimated lidar measurements of particle and ground truth
    max_lidar_est_dist = np.linalg.norm(MAX_LIDAR_DISTANCE * np.ones(len(distances)) - COLLISION_DISTANCE * np.ones(len(distances)))

    # weight of particle - not normalized
    p.w = (max_lidar_est_dist - lidar_est_dist) ** 2

    return p

def apply_state_transision(p, dX):
    """
    Apply state transision model p(x_t|x_t_1, u_t) to particle
    """
    traveled_distance = np.sqrt(dX[0] ** 2 + dX[1] ** 2)
    dX_particle = [traveled_distance * np.cos(p.X[2] + dX[2]),
                   traveled_distance * np.sin(p.X[2] + dX[2]),
                   dX[2]]

    p.X = list(np.array(p.X) + np.array(dX_particle))

    # normalize to 0 - 2 pi
    if p.X[2] > 2 * np.pi:
        p.X[2] = p.X[2] % (2 * np.pi)

    return p

def generate_uniform_particles(map, N):
    """
    Generate N uniformly distributed particles on the map
    """
    x_min, y_min, x_max, y_max = 100, 100, -100, -100

    for w in map.walls:
        x_min = np.min(np.append(w.x, x_min))
        x_max = np.max(np.append(w.x, x_max))
        y_min = np.min(np.append(w.y, y_min))
        y_max = np.max(np.append(w.y, y_max))

    x_min = x_min + (COLLISION_DISTANCE + 0.03)
    x_max = x_max - (COLLISION_DISTANCE + 0.03)
    y_min = y_min + (COLLISION_DISTANCE + 0.03)
    y_max = y_max - (COLLISION_DISTANCE + 0.03)

    x_rand = np.random.uniform(low = x_min, high = x_max, size = N)
    y_rand = np.random.uniform(low = y_min, high = y_max, size = N)
    theta_rand = np.random.uniform(low = 0, high = 2*np.pi, size = N)

    particles = []

    for i in range(N):
        X = [x_rand[i], y_rand[i], theta_rand[i]]
        w = 1.0 / N
        particles.append(Particle(X, w))

    return particles

def copy_particles(particles):
    """
    Copy particles function - for appending list of lists of particles
    DATA_PARTICLES = [particles_0, particles_1, ...]
    """
    new_particles = []

    for p in particles:
        new_particles.append(Particle(p.X, p.w))

    return new_particles

def fit_line(X_line, X_robot):
    """
    Fit line parameters r and alpha from X,Y points
    """
    # Transformation to robot's local coordinate frame
    R = np.array([[+np.cos(X_robot[2]), +np.sin(X_robot[2])],
                  [-np.sin(X_robot[2]), +np.cos(X_robot[2])]])

    X_line_local = np.matmul(R, X_line - np.reshape(X_robot[0:2], (2,1)))

    # line center
    line_center_x = sum(X_line_local[0]) / len(X_line_local[0])
    line_center_y = sum(X_line_local[1]) / len(X_line_local[1])

    num = 0
    denum = 0

    for i in range(len(X_line_local[0])):
        num += (X_line_local[0][i] - line_center_x) * (X_line_local[1][i] - line_center_y)
        denum += (X_line_local[1][i] - line_center_y) ** 2 - (X_line_local[0][i] - line_center_x) ** 2

	num = -2 * num
	alpha = atan2(num, denum) / 2

	r = line_center_x * cos(alpha) + line_center_y * sin(alpha)

	if r < 0:
		alpha = alpha + pi
		if alpha > pi:
			alpha = alpha - 2 * pi
		r = -r

	return (r, alpha)

def get_estimation_step(est_csv_name):
    """
    Get estimation step from csv file with name 'PARTICLES_step_i.csv', step == i
    """
    return int(est_csv_name.split('_')[-1].split('.')[0])

def transform_orientation(orientation_q):
    """
    Transform theta to [radians] from [quaternion orientation]
    """
    orientation_list = [ orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    if yaw < 0:
        yaw = 2 * np.pi + yaw  # 0->360 degrees >> 0->2pi
    return yaw

def get_odom_orientation(msgOdom):
    """"
    Get theta from Odometry msg in [radians]
    """
    orientation_q = msgOdom.pose.pose.orientation
    theta = transform_orientation(orientation_q)
    return theta

def get_odom_position(msgOdom):
    """
    Get (x,y) coordinates from Odometry msg in [m]
    """
    x = msgOdom.pose.pose.position.x
    y = msgOdom.pose.pose.position.y
    return (x, y)

def robot_set_pos(setPosPub, x, y, theta):
    """
    Set robot position and orientation
    """
    checkpoint = ModelState()

    checkpoint.model_name = 'turtlebot3_burger'

    checkpoint.pose.position.x = x
    checkpoint.pose.position.y = y
    checkpoint.pose.position.z = 0.0

    [x_q,y_q,z_q,w_q] = quaternion_from_euler(0.0,0.0,theta)

    checkpoint.pose.orientation.x = x_q
    checkpoint.pose.orientation.y = y_q
    checkpoint.pose.orientation.z = z_q
    checkpoint.pose.orientation.w = w_q

    checkpoint.twist.linear.x = 0.0
    checkpoint.twist.linear.y = 0.0
    checkpoint.twist.linear.z = 0.0

    checkpoint.twist.angular.x = 0.0
    checkpoint.twist.angular.y = 0.0
    checkpoint.twist.angular.z = 0.0

    setPosPub.publish(checkpoint)
    return (x, y, theta)

def lidar_scan(msgScan):
    """
    Convert LaserScan msg to array
    """
    distances = np.array([])
    angles = np.array([])

    for i in range(len(msgScan.ranges)):
        angle = i * msgScan.angle_increment
        if ( msgScan.ranges[i] > MAX_LIDAR_DISTANCE ):
            distance = MAX_LIDAR_DISTANCE
        elif ( msgScan.ranges[i] < msgScan.range_min ):
            distance = msgScan.range_min
            # For real robot - protection
            if msgScan.ranges[i] < 0.01:
                distance = MAX_LIDAR_DISTANCE
        else:
            distance = msgScan.ranges[i]

        distances = np.append(distances, distance)
        angles = np.append(angles, angle)

    # distances in [m], angles in [radians]
    return ( distances, angles )

def check_crash(distances):
    """
    Check for crash
    """
    if np.min(distances) <= COLLISION_DISTANCE:
        return True
    else:
        return False
