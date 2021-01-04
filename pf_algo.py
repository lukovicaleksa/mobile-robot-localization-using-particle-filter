#! /usr/bin/env python

import rospy
import os

from time import clock
from time import sleep
from datetime import datetime
import matplotlib.pyplot as plt

from pf_functions import *

DATA_DIR = 'Data'
MAP_NAME = 'stage_1'

LOAD_EST_DATA = True # True - load data from DATA_DIR/Estimation

if __name__ == '__main__':
    try:
        # Map configuration and object extraction
        map = Map(MAP_NAME)
        map.load_objects()

        # Load data
        DATA_X_odom = np.genfromtxt(DATA_DIR + '/X_odom.csv', delimiter = ' , ')
        DATA_Y_odom = np.genfromtxt(DATA_DIR + '/Y_odom.csv', delimiter = ' , ')
        DATA_THETA_odom = np.genfromtxt(DATA_DIR + '/THETA_odom.csv', delimiter = ' , ')
        DATA_DISTANCES_lidar = np.genfromtxt(DATA_DIR + '/DISTANCES_lidar.csv', delimiter = ' , ')
        DATA_ANGLES_lidar = np.genfromtxt(DATA_DIR + '/ANGLES_lidar.csv', delimiter = ' , ')

        print('\r\nLoaded data:')
        print('X_odom: ' + str(DATA_X_odom.shape) + ' ' + str(type(DATA_X_odom)))
        print('Y_odom: ' + str(DATA_Y_odom.shape) + ' ' + str(type(DATA_Y_odom)))
        print('THETA_odom: ' + str(DATA_THETA_odom.shape) + ' ' + str(type(DATA_THETA_odom)))
        print('DISTANCES_lidar: ' + str(DATA_DISTANCES_lidar.shape) + ' ' + str(type(DATA_DISTANCES_lidar)))
        print('ANGLES_lidar: ' + str(DATA_ANGLES_lidar.shape) + ' ' + str(type(DATA_ANGLES_lidar)))

        # Init storing data
        DATA_PARTICLES = []

        # Remembering init values
        x_odom_prev = DATA_X_odom[0]
        y_odom_prev = DATA_Y_odom[0]
        theta_odom_prev = DATA_THETA_odom[0]
        distances = DATA_DISTANCES_lidar[0]
        angles = DATA_ANGLES_lidar[0]

        # Init time
        t_start = clock()

        if not LOAD_EST_DATA:
            #######################################
            ### PARTICLE FILTER INITIALIZATION ###
            #####################################

            N = 200  # Number of particles

            # Particle filter algorithm - Create N uniformly distributed particles
            particles = generate_uniform_particles(map, N)

            for p in particles:
                # Calculate particle weights: w_t = p(z_t|x_t)
                p = calculate_particle_weight(p, angles, distances, map)

            # Normalize particle weight
            particles = normalize_particle_weights(particles)

            print('\r\nInitial particles: ')
            for (i, p) in enumerate(particles):
                print(str(i + 1) + '. (x,y,theta) = (%.2f,%.2f,%.2f), weight = %.5f' % (p.X[0], p.X[1], p.X[2], p.w))

            ##################################
            ### PARTICLE FILTER MAIN LOOP ###
            ################################
            for i in range(1, len(DATA_X_odom)):
                # Evaluate Particle Filter algorithm on DATA_DIR

                print('\r\nStep {} calculating...\r\n'.format(i))

                # Calculate step time in [s]
                step_time = clock()
                step_duration = step_time - t_start
                t_start = step_time

                # LIDAR measurements
                distances = DATA_DISTANCES_lidar[i]     # distances in [m]
                angles = DATA_ANGLES_lidar[i]           # angles in [radians]

                # Odometry measurements
                x_odom = DATA_X_odom[i]          # x coordinate in [m]
                y_odom = DATA_Y_odom[i]          # y coordinate in [m]
                theta_odom = DATA_THETA_odom[i]  # theta coordinate in [radians]

                X_odom = np.array([x_odom, y_odom, theta_odom])
                X_odom_prev = np.array([x_odom_prev, y_odom_prev, theta_odom_prev])
                dX = X_odom - X_odom_prev

                # Resample particles from distribution based on weights w_t
                particles = resample_particles(particles, map)

                for p in particles:
                    # State transition model: p(x_t|x_t_1, u_t)
                    p = apply_state_transision(p, dX)

                    # Calculate particle weights: w_t = p(z_t|x_t)
                    p = calculate_particle_weight(p, angles, distances, map)

                # Normalize particle weight
                particles = normalize_particle_weights(particles)

                # Sorting particles by weights
                particles.sort(key = Particle.get_weight, reverse = True)
                # Top particle - maximum likelihood estimate
                p_mle = particles[0]

                print('\r\nStep {}, step time: {} [s]'.format(i, step_duration))

                print('Particles: ')
                for (i, p) in enumerate(particles):
                    print(str(i + 1) + '. (x,y,theta) = (%.2f,%.2f,%.2f), weight = %.5f' % (p.X[0], p.X[1], p.X[2], p.w))

                x_odom_prev = x_odom
                y_odom_prev = y_odom
                theta_odom_prev = theta_odom

                # DATA_PARTICLES.append(particles) overwrites all existing elements in list
                DATA_PARTICLES.append(copy_particles(particles))
        else:
            """
            Load estimated data from file
            """
            DATA_PARTICLES = []
            estimations = os.listdir(DATA_DIR + '/Estimation')
            estimations.sort(key = get_estimation_step, reverse = False)

            for est in estimations:
                data_particles = np.genfromtxt(DATA_DIR + '/Estimation/' + est, delimiter = ' , ')
                particles = []

                for dp in data_particles:
                    p_X = [dp[0], dp[1], dp[2]]
                    p_w = dp[3]
                    particles.append(Particle(p_X, p_w))

                DATA_PARTICLES.append(particles)


        algo_time_s = clock()
        algo_time_h = algo_time_s // 3600
        algo_time_m = (algo_time_s - algo_time_h * 3600) // 60
        algo_time_s = algo_time_s - algo_time_h * 3600 - algo_time_m * 60

        print('\r\nAlgorithm duration: %i:%i:%.2f [h:m:s]\r\n' % (algo_time_h, algo_time_m, algo_time_s))

        # Log the data
        if not LOAD_EST_DATA:
            for (i, particles) in enumerate(DATA_PARTICLES):

                PARTICLES = np.empty([len(particles), 4])
                file_name = 'PARTICLES_step_{}.csv'.format(i)

                for (j,p) in enumerate(particles):
                    PARTICLES[j,0] = p.X[0]
                    PARTICLES[j,1] = p.X[1]
                    PARTICLES[j,2] = p.X[2]
                    PARTICLES[j,3] = p.w

                np.savetxt(DATA_DIR + '/Estimation/' + file_name, PARTICLES, delimiter = ' , ')

        # Init figure - simulated real time
        plt.style.use('seaborn-ticks')
        fig = plt.figure(1)
        ax = fig.add_subplot(1,1,1)
        box = ax.get_position()
        ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

        print('Plotting\r\n')

        for (i, particles) in enumerate(DATA_PARTICLES):
            print('Plotting step {}'.format(i))

            # LIDAR measurements
            distances = DATA_DISTANCES_lidar[i]     # distances in [m]
            angles = DATA_ANGLES_lidar[i]           # angles in [radians]

            # Sorting particles by weights
            particles.sort(key = Particle.get_weight, reverse = True)
            # Top particle - maximum likelihood estimate
            p_mle = particles[0]

            # Lidar measurements of MLE in x-y plane
            lidar_p_mle_x = np.array([])
            lidar_p_mle_y = np.array([])

            for (dist, ang) in zip(distances, angles):
                lidar_p_mle_x = np.append(lidar_p_mle_x, p_mle.X[0] + dist * np.cos(ang + p_mle.X[2]))
                lidar_p_mle_y = np.append(lidar_p_mle_y, p_mle.X[1] + dist * np.sin(ang + p_mle.X[2]))

            # Plot
            ax.clear()

            # objects
            for w in map.walls:
                ax.plot(w.x, w.y, color = 'black', lw = 4)

            # particles
            for (j,p) in enumerate(particles):

                dir_p_x = np.array([p.X[0], p.X[0] + 0.1 * np.cos(p.X[2])])
                dir_p_y = np.array([p.X[1], p.X[1] + 0.1 * np.sin(p.X[2])])

                if j == 0:
                    # MLE
                    ax.scatter(p.X[0], p.X[1], color = 'green', marker = 'o', lw = 7, label = 'MLE')
                    ax.plot(dir_p_x, dir_p_y, color = 'green', lw = 4)
                elif j == 1:
                    ax.scatter(p.X[0], p.X[1], color = 'blue', marker = 'o', lw = 1, label = 'particles')
                    ax.plot(dir_p_x, dir_p_y, color = 'blue', lw = 0.75)
                else:
                    ax.scatter(p.X[0], p.X[1], color = 'blue', marker = 'o', lw = 1)
                    ax.plot(dir_p_x, dir_p_y, color = 'blue', lw = 0.75)

            # LIDAR measurements of MLE
            ax.plot(lidar_p_mle_x, lidar_p_mle_y, 'r.', markersize = 4, label = 'lidar')

            # plotting MLE again to get in front
            dir_p_mle_x = np.array([p_mle.X[0], p_mle.X[0] + 0.12 * np.cos(p_mle.X[2])])
            dir_p_mle_y = np.array([p_mle.X[1], p_mle.X[1] + 0.12 * np.sin(p_mle.X[2])])

            ax.scatter(p_mle.X[0], p_mle.X[1], color = 'green', marker = 'o', lw = 7)
            ax.plot(dir_p_mle_x, dir_p_mle_y, color = 'green', lw = 4)

            plt.xlabel('x[m]')
            plt.ylabel('y[m]')
            plt.xlim((-2.2, 2.2))
            plt.ylim((-2.5, 2.2))
            plt.title('Particle filter algorithm')
            ax.legend(loc = 'center left', bbox_to_anchor = (1, 0.5))
            plt.draw()
            plt.pause(0.0001)

            if i == 0:
                sleep(3)

            if i == (len(DATA_PARTICLES) - 1):
                print('End of plotting')
                sleep(10)

    except rospy.ROSInterruptException:
        print 'Simulation terminated!'
        pass
