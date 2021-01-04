# mobile-robot-localization-using-Particle-filter
Implementation of Particle filter algorithm for mobile robot localization (turtlebot3_burger) in ROS.

Content:

    Data -> folder containing Odometry and Lidar data needed for the algorithm
    
    Data/Estimation -> folder containing particles data at each step
    
    Maps -> folder containing map parameters
    
    plot_map_features.py -> plotting the selected map with its features (obstacles)
    
    pf_functions.py -> functions needed for Particle filter algorithm and Odometry/Lidar messages processing 
    
    pf_classes.py -> classes needed for Particle filter algorithm and Map representation
    
    pf_collect_data_node.py -> node for collecting the data from Odometry and Lidar and storing in Data folder 
    
    pf_algo.py -> calculating particles at each step using Data folder and storing results in Data/Estimation folder or just plotting the results directly from Data/Estimation folder
    
    
    
