# mobile-robot-localization-using-particle-filter
Implementation of Particle filter algorithm for mobile robot (turtlebot3_burger) localization in ROS.

 * Algorithm is implemented from scratch 
 * Only the map turtlebot3_stage_1.launch from turtlebot3_gazebo is handled, im working on more complex maps
 * Data collection is done by manually driving turtlebot using turtlebot3_teleop_key.launch while [pf_collect_data_node.py](scripts/pf_collect_data_node.py) is active

# Content
* [Data](Data) -> folder containing Odometry and Lidar data needed for the algorithm
* [Estimation](Data/stage_1/Estimation) -> folder containing particles data at each step
* [Maps](Maps) -> folder containing map parameters
* [scripts](scripts) -> python scripts
* [plot_map_features.py](scripts/plot_map_features.py) -> plotting the selected map with its features (obstacles)
* [pf_functions.py](scripts/pf_functions.py) -> functions needed for Particle filter algorithm and Odometry/Lidar messages processing 
* [pf_classes.py](scripts/pf_classes.py) -> classes needed for Particle filter algorithm and Map representation
* [pf_collect_data_node.py](scripts/pf_collect_data_node.py) -> node for collecting the data from Odometry and Lidar and storing in Data folder 
* [pf_algo.py](scripts/pf_algo.py) -> calculating particles at each step using Data folder and storing results in Data/Estimation folder or just plotting the results directly from Data/Estimation folder
