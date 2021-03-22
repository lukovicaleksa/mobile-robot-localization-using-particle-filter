cmake_minimum_required(VERSION 3.0.2)
project(pf_localization)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs )

## Declare a catkin package
catkin_package()

# Install python scripts
catkin_install_python(PROGRAMS scripts/pf_algo.py
				scripts/pf_classes.py
				scripts/pf_functions.py
				scripts/pf_collect_data_node.py
				scripts/plot_map_features.py
				scripts/scan_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
