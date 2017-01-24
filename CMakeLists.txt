cmake_minimum_required(VERSION 2.8.3)
project(garmin_lidar_driver)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(-std=c++11)
############
# BINARIES #
############
cs_add_executable(garmin_lidar_node
			src/garmin_lidar_node.cpp
        )
##########
# EXPORT #
##########
cs_install()
cs_export()
