# Taken from http://www.ros.org/wiki/StackBuildFiles
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_make_distribution(0.2.0)
