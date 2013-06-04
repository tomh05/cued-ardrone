# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rujian/fuerte_workspace/cued-ardrone/dynamics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build

# Utility rule file for ROSBUILD_gensrv_py.

# Include the progress variables for this target.
include CMakeFiles/ROSBUILD_gensrv_py.dir/progress.make

CMakeFiles/ROSBUILD_gensrv_py: ../src/dynamics/srv/__init__.py

../src/dynamics/srv/__init__.py: ../src/dynamics/srv/_FollowerImageServer.py
../src/dynamics/srv/__init__.py: ../src/dynamics/srv/_CamSelect.py
../src/dynamics/srv/__init__.py: ../src/dynamics/srv/_CaptureImageFeatures.py
../src/dynamics/srv/__init__.py: ../src/dynamics/srv/_HullSelect.py
../src/dynamics/srv/__init__.py: ../src/dynamics/srv/_capture_image_features.py
../src/dynamics/srv/__init__.py: ../src/dynamics/srv/_LedAnim.py
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/dynamics/srv/__init__.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --initpy /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/FollowerImageServer.srv /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/CamSelect.srv /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/CaptureImageFeatures.srv /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/HullSelect.srv /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/capture_image_features.srv /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/LedAnim.srv

../src/dynamics/srv/_FollowerImageServer.py: ../srv/FollowerImageServer.srv
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/std_msgs/msg/Float32MultiArray.msg
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/std_msgs/msg/MultiArrayLayout.msg
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/std_msgs/msg/MultiArrayDimension.msg
../src/dynamics/srv/_FollowerImageServer.py: ../manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/std_srvs/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /home/rujian/fuerte_workspace/ccny_vision/artoolkit/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/robot_model/resource_retriever/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/manifest.xml
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/dynamics/srv/_FollowerImageServer.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
../src/dynamics/srv/_FollowerImageServer.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/msg_gen/generated
../src/dynamics/srv/_FollowerImageServer.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/srv_gen/generated
../src/dynamics/srv/_FollowerImageServer.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/dynamics/srv/_FollowerImageServer.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/FollowerImageServer.srv

../src/dynamics/srv/_CamSelect.py: ../srv/CamSelect.srv
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/dynamics/srv/_CamSelect.py: ../manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/std_srvs/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
../src/dynamics/srv/_CamSelect.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/manifest.xml
../src/dynamics/srv/_CamSelect.py: /home/rujian/fuerte_workspace/ccny_vision/artoolkit/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/robot_model/resource_retriever/manifest.xml
../src/dynamics/srv/_CamSelect.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/manifest.xml
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/dynamics/srv/_CamSelect.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
../src/dynamics/srv/_CamSelect.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/msg_gen/generated
../src/dynamics/srv/_CamSelect.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/srv_gen/generated
../src/dynamics/srv/_CamSelect.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/dynamics/srv/_CamSelect.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/CamSelect.srv

../src/dynamics/srv/_CaptureImageFeatures.py: ../srv/CaptureImageFeatures.srv
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/std_msgs/msg/Float32MultiArray.msg
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/std_msgs/msg/MultiArrayLayout.msg
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/std_msgs/msg/MultiArrayDimension.msg
../src/dynamics/srv/_CaptureImageFeatures.py: ../manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/std_srvs/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /home/rujian/fuerte_workspace/ccny_vision/artoolkit/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/robot_model/resource_retriever/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/manifest.xml
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/dynamics/srv/_CaptureImageFeatures.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
../src/dynamics/srv/_CaptureImageFeatures.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/msg_gen/generated
../src/dynamics/srv/_CaptureImageFeatures.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/srv_gen/generated
../src/dynamics/srv/_CaptureImageFeatures.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/dynamics/srv/_CaptureImageFeatures.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/CaptureImageFeatures.srv

../src/dynamics/srv/_HullSelect.py: ../srv/HullSelect.srv
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/dynamics/srv/_HullSelect.py: ../manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/std_srvs/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
../src/dynamics/srv/_HullSelect.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/manifest.xml
../src/dynamics/srv/_HullSelect.py: /home/rujian/fuerte_workspace/ccny_vision/artoolkit/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/robot_model/resource_retriever/manifest.xml
../src/dynamics/srv/_HullSelect.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/manifest.xml
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/dynamics/srv/_HullSelect.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
../src/dynamics/srv/_HullSelect.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/msg_gen/generated
../src/dynamics/srv/_HullSelect.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/srv_gen/generated
../src/dynamics/srv/_HullSelect.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/dynamics/srv/_HullSelect.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/HullSelect.srv

../src/dynamics/srv/_capture_image_features.py: ../srv/capture_image_features.srv
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/std_msgs/msg/Float32MultiArray.msg
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/std_msgs/msg/MultiArrayLayout.msg
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/std_msgs/msg/MultiArrayDimension.msg
../src/dynamics/srv/_capture_image_features.py: ../manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/std_srvs/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /home/rujian/fuerte_workspace/ccny_vision/artoolkit/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/robot_model/resource_retriever/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/manifest.xml
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/dynamics/srv/_capture_image_features.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
../src/dynamics/srv/_capture_image_features.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/msg_gen/generated
../src/dynamics/srv/_capture_image_features.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/srv_gen/generated
../src/dynamics/srv/_capture_image_features.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/dynamics/srv/_capture_image_features.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/capture_image_features.srv

../src/dynamics/srv/_LedAnim.py: ../srv/LedAnim.srv
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/roslib/bin/gendeps
../src/dynamics/srv/_LedAnim.py: ../manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/geometry_msgs/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/sensor_msgs/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/vision_opencv/opencv2/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/roslang/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/roscpp/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/vision_opencv/cv_bridge/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/rospy/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/std_msgs/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/ros/core/rosbuild/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/roslib/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/rosconsole/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/pluginlib/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/message_filters/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/image_common/image_transport/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/std_srvs/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/bullet/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/geometry/angles/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/rostest/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/roswtf/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/geometry/tf/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/common_rosdeps/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/image_common/camera_calibration_parsers/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/image_common/camera_info_manager/manifest.xml
../src/dynamics/srv/_LedAnim.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/manifest.xml
../src/dynamics/srv/_LedAnim.py: /home/rujian/fuerte_workspace/ccny_vision/artoolkit/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/share/visualization_msgs/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/robot_model/resource_retriever/manifest.xml
../src/dynamics/srv/_LedAnim.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/manifest.xml
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/geometry/tf/msg_gen/generated
../src/dynamics/srv/_LedAnim.py: /opt/ros/fuerte/stacks/geometry/tf/srv_gen/generated
../src/dynamics/srv/_LedAnim.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/msg_gen/generated
../src/dynamics/srv/_LedAnim.py: /home/rujian/fuerte_workspace/cued-ardrone/ardrone_autonomy/srv_gen/generated
../src/dynamics/srv/_LedAnim.py: /home/rujian/fuerte_workspace/ccny_vision/ar_pose/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../src/dynamics/srv/_LedAnim.py"
	/opt/ros/fuerte/share/rospy/rosbuild/scripts/gensrv_py.py --noinitpy /home/rujian/fuerte_workspace/cued-ardrone/dynamics/srv/LedAnim.srv

ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py
ROSBUILD_gensrv_py: ../src/dynamics/srv/__init__.py
ROSBUILD_gensrv_py: ../src/dynamics/srv/_FollowerImageServer.py
ROSBUILD_gensrv_py: ../src/dynamics/srv/_CamSelect.py
ROSBUILD_gensrv_py: ../src/dynamics/srv/_CaptureImageFeatures.py
ROSBUILD_gensrv_py: ../src/dynamics/srv/_HullSelect.py
ROSBUILD_gensrv_py: ../src/dynamics/srv/_capture_image_features.py
ROSBUILD_gensrv_py: ../src/dynamics/srv/_LedAnim.py
ROSBUILD_gensrv_py: CMakeFiles/ROSBUILD_gensrv_py.dir/build.make
.PHONY : ROSBUILD_gensrv_py

# Rule to build all files generated by this target.
CMakeFiles/ROSBUILD_gensrv_py.dir/build: ROSBUILD_gensrv_py
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/build

CMakeFiles/ROSBUILD_gensrv_py.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/clean

CMakeFiles/ROSBUILD_gensrv_py.dir/depend:
	cd /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rujian/fuerte_workspace/cued-ardrone/dynamics /home/rujian/fuerte_workspace/cued-ardrone/dynamics /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build /home/rujian/fuerte_workspace/cued-ardrone/dynamics/build/CMakeFiles/ROSBUILD_gensrv_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ROSBUILD_gensrv_py.dir/depend

