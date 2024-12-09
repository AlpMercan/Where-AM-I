cmake_minimum_required(VERSION 3.0.2)
project(where_am_i)

# Add C++17 support
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  cv_bridge
  nav_msgs
  rospy
  std_msgs
  tf2_ros
)
find_package(OpenCV REQUIRED)
find_package(OpenMP)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES where_am_i
  CATKIN_DEPENDS cv_bridge nav_msgs rospy std_msgs geometry_msgs roscpp tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Existing executables
add_executable(anomaly_detector_node_cpp src/anomaly_detection.cpp)
target_link_libraries(anomaly_detector_node_cpp 
  ${catkin_LIBRARIES}
)


# Add the new localization node
add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  stdc++fs
)

# Python scripts installation
catkin_install_python(PROGRAMS
    src/anomaly_detection.py
    src/The_kidnapper_Algorithm.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
