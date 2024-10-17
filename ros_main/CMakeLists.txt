cmake_minimum_required(VERSION 3.0.2)
project(ros_main)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs 
  pcl_conversions pcl_ros 
  cv_bridge image_transport tf 
  sensor_msgs geometry_msgs std_msgs visualization_msgs 
  image_geometry message_generation
  move_base_msgs actionlib
  wpb_home_bringup wpb_home_tutorials wpb_home_behaviors
  xfyun_waterplus waterplus_map_tools
)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES wpb_home_apps
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


add_executable(entrance_detect
  src/entrance_detect.cpp
)
add_dependencies(entrance_detect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(entrance_detect
  ${catkin_LIBRARIES}
)
add_executable(main
  src/main.cpp
  src/Robot_Act/RobotAct.cpp
)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main
  ${catkin_LIBRARIES}
)

