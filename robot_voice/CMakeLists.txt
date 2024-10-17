cmake_minimum_required(VERSION 3.0.2)
project(robot_voice)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  pcl_conversions pcl_ros 
  cv_bridge image_transport tf 
  sensor_msgs geometry_msgs std_msgs visualization_msgs 
  image_geometry message_generation
  move_base_msgs actionlib
  wpb_home_bringup wpb_home_tutorials wpb_home_behaviors
  xfyun_waterplus waterplus_map_tools
)
add_service_files(
  FILES
  StringToVoice.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(voice_detector 
  src/voice_detector.cpp
  ifly_voice/speech_recognizer.c
  ifly_voice/linuxrec.c)

add_executable(robot_controller src/robot_controller.cpp)
add_executable(voice_creator src/voice_creator.cpp)

add_dependencies(voice_detector ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(voice_detector
  ${catkin_LIBRARIES} 
  libmsc.so -ldl -lpthread -lm -lrt -lasound
)
add_dependencies(robot_controller ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(robot_controller
  ${catkin_LIBRARIES} 
)
add_dependencies(voice_creator ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(voice_creator
  ${catkin_LIBRARIES} 
  libmsc.so -ldl -pthread
)

