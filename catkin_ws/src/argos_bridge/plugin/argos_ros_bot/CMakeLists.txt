
add_library(argos_ros_bot MODULE argos_ros_bot.h argos_ros_bot.cpp)
target_link_libraries(argos_ros_bot
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)


add_library(eyebot_flocking MODULE eyebot_flocking.h eyebot_flocking.cpp)
target_link_libraries(eyebot_flocking
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)