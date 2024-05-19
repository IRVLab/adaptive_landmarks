cmake_minimum_required(VERSION 3.0.2)
project(adaptive_landmarks)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    message_generation
)

add_message_files(
    FILES
    Rgb.msg
    Markers.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
    CATKIN_DEPENDS rospy sensor_msgs std_msgs message_runtime cv_bridge image_transport
)