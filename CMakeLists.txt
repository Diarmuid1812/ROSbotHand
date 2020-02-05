cmake_minimum_required(VERSION 2.8.3)
project(rosbotic_hand)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        message_generation
        roslaunch
        )

add_message_files(
        DIRECTORY msg
        FILES zeros.msg grip.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(ADC_node_cpp src/ADC_Node.cpp)
catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy)

target_link_libraries(ADC_node_cpp
        ${catkin_LIBRARIES}
        )
