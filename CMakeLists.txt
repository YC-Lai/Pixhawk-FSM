cmake_minimum_required(VERSION 2.8.3)
project(pixhawk_fsm)
add_compile_options(-std=c++14)

#########################################################################################

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        ascend_msgs
        std_msgs
        visualization_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
        message_generation
)

add_service_files(
        FILES
        OperationCompletion.srv
        Explore.srv
        Land.srv
        TakeOff.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime ascend_msgs roscpp 
)

#########################################################################################

include_directories(include
        include/pixhawk_fsm
        include/pixhawk_fsm/operations
        ${catkin_INCLUDE_DIRS}
)

file(GLOB SRC "src/*.cpp")
file(GLOB Operations_SRC "src/operations/*.cpp")

#########################################################################################
