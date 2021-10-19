cmake_minimum_required(VERSION 2.8.3)
project(pixhawk_fsm)
add_compile_options(-std=c++14)

#########################################################################################

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        std_msgs
        visualization_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
        message_generation
)

add_message_files(
        FILES
        PixhawkStatus.msg 
        Path.msg
)

add_service_files(
        FILES
        OperationCompletion.srv
        Explore.srv
        Land.srv
        TakeOff.srv
        ObstacleAvoidanceService.srv
        Travel.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime roscpp 
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

add_executable(pixhawk_fsm      src/nodes/main.cpp      ${SRC} ${Operations_SRC})
add_executable(keyboard_client  src/client/keyboard_client.cpp        ${SRC} ${Operations_SRC})
add_executable(path_client  src/client/path_client.cpp        ${SRC} ${Operations_SRC})

add_dependencies(pixhawk_fsm    ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(keyboard_client        ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})
add_dependencies(path_client        ${${PROJECT_NAME}_EXPORTED_TARGETS}     ${catkin_EXPORTED_TARGETS})

target_link_libraries(pixhawk_fsm       ${catkin_LIBRARIES})
target_link_libraries(keyboard_client   ${catkin_LIBRARIES})
target_link_libraries(path_client   ${catkin_LIBRARIES})