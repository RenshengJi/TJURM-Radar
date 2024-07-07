cmake_minimum_required(VERSION 3.0.2)
project(radar_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        message_generation
        message_runtime
        sensor_msgs
        std_msgs
        )

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        point.msg
        points.msg
        dist_point.msg
        dist_points.msg
        yolo_point.msg
        yolo_points.msg
        game_state.msg
        referee_warning.msg
        supply_projectile_action.msg
        small_map.msg
        world_point.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
        DEPENDS
)
