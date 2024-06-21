cmake_minimum_required(VERSION 3.0.2)
project(driver_bridge)

add_compile_options(-g)

set(cv_bridge_DIR /usr/local/cv_bridge4.5.4/share/cv_bridge/cmake)
find_package(OpenCV 4.5.4 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  sensor_msgs
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


# 1. 添加头文件
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/driver_bridge/include
)


# 2.添加可执行文件
add_executable(driver_bridge_node src/driver_bridge.cc)


# 3.链接
target_link_libraries(driver_bridge_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# 4. 添加依赖项
add_dependencies(driver_bridge_node ${PROJECT_NAME}_generate_messages_cpp)



