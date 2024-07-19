cmake_minimum_required(VERSION 3.0.2)
project(driver_bridge)

add_compile_options(-g -O0)

set(cv_bridge_DIR /usr/local/cv_bridge4.5.4/share/cv_bridge/cmake)
# set(OpenCV_DIR /usr/local/opencv4.5.4/lib/cmake/opencv4)


# 注意，要找到正确的opencv包需要这么写！！
find_package(OpenCV 4.5.4 EXACT)

# 输出opencv的版本
# message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(PCL REQUIRED)
find_package(OpenRM REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  cv_bridge
  sensor_msgs
  radar_msgs
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
  ${CMAKE_SOURCE_DIR}/driver_bridge/include/driver_bridge
  ${OpenRM_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/../../libs/json/single_include/nlohmann
)


# 2.添加可执行文件
add_executable(driver_bridge_node src/driver_bridge.cc)

# 5. 添加依赖项
add_dependencies(driver_bridge_node radar_msgs_generate_messages)
add_dependencies(driver_bridge_node ${PROJECT_NAME}_generate_messages_cpp)


# 3.添加构建子目录
add_subdirectory(src/data_manager)

# 4.链接
target_link_libraries(driver_bridge_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  -lpthread
  data_manager
)




