##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(displayer_qt5)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
##############################################################################
# Catkin
##############################################################################

set(cv_bridge_DIR /usr/local/cv_bridge4.5.4/share/cv_bridge/cmake)

# 注意，要找到正确的opencv包需要这么写！！
find_package(OpenCV 4.5.4 REQUIRED)
# qt_build provides the qt cmake glue, roscpp the comms for a default talker
# set(OpenCV_DIR /usr/share/OpenCV/)
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    sensor_msgs
    std_msgs
    radar_msgs
    )
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
include_directories(
    ${catkin_INCLUDE_DIRS}
    include
    )
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/displayer_qt5/*.hpp *.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(displayer_qt5 ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(displayer_qt5 radar_msgs_generate_messages)
target_link_libraries(displayer_qt5 ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS displayer_qt5 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(displayer_qt5
    /usr/lib/x86_64-linux-gnu/libtiff.so.5
    /usr/lib/x86_64-linux-gnu/libsqlite3.so.0
    )
message(STATUS "OpenCV_INCLUDE_DIRS ?" : ${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_VERSION ?" : ${OpenCV_VERSION})
message(STATUS "OpenCV_LIB ?" : ${OpenCV_LIBS})
message(STATUS "OpenCV_CONGIG_PATH ?" : ${OpenCV_CONFIG_PATH})

add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
