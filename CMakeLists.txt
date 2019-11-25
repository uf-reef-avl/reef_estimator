cmake_minimum_required(VERSION 2.8.3)
project(reef_estimator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -I /usr/include/eigen3")
#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    rosflight_msgs
    sensor_msgs
    std_msgs
    tf2_eigen
    tf_conversions
    reef_msgs
)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs rosflight rosflight_msgs sensor_msgs std_msgs tf2_eigen
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
    reef_estimator
    src/reef_estimator_node.cpp
    src/sensor_manager.cpp
    src/estimator.cpp
    src/z_estimator.cpp
    src/xyz_estimator.cpp include/xyz_estimator.h
    src/xy_estimator.cpp
)

add_dependencies(reef_estimator ${catkin_EXPORTED_TARGETS})
target_link_libraries(reef_estimator ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/verify_estimates.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS reef_estimator
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
