cmake_minimum_required(VERSION 2.8.3)
project(riptide_mapping)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
add_compile_options(-std=c++11)

# NOTE: Most dependencies can be taken care of with $(catkin_EXPORTED_TARGETS).
# There is no need to add mention the specific dependency, such as
# <package_name>_gencpp, or <package_name>_gencfg

# NOTE: To include OpenCV, must do find_package(OpenCV REQUIRED), followed by
# include_directories(...)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
    roscpp
    roslint
    riptide_msgs
    sensor_msgs
    vision_msgs
)

# catkin_package(INCLUDE_DIRS include)

# cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)

# target_link_libraries(${catkin_LIBRARIES})
# add_dependencies(${catkin_EXPORTED_TARGETS})