cmake_minimum_required(VERSION 3.0.2)
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
    sensor_msgs
    vision_msgs
    dynamic_reconfigure
)

# NOTE: Used for dynamic reconfiguration of the mapping system
generate_dynamic_reconfigure_options(
  cfg/Mapping.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
)

include_directories(${catkin_INCLUDE_DIRS})

# catkin_package(INCLUDE_DIRS include)

# cmake_host_system_information(RESULT _host_name QUERY HOSTNAME)

# target_link_libraries(${catkin_LIBRARIES})
# add_dependencies(${catkin_EXPORTED_TARGETS})