cmake_minimum_required(VERSION 3.0.2)
project(a1_joystick)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS src
   CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS}
                    include)

add_executable(converter src/converter.cpp)
target_link_libraries(converter ${catkin_LIBRARIES})
add_dependencies(converter ${catkin_EXPORTED_TARGETS})

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})