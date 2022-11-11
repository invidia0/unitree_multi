cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++11)
project(multi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS src
   CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

add_executable(rrt_star src/rrt_star_node.cpp src/rrt_star.cpp)
target_link_libraries(rrt_star ${catkin_LIBRARIES})
add_dependencies(rrt_star ${catkin_EXPORTED_TARGETS})

add_executable(apf src/apf_node.cpp src/apf.cpp)
target_link_libraries(apf ${catkin_LIBRARIES})
add_dependencies(apf ${catkin_EXPORTED_TARGETS})

add_executable(nav src/nav.cpp)
target_link_libraries(nav ${catkin_LIBRARIES})
add_dependencies(nav ${catkin_EXPORTED_TARGETS})

add_executable(world_br src/world_br.cpp)
target_link_libraries(world_br ${catkin_LIBRARIES})
add_dependencies(world_br ${catkin_EXPORTED_TARGETS})

add_executable(obstacles_generator src/obstacles_generator.cpp)
target_link_libraries(obstacles_generator ${catkin_LIBRARIES})
add_dependencies(obstacles_generator ${catkin_EXPORTED_TARGETS})


install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})