cmake_minimum_required(VERSION 3.0.2)
project(multi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
)

catkin_package(
   INCLUDE_DIRS src
   CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf
)

include_directories(${catkin_INCLUDE_DIRS}
                    include)

add_executable(nav src/nav.cpp)
target_link_libraries(nav ${catkin_LIBRARIES})
add_dependencies(nav ${catkin_EXPORTED_TARGETS})

add_executable(world_br src/world_br.cpp)
target_link_libraries(world_br ${catkin_LIBRARIES})
add_dependencies(world_br ${catkin_EXPORTED_TARGETS})

add_executable(path_visualizer src/path_visualizer.cpp)
target_link_libraries(path_visualizer ${catkin_LIBRARIES})
add_dependencies(path_visualizer ${catkin_EXPORTED_TARGETS})

add_executable(obstacles_generator src/obstacles_generator.cpp)
target_link_libraries(obstacles_generator ${catkin_LIBRARIES})
add_dependencies(obstacles_generator ${catkin_EXPORTED_TARGETS})

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})
add_dependencies(planner ${catkin_EXPORTED_TARGETS})


install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})