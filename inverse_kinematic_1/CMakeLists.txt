cmake_minimum_required(VERSION 3.0.2)
project(inverse_kinematic_1)

find_package(catkin REQUIRED roscpp std_msgs sensor_msgs geometry_msgs)



catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_test src/ur3_gazebo_inverse_1.cpp)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test ${catkin_LIBRARIES})
