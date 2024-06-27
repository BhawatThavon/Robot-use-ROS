cmake_minimum_required(VERSION 3.5)
project(project_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# No need to add an executable if you don't have a source file
# add_executable(robot_node src/robot.cpp)  # Remove this line

# No need to link any libraries if there's no executable
# target_link_libraries(robot_node ${catkin_LIBRARIES})  # Remove this line

