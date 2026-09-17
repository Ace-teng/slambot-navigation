cmake_minimum_required(VERSION 3.8)
project(landerpi_gazebo_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds config maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/initial_pose_publisher.py
  scripts/staged_navigation_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
