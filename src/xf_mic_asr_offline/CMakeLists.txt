
cmake_minimum_required(VERSION 3.8)
project(xf_mic_asr_offline)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xf_mic_asr_offline_msgs REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)

include_directories(
  include
)

# The vendor offline_record_lib / lib/arm64 tree is delivered separately by the
# hardware vendor and is not committed to this repository. Link against it only
# when it is present so that a clean `colcon build` does not fail on its absence.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib")
  link_directories(
    lib/arm64
  )
  set(OFFLINE_RECORD_LIBS offline_record_lib)
else()
  set(OFFLINE_RECORD_LIBS)
endif()

# Create the executable target
add_executable(voice_control src/voice_control.cpp)
ament_target_dependencies(voice_control rclcpp std_msgs xf_mic_asr_offline_msgs ros_robot_controller_msgs)
target_link_libraries(voice_control ${OFFLINE_RECORD_LIBS} rt dl pthread stdc++ asound)
install(TARGETS voice_control
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/awake_node.py
  scripts/asr_node.py
  scripts/voice_control_move.py
  scripts/voice_control_navigation.py
  scripts/voice_control_arm.py
  scripts/voice_control_color_detect.py
  scripts/voice_control_color_sorting.py
  scripts/voice_control_color_track.py
  scripts/voice_control_garbage_classification.py
  scripts/voice_control_navigation_transport.py
  DESTINATION lib/${PROJECT_NAME}
)

# `feedback_voice` is also a vendor asset and may be absent; install it if present.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/feedback_voice")
  install(DIRECTORY launch config feedback_voice
    DESTINATION share/${PROJECT_NAME})
else()
  install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})
endif()

# Add the test directories if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the ROS 2 package
ament_package()

