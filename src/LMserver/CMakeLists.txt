cmake_minimum_required(VERSION 3.8)
project(LMserver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(audio_capture REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# WebRTC VAD library
set(WEBRTC_VAD_INCLUDE_DIRS /home/orangepi/ros/WebRTC_VAD/vad/include)
set(WEBRTC_VAD_LIBRARIES /home/orangepi/ros/WebRTC_VAD/build/libvad.so)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(vad_node src/vad_node.cpp)
target_include_directories(vad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${WEBRTC_VAD_INCLUDE_DIRS})

set_target_properties(vad_node PROPERTIES
  BUILD_RPATH "/home/orangepi/ros/WebRTC_VAD/build"
  INSTALL_RPATH "/home/orangepi/ros/WebRTC_VAD/build"
)

ament_target_dependencies(vad_node
  rclcpp
  std_msgs
  audio_capture
  geometry_msgs
)

target_link_libraries(vad_node
  ${WEBRTC_VAD_LIBRARIES}
)



add_executable(play src/play.cpp)
target_include_directories(play PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(play
  rclcpp
  std_msgs
  audio_capture
)

add_executable(speaker src/speaker.cpp)
target_include_directories(speaker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(speaker
  rclcpp
  std_msgs
)

# Link ALSA library
target_link_libraries(speaker
  asound
)

# Add playtest executable
add_executable(playtest src/playtest.cpp)
target_include_directories(playtest PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(playtest
  rclcpp
  std_msgs
)

install(TARGETS vad_node play speaker playtest
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# Install Python packages
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  LMserver/omni_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
