cmake_minimum_required(VERSION 3.8)
project(audio_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ALSA REQUIRED)


# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  DEPENDENCIES std_msgs
)


add_executable(video src/video.cpp)
add_executable(video2 src/video2.cpp)
add_executable(show src/show.cpp)
add_executable(audio_recorder src/record.cpp)
add_executable(audio_player src/play.cpp)

ament_target_dependencies(video
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  std_msgs
)


ament_target_dependencies(video2
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  std_msgs
)

ament_target_dependencies(show
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  std_msgs
)

ament_target_dependencies(audio_recorder
  rclcpp
  std_msgs
)

rosidl_target_interfaces(audio_recorder
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(audio_recorder
  ${ALSA_LIBRARIES}
)

target_include_directories(audio_recorder
  PRIVATE
  ${ALSA_INCLUDE_DIRS}
)

ament_target_dependencies(audio_player
  rclcpp
  std_msgs
)

rosidl_target_interfaces(audio_player
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(audio_player
  ${ALSA_LIBRARIES}
)

target_include_directories(audio_player
  PRIVATE
  ${ALSA_INCLUDE_DIRS}
)


install(TARGETS
  
  video2
  show
  
  audio_recorder
  audio_player
  DESTINATION lib/${PROJECT_NAME})



# 安装Python脚本并设置为可执行，重命名为video_angle
install(PROGRAMS
  scripts/video_angle.py
  DESTINATION lib/${PROJECT_NAME}

)

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

ament_package()
