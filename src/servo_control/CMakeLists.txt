cmake_minimum_required(VERSION 3.8)
project(servo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 构建可执行文件
add_executable(eyes_ctrl src/eyes_ctrl.cpp)

# 添加依赖
ament_target_dependencies(eyes_ctrl
  rclcpp
  geometry_msgs
)



# 构建可执行文件
add_executable(mouth_ctrl src/mouth_ctrl.cpp)

# 添加依赖
ament_target_dependencies(mouth_ctrl
  rclcpp
  geometry_msgs
)




# 添加依赖
ament_target_dependencies(mouth_ctrl
  rclcpp
  geometry_msgs
)


# 安装可执行文件
install(TARGETS eyes_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

# 安装可执行文件
install(TARGETS mouth_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
