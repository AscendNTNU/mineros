cmake_minimum_required(VERSION 3.8)
project(vnode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Messages
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mineros_inter REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "../mineros_interface/srv/BotPos.srv"
	"../mineros_interface/srv/FindBlocks.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(
  launch
  src/launch.cpp
)
ament_target_dependencies(launch rclcpp geometry_msgs mineros_inter)
ament_export_dependencies(rosidl_default_runtime)

install(
	TARGETS launch
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
