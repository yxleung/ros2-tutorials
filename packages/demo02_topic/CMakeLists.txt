cmake_minimum_required(VERSION 3.8)
project(demo02_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(demo01_interface REQUIRED)

add_executable(talker_str src/talker_str.cpp)
add_executable(listener_str src/listener_str.cpp)
add_executable(talker_stu src/talker_stu.cpp)
add_executable(listener_stu src/listener_stu.cpp)

target_include_directories(talker_str PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(talker_str PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  talker_str
  "rclcpp"
  "std_msgs"
)
ament_target_dependencies(
  listener_str
  "rclcpp"
  "std_msgs"
)
ament_target_dependencies(
  talker_stu
  "rclcpp"
  "std_msgs"
  "demo01_interface"
)
ament_target_dependencies(
  listener_stu
  "rclcpp"
  "std_msgs"
  "demo01_interface"
)

install(TARGETS 
  talker_str 
  listener_str 
  talker_stu 
  listener_stu
  DESTINATION lib/${PROJECT_NAME})

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
