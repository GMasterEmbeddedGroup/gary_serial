cmake_minimum_required(VERSION 3.8)
project(gary_contact)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)


add_executable(auto_aim_contact src/auto_aim_contact.cpp)
target_link_libraries(auto_aim_contact serial)
target_include_directories(auto_aim_contact PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(auto_aim_contact PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  auto_aim_contact
  "rclcpp"
  "gary_msgs"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
)

install(TARGETS auto_aim_contact
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
