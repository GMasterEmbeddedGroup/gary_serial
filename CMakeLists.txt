cmake_minimum_required(VERSION 3.5)
project(gary_serial)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(dr16_receiver
        src/dr16_receiver.cpp)

add_executable(rm_referee
        src/utils/crc8_crc16.cpp
        src/rm_referee.cpp)

target_link_libraries(rm_referee serial)

target_include_directories(dr16_receiver PRIVATE include)
target_include_directories(rm_referee PRIVATE include)

ament_target_dependencies(
        dr16_receiver
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)
ament_target_dependencies(
        rm_referee
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)

add_library(dr16_receiver_lib SHARED
        src/dr16_receiver.cpp)

add_library(rm_referee_lib SHARED
        src/utils/crc8_crc16.cpp
        src/rm_referee.cpp)

target_link_libraries(rm_referee_lib serial)

target_include_directories(dr16_receiver_lib PRIVATE include)
target_include_directories(rm_referee_lib PRIVATE include)

ament_target_dependencies(
        dr16_receiver_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)
ament_target_dependencies(
        rm_referee_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)

rclcpp_components_register_nodes(dr16_receiver_lib "gary_serial::DR16Receiver")
rclcpp_components_register_nodes(rm_referee_lib "gary_serial::RMReferee")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS dr16_receiver rm_referee
        DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS dr16_receiver_lib rm_referee_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_dependencies(
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        lifecycle_msgs
        gary_msgs
        diagnostic_msgs
)

ament_package()
