cmake_minimum_required(VERSION 3.8)
project(coord_conversion_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(msg_interfaces REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(converter src/converter.cpp)
target_include_directories(converter 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(converter PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(converter rclcpp std_msgs sensor_msgs Eigen3 msg_interfaces)

add_executable(converter_livox src/converter_livox.cpp)
ament_target_dependencies(converter_livox rclcpp msg_interfaces sensor_msgs livox_ros_driver2)

install(TARGETS
  converter 
  converter_livox
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
