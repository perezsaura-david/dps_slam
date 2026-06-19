cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME dual_pose_graph)
project(${PROJECT_NAME} VERSION 1.0.0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set Release as default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_BUILD_TYPE Release)


# Pre-find GeographicLib using CONFIG mode to avoid system FindModule issues
find_package(GeographicLib CONFIG QUIET)

# find dependencies
set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  as2_core
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  fmt
  backward_ros
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

# dual_pose_graph brings g2o, glog, ceres and Eigen transitively.
find_package(dual_pose_graph CONFIG REQUIRED)
#FMT
set(FMT_LIBRARIES fmt::fmt)

# Plugins
# set(PLUGIN_LIST
#   raw_odometry
#   ground_truth
#   mocap_pose
# )

# Create a list of all the plugin files
# foreach(PLUGINS ${PLUGIN_LIST})
#   list(APPEND PLUGINS_CPP_FILES
#     plugins/${PLUGINS}/src/${PLUGINS}.cpp
#   )

#   include_directories(
#     plugins/${PLUGINS}/include
#     plugins/${PLUGINS}/include/${PLUGINS}
#   )
# endforeach()

# Library
# add_library(${PROJECT_NAME} SHARED ${PLUGINS_CPP_FILES})

# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
# pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# ament_export_include_directories(
#   include
# )
# ament_export_libraries(
#   ${PROJECT_NAME}
# )
# ament_export_targets(
#   export_${PROJECT_NAME}
# )

set(SOURCE_CPP_FILES
  src/semantic_slam_node.cpp
  src/semantic_slam.cpp
  src/conversions.cpp
  src/debug_utils.cpp)

# Executable
add_executable(${PROJECT_NAME}_node ${SOURCE_CPP_FILES})
# add_executable(${PROJECT_NAME}_node src/semantic_slam_node.cpp src/semantic_slam.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node
                      dual_pose_graph::dual_pose_graph
                      ${FMT_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# For each plugin, install config files
# foreach(PLUGINS ${PLUGIN_LIST})
#   if(EXISTS plugins/${PLUGINS}/config)
#     install(
#       DIRECTORY plugins/${PLUGINS}/config
#       DESTINATION share/${PROJECT_NAME}/plugins/${PLUGINS}
#     )
#   endif()
# endforeach()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  if(TEST_SOURCE_CPP_FILES)
    ament_add_gtest(${PROJECT_NAME}_test ${TEST_SOURCE_CPP_FILES})
    ament_target_dependencies(${PROJECT_NAME}_test ${PROJECT_DEPENDENCIES})
  endif()
endif()

ament_package()
