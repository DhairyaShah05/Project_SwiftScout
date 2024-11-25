cmake_minimum_required(VERSION 3.8)
project(multi_robot)

# Add compiler options for warnings if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###################################
# Find Required Dependencies
###################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Doxygen REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

###################################
# Configure Code Coverage and Tests
###################################
# Enable code coverage options when COVERAGE is set
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()

# Configure unit testing with GTest if BUILD_TESTING is enabled
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/basic_test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp std_msgs)
  target_link_libraries(${PROJECT_NAME}_test sar)
endif()

###################################
# Create Compile Commands File
###################################
# Enable generation of compile_commands.json for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

###################################
# Doxygen Documentation Configuration
###################################
# Configure Doxygen if available
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs)
  set(DOXYGEN_COLLABORATION_GRAPH  YES)
  set(DOXYGEN_EXTRACT_ALL          YES)
  set(DOXYGEN_CLASS_DIAGRAMS       YES)
  set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
  set(DOXYGEN_HAVE_DOT             YES)
  set(DOXYGEN_CLASS_GRAPH          YES)
  set(DOXYGEN_CALL_GRAPH           YES)
  set(DOXYGEN_CALLER_GRAPH         YES)
  set(DOXYGEN_BUILTIN_STL_SUPPORT  YES)
  set(DOXYGEN_EXTRACT_PRIVATE      YES)
  set(DOXYGEN_EXTRACT_PACKAGE      YES)
  set(DOXYGEN_EXTRACT_STATIC       YES)
  set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
  set(DOXYGEN_UML_LOOK             YES)
  set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
  set(DOXYGEN_TEMPLATE_RELATIONS   YES)
  set(DOXYGEN_DOT_GRAPH_MAX_NODES  100)
  set(DOXYGEN_MAX_DOT_GRAPH_DEPTH  0)
  set(DOXYGEN_DOT_TRANSPARENT      YES)
else()
  message(FATAL_ERROR "Doxygen must be installed to generate documentation.")
endif()

###################################
# Add Targets
###################################
# Add a Doxygen target to generate documentation
doxygen_add_docs(
  docs
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test
)

# Create the goal_pub executable
add_executable(goal_pub src/goal_publisher.cpp)

# Add dependencies for the goal_pub executable
ament_target_dependencies(goal_pub
  rclcpp std_msgs nav2_msgs rclcpp_action nav2_map_server cv_bridge sensor_msgs)
target_link_libraries(goal_pub sar)

###################################
# Installation Configuration
###################################
# Install executables
install(TARGETS
  goal_pub
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and related assets
install(DIRECTORY
  launch
  model
  urdf
  map
  worlds
  param
  rviz
  DESTINATION share/${PROJECT_NAME})

# Install scripts with executable permissions
install(PROGRAMS
  scripts/run_me_for_fun.bash
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})

# Finalize the package setup
ament_package()

