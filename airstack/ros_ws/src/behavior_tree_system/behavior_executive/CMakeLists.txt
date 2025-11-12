cmake_minimum_required(VERSION 3.8)
project(behavior_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(behavior_tree REQUIRED)
find_package(behavior_tree_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(behavior_executive
  src/behavior_executive.cpp
  src/mavros_adapter.cpp
  src/behavior_logger.cpp
  src/action_handlers/geofence_mapping_handler.cpp
  src/action_handlers/navigate_to_waypoint_handler.cpp
  src/action_handlers/search_handler.cpp
  src/action_handlers/survey_handler.cpp
  src/mission_planning/waypoint_generator.cpp
  src/mission_planning/geometry_utils.cpp
)
target_include_directories(behavior_executive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(behavior_executive PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(behavior_executive
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
  behavior_tree
  behavior_tree_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS behavior_executive
  DESTINATION lib/${PROJECT_NAME})

# Install Python test scripts (commented out - files don't exist yet)
# install(PROGRAMS
#   test/transition_tester.py
#   test/complete_transition_tester.py
#   test/validate_transitions.py
#   test/mock_mavros.py
#   test/safety_tester.py
#   test/analyze_safety.py
#   test/geofence_mapping_tester.py
#   test/analyze_geofence_mapping.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install config files (commented out - directory doesn't exist yet)
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# Install documentation (commented out - files don't exist yet)
# install(FILES
#   TRANSITION_LOGGING.md
#   DESTINATION share/${PROJECT_NAME}
# )

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
  
  # TODO: 添加 C++ 单元测试（测试文件暂未创建）
  # find_package(ament_cmake_gtest REQUIRED)
  # 
  # ament_add_gtest(test_geofence_handler
  #   test/test_geofence_handler.cpp
  #   src/action_handlers/geofence_mapping_handler.cpp
  #   src/mission_planning/waypoint_generator.cpp
  #   src/mission_planning/geometry_utils.cpp
  # )
  # 
  # target_include_directories(test_geofence_handler PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )
  # 
  # ament_target_dependencies(test_geofence_handler
  #   rclcpp
  #   sensor_msgs
  #   mavros_msgs
  #   behavior_tree
  # )
  # 
  # ament_add_gtest(test_navigate_to_waypoint_handler
  #   test/test_navigate_to_waypoint_handler.cpp
  #   src/action_handlers/navigate_to_waypoint_handler.cpp
  # )
  # 
  # target_include_directories(test_navigate_to_waypoint_handler PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  # )
  # 
  # ament_target_dependencies(test_navigate_to_waypoint_handler
  #   rclcpp
  #   sensor_msgs
  #   mavros_msgs
  #   behavior_tree
  # )
endif()

ament_package()
