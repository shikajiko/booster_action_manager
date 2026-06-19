cmake_minimum_required(VERSION 3.8)
project(booster_action_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(booster_model REQUIRED)
find_package(booster_interface REQUIRED)
find_package(booster_client_interface REQUIRED)
find_package(action_interface REQUIRED)


add_library(${PROJECT_NAME}_lib
  src/node/action_manager_node.cpp
  src/action_manager.cpp 
  src/parser.cpp 
  src/trajectory_client.cpp 
  src/change_mode_client.cpp
 )

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_action
  booster_model
  booster_interface
  action_interface
  booster_client_interface
)

add_executable(main src/main.cpp)

target_link_libraries(main ${PROJECT_NAME}_lib)
ament_target_dependencies(
  main
  rclcpp
  rclcpp_action
  booster_model
  booster_interface
  action_interface
  booster_client_interface
)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_waving_flag_json
    test/test_waving_flag_json.cpp
  )

  target_link_libraries(test_waving_flag_json
    ${PROJECT_NAME}_lib
  )

  ament_target_dependencies(test_waving_flag_json
    rclcpp
    rclcpp_action
    booster_model
    booster_interface
    action_interface
    booster_client_interface
  )
endif()

ament_package()
