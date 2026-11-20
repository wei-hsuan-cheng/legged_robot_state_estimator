cmake_minimum_required(VERSION 3.14)
project(legged_robot_state_estimator LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(legged_state_estimator REQUIRED)
set(DEPENDENCIES
  controller_interface
  generate_parameter_library
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2_msgs
)
foreach(dependency IN LISTS DEPENDENCIES)
  find_package(${dependency} REQUIRED)
endforeach()

generate_parameter_library(humanoid_inekf_parameters config/humanoid_inekf_parameter.yaml)
generate_parameter_library(humanoid_linear_kf_parameters config/humanoid_linear_kf_parameter.yaml)

add_library(${PROJECT_NAME} SHARED
  src/inekf_floating_base_estimator.cpp
  src/linear_kf_floating_base_estimator.cpp
  src/humanoid_state_estimator.cpp
  src/humanoid_inekf.cpp
  src/humanoid_linear_kf.cpp
  src/humanoid_state_estimator_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  legged_state_estimator::legged_state_estimator
  humanoid_inekf_parameters
  humanoid_linear_kf_parameters
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})
target_compile_definitions(${PROJECT_NAME} PUBLIC PLUGINLIB__DISABLE_BOOST_FUNCTIONS)
pluginlib_export_plugin_description_file(controller_interface legged_robot_state_estimator_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md legged_robot_state_estimator_plugin.xml DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} humanoid_inekf_parameters humanoid_linear_kf_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(estimator_test test/estimator_test.cpp)
  target_link_libraries(estimator_test ${PROJECT_NAME})
  target_compile_definitions(estimator_test PRIVATE
    TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/two_wheel.urdf"
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio legged_state_estimator ${DEPENDENCIES})
ament_package()
