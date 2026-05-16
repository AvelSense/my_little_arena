cmake_minimum_required(VERSION 3.8)
project(description_simple_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# list dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  simple_hardware_interfaces
  SHARED
  hardware/PwmWheelActuator.cpp
  hardware/PwmServoActuator.cpp
)
ament_target_dependencies(
  simple_hardware_interfaces
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_include_directories(simple_hardware_interfaces PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/description_simple_car>
)

pluginlib_export_plugin_description_file(hardware_interface description_simple_car.xml)

# INSTALL
install(
  TARGETS simple_hardware_interfaces
  DESTINATION lib
)
install(
  DIRECTORY hardware/include/
  DESTINATION include/description_simple_car
)

install(
  DIRECTORY config launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()