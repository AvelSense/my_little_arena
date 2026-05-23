cmake_minimum_required(VERSION 3.8)
project(description_simple_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++23)
endif()

# list dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  # For controller
  controller_interface
  generate_parameter_library
  realtime_tools
  std_srvs
  # For hardware interface
  # For both
  pluginlib
  rclcpp_lifecycle
  hardware_interface
  # For standard ROS2 features
  rclcpp
)
# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

##############################################################################
#############    Generate custom hardware interfaces plugin      #############
##############################################################################
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
  $<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(hardware_interface description_simple_car.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include
)
install(
  TARGETS simple_hardware_interfaces
  DESTINATION lib
)

##############################################################################
#################      Generate custom controller plugin    ##################
##############################################################################
generate_parameter_library(simple_car_controller_parameters
  controllers/include/car_controller.yaml
)
add_library(
  simple_car_controller
  SHARED
  controllers/car_controller.cpp
)
target_include_directories(simple_car_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
  $<INSTALL_INTERFACE:include/description_simple_car>
)
target_link_libraries(simple_car_controller PUBLIC
                      simple_car_controller_parameters
                      controller_interface::controller_interface
                      hardware_interface::hardware_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      ${std_srvs_TARGETS})
pluginlib_export_plugin_description_file(controller_interface description_simple_car.xml)

install(
  DIRECTORY controllers/include
  DESTINATION include
)
install(
  TARGETS simple_car_controller
  DESTINATION lib
)


##############################################################################
#################             Gather resources              ##################
##############################################################################

install(
  DIRECTORY config launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)


##############################################################################
#################                  Testing                  ##################
##############################################################################
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