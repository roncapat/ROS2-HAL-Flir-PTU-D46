cmake_minimum_required(VERSION 3.5)
project(hal_flir_d46)

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

# NEEDED TO WORK IN CLION (CMAKE-BASED IDEs)
include_directories(/opt/ros/$ENV{ROS_CHOOSE_DISTRO}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../build/${PROJECT_NAME}/rosidl_generator_cpp)
link_directories(/opt/ros/$ENV{ROS_CHOOSE_DISTRO}/lib)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../build/${PROJECT_NAME})
set(FastRTPS_INCLUDE_DIR /opt/ros/$ENV{ROS_CHOOSE_DISTRO}/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/$ENV{ROS_CHOOSE_DISTRO}/lib/libfastrtps.so)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(flir_ptu_d46_interfaces REQUIRED)

#rosidl_generate_interfaces(${PROJECT_NAME}
#  "srv/GetAmplitudeThreshold.srv"
#   DEPENDENCIES sensor_msgs
# )

include_directories(include)

add_executable(HALFlirD46 src/HALPTUFlirD46.cpp src/driver.cpp)
#rosidl_target_interfaces(HALFlirD46 ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(HALFlirD46 rclcpp rcutils serial sensor_msgs std_msgs std_srvs flir_ptu_d46_interfaces)
#target_link_libraries(HALFlirD46 mesasr)


install(TARGETS HALFlirD46
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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
