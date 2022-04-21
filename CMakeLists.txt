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

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -O3 -DNDEBUG -mtune=native -march=native")
else()
  message(WARNING "Compiling with an unsupported compiler. No optimisations set.")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ptu_interfaces REQUIRED)

include_directories(include)

add_library(hal_flir_d46 SHARED src/HALPTUFlirD46.cpp src/driver.cpp)
ament_target_dependencies(hal_flir_d46 rclcpp rcutils serial sensor_msgs std_msgs std_srvs ptu_interfaces rclcpp_action rclcpp_components)

rclcpp_components_register_node(
  hal_flir_d46
  PLUGIN "hal::FlirD46"
  EXECUTABLE HALFlirExecutable)

install(TARGETS hal_flir_d46
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
