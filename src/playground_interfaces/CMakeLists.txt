cmake_minimum_required(VERSION 3.19)
project(playground_interfaces)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB MSG_FILES     RELATIVE ${PROJECT_SOURCE_DIR} CONFIGURE_DEPENDS msg/*.msg)
file(GLOB SRV_FILES     RELATIVE ${PROJECT_SOURCE_DIR} CONFIGURE_DEPENDS srv/*.srv)
file(GLOB ACTION_FILES  RELATIVE ${PROJECT_SOURCE_DIR} CONFIGURE_DEPENDS action/*.action)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  ${ACTION_FILES}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
