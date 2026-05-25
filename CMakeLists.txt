cmake_minimum_required(VERSION 3.8)
project(whycon_whycode_localization)

option(DISABLE_ROS "Do not build for ROS, but as standalone code" OFF)
option(BUILD_BENCHMARKS "Build benchmark executables" ON)
option(BUILD_TOOLS "Build tool executables" OFF)
option(BUILD_TRIANGULATION "Build triangulation nodes" ON)

if(DISABLE_ROS)
  message(FATAL_ERROR "DISABLE_ROS=ON is not supported in this ROS2 port yet.")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# General optimization flags
set(OPTIMIZATION_FLAGS -O3 -msse3 -mavx -mfma -ffast-math -flto=jobserver -march=native)

# Check for AVX2 support and add it conditionally
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-mavx2 COMPILER_SUPPORTS_AVX2)
if(COMPILER_SUPPORTS_AVX2)
  message(STATUS "Compiler supports AVX2, enabling it with -mavx2 flag.")
  set(OPTIMIZATION_FLAGS ${OPTIMIZATION_FLAGS} -mavx2)
else()
  message(WARNING "Compiler does not support AVX2. The AVX2-optimized code will not compile.")
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# System dependencies
find_package(OpenCV REQUIRED)
message(STATUS "Using OpenCV version ${OpenCV_VERSION}")
find_package(yaml-cpp REQUIRED)

# Find the Simd library for accelerated image processing
find_path(SIMD_INCLUDE_DIR Simd/SimdLib.hpp)
find_library(SIMD_LIBRARY NAMES Simd)
if(NOT SIMD_INCLUDE_DIR OR NOT SIMD_LIBRARY)
  message(FATAL_ERROR "Simd library not found. Please ensure it is installed correctly (e.g., in /usr/local).")
else()
  message(STATUS "Found Simd library: ${SIMD_LIBRARY}")
endif()

include_directories(/usr/include/eigen3)

# ROS2 custom message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WhyCodePose.msg"
  "msg/WhyCodePoseArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# WhyCon Core Library (algorithmic core + image pipeline)
add_library(whycon_core SHARED
  src/core/CNecklace.cpp
  src/utils/coord_lut.cpp
  src/image/debug_image_manager.cpp
  src/tracking/id_stabilizer.cpp
  src/image/image_handler.cpp
  src/image/packed_binary_image.cpp
  src/core/marker_detector.cpp
  src/tracking/marker_tracker.cpp
  src/core/multi_marker_detector.cpp
  src/utils/param_loader.cpp
  src/tracking/pose_stabilizer.cpp
  src/utils/whycon_config.cpp
  src/core/whycon_localization.cpp
)

target_include_directories(whycon_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SIMD_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

target_compile_options(whycon_core PRIVATE ${OPTIMIZATION_FLAGS})

target_link_libraries(whycon_core
  # PUBLIC
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  ${SIMD_LIBRARY}
)

ament_target_dependencies(whycon_core
  sensor_msgs
)

add_library(whycon_component SHARED
  src/ros/whycon_component.cpp
  src/ros/whycon_ros_interface.cpp
)

target_compile_options(whycon_component PRIVATE ${OPTIMIZATION_FLAGS})

target_link_libraries(whycon_component
  whycon_core
  "${cpp_typesupport_target}"
)

ament_target_dependencies(whycon_component
  ament_index_cpp
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rclcpp_components_register_nodes(whycon_component "whycon::WhyconComponent")

# WhyCon ROS2 node
add_executable(whycon
  src/ros/whycon_localization_node.cpp
)

target_compile_options(whycon PRIVATE ${OPTIMIZATION_FLAGS})

target_link_libraries(whycon
  whycon_component
)

ament_target_dependencies(whycon
  ament_index_cpp
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(whycon "${cpp_typesupport_target}")

# WhyCode Marker Generator Tool
add_executable(whycode_gen
  src/tools/whycode_gen.cpp
  src/core/CNecklace.cpp
)

target_include_directories(whycode_gen PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(whycode_gen PROPERTIES
  OUTPUT_NAME whycon-id-gen
)

target_link_libraries(whycode_gen
  ${CMAKE_MATH_LIBS}
)

# Optional subdirectories
if(BUILD_TOOLS)
  message(WARNING "BUILD_TOOLS is ON, but ROS2 port for src/tools is not completed yet. Skipping tools build.")
endif()

if(BUILD_TRIANGULATION)
  message(STATUS "BUILD_TRIANGULATION=ON: adding triangulation nodes")

  add_executable(two_marker_whycode_triangulation_node
    src/triangulate/two_marker_whycode_triangulation_node.cpp
  )

  target_compile_options(two_marker_whycode_triangulation_node PRIVATE ${OPTIMIZATION_FLAGS})

  ament_target_dependencies(two_marker_whycode_triangulation_node
    rclcpp
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
  )

  target_link_libraries(two_marker_whycode_triangulation_node
    "${cpp_typesupport_target}"
  )

  add_executable(four_marker_whycode_triangulation_node
    src/triangulate/four_marker_whycode_triangulation_node.cpp
  )

  target_compile_options(four_marker_whycode_triangulation_node PRIVATE ${OPTIMIZATION_FLAGS})

  ament_target_dependencies(four_marker_whycode_triangulation_node
    rclcpp
    geometry_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
  )

  target_link_libraries(four_marker_whycode_triangulation_node
    "${cpp_typesupport_target}"
  )

  install(TARGETS two_marker_whycode_triangulation_node four_marker_whycode_triangulation_node
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(BUILD_BENCHMARKS)
  add_subdirectory(src/benchmarks)
endif()

# Install
install(TARGETS whycon_core whycon_component whycode_gen whycon
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Export and package
ament_export_include_directories(include)
ament_export_libraries(whycon_core whycon_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  rosidl_default_runtime
  tf2_geometry_msgs
)

ament_package()
