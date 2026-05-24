cmake_minimum_required(VERSION 3.0.2)
project(whycon_whycode_localization)
option(DISABLE_ROS "Do not build for ROS, but as standalone code" OFF)

# Build control options
option(BUILD_BENCHMARKS "Build benchmark executables" ON) 
option(BUILD_TOOLS "Build tool executables" ON)
option(BUILD_TRIANGULATION "Build triangulation nodes" ON)

#=============================================================================
# Compiler Settings & Optimizations
#=============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# General optimization flags
set(OPTIMIZATION_FLAGS -O3 -msse3 -mavx -mfma -ffast-math -flto=jobserver -march=native)

# Standard developer warnings to catch potential issues
# set(COMPILER_WARNINGS -Wall -Wextra -Wpedantic)

# add_compile_options(-fopt-info-vec) # Uncomment for vectorization optimization info

# Check for AVX2 support and add it conditionally
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG(-mavx2 COMPILER_SUPPORTS_AVX2)
if(COMPILER_SUPPORTS_AVX2)
  message(STATUS "Compiler supports AVX2, enabling it with -mavx2 flag.")
  set(OPTIMIZATION_FLAGS ${OPTIMIZATION_FLAGS} -mavx2)
else()
  message(WARNING "Your compiler does not support AVX2. The AVX2-optimized code will not compile.")
endif()

#=============================================================================
# Find ROS & System Dependencies
#=============================================================================
if(NOT DISABLE_ROS)
  # Find required ROS components
  find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    message_generation
    nodelet
    pluginlib
    rosbag
    roscpp
    std_msgs
    sensor_msgs
    tf
    visualization_msgs
  )
endif()

# Find required system libraries
find_package(OpenCV REQUIRED)
message(STATUS "Using OpenCV version ${OpenCV_VERSION}")
find_package(yaml-cpp REQUIRED)


# Find the Simd library for accelerated image processing
find_path(SIMD_INCLUDE_DIR Simd/SimdLib.hpp)
find_library(SIMD_LIBRARY NAMES Simd)
if(NOT SIMD_INCLUDE_DIR OR NOT SIMD_LIBRARY)
  message(FATAL_ERROR "Simd Library not found. Please ensure it is installed correctly (e.g., in /usr/local).")
else()
  message(STATUS "Found Simd Library: ${SIMD_LIBRARY}")
endif()

include_directories(/usr/include/eigen3)

# Iceoryx Shared Memory 
find_package(iceoryx_posh REQUIRED)

#=============================================================================
# Custom Message Generation
#=============================================================================
if(NOT DISABLE_ROS)
  add_message_files(
    FILES
    WhyCodePose.msg
    WhyCodePoseArray.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
  )
endif()
#=============================================================================
# Catkin Package Configuration
#=============================================================================
if(NOT DISABLE_ROS)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime nodelet pluginlib visualization_msgs
  )
endif()

#=============================================================================
# Build Targets (Core Library & Main Executable)
#=============================================================================
# Add public include directory for all targets in this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#--- WhyCon Core Library (no ROS) ---
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

target_compile_options(whycon_core PRIVATE ${OPTIMIZATION_FLAGS} ${COMPILER_WARNINGS})

target_link_libraries(whycon_core
  PUBLIC
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  ${SIMD_LIBRARY}
  ${catkin_LIBRARIES}
  iceoryx_posh::iceoryx_posh
)

if(NOT DISABLE_ROS)
  #--- WhyCon ROS Nodelet ---
  add_library(whycon_ros_nodelet SHARED
    src/ros/whycon_ros_interface.cpp
    src/ros/whycon_nodelet.cpp
  )

  target_compile_options(whycon_ros_nodelet PRIVATE ${OPTIMIZATION_FLAGS} ${COMPILER_WARNINGS})

  target_link_libraries(whycon_ros_nodelet 
    whycon_core 
    ${catkin_LIBRARIES}
  )

  #--- WhyCon ROS Node ---
  add_executable(whycon_localization_node
    src/ros/whycon_localization_node.cpp
  )

  set_target_properties(whycon_localization_node PROPERTIES OUTPUT_NAME whycon)

  target_compile_options(whycon_localization_node PRIVATE ${OPTIMIZATION_FLAGS} ${COMPILER_WARNINGS})

  target_link_libraries(whycon_localization_node
    whycon_core
    whycon_ros_nodelet
    ${catkin_LIBRARIES}
    ${cv_bridge_LIBRARIES}
  )

  # Add dependency to ensure messages are generated before the node is built
  add_dependencies(whycon_localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

endif()

#=============================================================================
# WhyCode Marker Generator Tool
#=============================================================================
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

#=============================================================================
# Optional Subdirectories
#=============================================================================

# Tools directory (cameras, bag processing, etc.)
if(BUILD_TOOLS AND NOT DISABLE_ROS)
  add_subdirectory(src/tools)
endif()

# Triangulation nodes
if(BUILD_TRIANGULATION AND NOT DISABLE_ROS)
  add_subdirectory(src/triangulate)
endif()

# Benchmark executables
if(BUILD_BENCHMARKS)
  add_subdirectory(src/benchmarks)
endif()

#=============================================================================
# Installation Rules
#=============================================================================
if(NOT DISABLE_ROS)
  install(TARGETS whycon_core whycon_ros_nodelet whycon_localization_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(FILES nodelet_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

  install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
  )

  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()

# Install the marker generator tool
install(TARGETS whycode_gen
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



