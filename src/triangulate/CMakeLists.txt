# Contains triangulation nodes for marker-based pose estimation
message(STATUS "Building triangulation nodes...")

# Set required CMake policies
cmake_policy(SET CMP0079 NEW)  # Allow target_link_libraries() for targets in other directories

# Check for AprilTag dependency for AprilTag triangulation
find_package(apriltag_ros QUIET)
if(apriltag_ros_FOUND)
  set(BUILD_APRILTAG_TRIANGULATION ON)
  message(STATUS "Found apriltag_ros - building AprilTag triangulation node")
else()
  set(BUILD_APRILTAG_TRIANGULATION OFF)
  message(WARNING "apriltag_ros not found - skipping AprilTag triangulation node")
endif()

#=============================================================================
# Four Marker WhyCode Triangulation Node
#=============================================================================
add_executable(four_marker_whycode_triangulation_node
  four_marker_whycode_triangulation.cpp
)

target_compile_options(four_marker_whycode_triangulation_node PRIVATE ${OPTIMIZATION_FLAGS} ${COMPILER_WARNINGS})

target_link_libraries(four_marker_whycode_triangulation_node
  ${catkin_LIBRARIES}
)

add_dependencies(four_marker_whycode_triangulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#=============================================================================
# Two Marker WhyCode Triangulation Node
#=============================================================================
add_executable(two_marker_whycode_triangulation_node
  two_marker_whycode_triangulation.cpp
)

target_compile_options(two_marker_whycode_triangulation_node PRIVATE ${OPTIMIZATION_FLAGS} ${COMPILER_WARNINGS})

target_link_libraries(two_marker_whycode_triangulation_node
  ${catkin_LIBRARIES}
)

add_dependencies(two_marker_whycode_triangulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#=============================================================================
# Two Marker AprilTag Triangulation Node (conditional)
#=============================================================================
if(BUILD_APRILTAG_TRIANGULATION)
  add_executable(two_marker_apriltag_triangulation_node
    two_marker_apriltag_triangulation.cpp
  )

  target_compile_options(two_marker_apriltag_triangulation_node PRIVATE ${OPTIMIZATION_FLAGS} ${COMPILER_WARNINGS})

  target_link_libraries(two_marker_apriltag_triangulation_node
    ${catkin_LIBRARIES}
  )

  add_dependencies(two_marker_apriltag_triangulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
  # Install AprilTag node
  install(TARGETS two_marker_apriltag_triangulation_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

#=============================================================================
# Installation
#=============================================================================
install(TARGETS 
  four_marker_whycode_triangulation_node
  two_marker_whycode_triangulation_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)