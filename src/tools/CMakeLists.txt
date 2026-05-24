# Contains utilities for bag processing, shared memory, and camera tools
message(STATUS "Building tools...")

# Set required CMake policies
cmake_policy(SET CMP0076 NEW)  # target_sources() converts relative paths to absolute
cmake_policy(SET CMP0079 NEW)  # Allow target_link_libraries() for targets in other directories

#=============================================================================
# Bag Image Publisher Nodelet (add to existing nodelet target)
#=============================================================================
# Add source to the nodelet target that was created in the main CMakeLists.txt
target_sources(whycon_ros_nodelet PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/bag_image_publisher_nodelet.cpp
)

# Link rosbag library to the nodelet target
target_link_libraries(whycon_ros_nodelet 
  ${rosbag_LIBRARIES}
)

#=============================================================================
# Shared Memory Tools
#=============================================================================

# Bag to Iceoryx Publisher
add_executable(bag_to_shm
  bag_to_shm.cpp
)

target_compile_options(bag_to_shm PRIVATE ${OPTIMIZATION_FLAGS} ${COMPILER_WARNINGS})

target_link_libraries(bag_to_shm
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  iceoryx_posh::iceoryx_posh
)

add_dependencies(bag_to_shm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Iceoryx Image Subscriber/Viewer
add_executable(shm_viewer
  shm_viewer.cpp
)

target_compile_options(shm_viewer PRIVATE ${OPTIMIZATION_FLAGS} ${COMPILER_WARNINGS})

target_link_libraries(shm_viewer
  ${OpenCV_LIBRARIES}
  iceoryx_posh::iceoryx_posh
)

add_dependencies(shm_viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#=============================================================================
# Installation
#=============================================================================
install(TARGETS bag_to_shm shm_viewer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)