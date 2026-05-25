# Triangulation nodes (ROS2)
message(STATUS "Building triangulation nodes...")

# Two marker WhyCode triangulation
add_executable(two_marker_whycode_triangulation_node
  two_marker_whycode_triangulation.cpp
  two_marker_whycode_triangulation_node.cpp
)

target_compile_options(two_marker_whycode_triangulation_node PRIVATE ${OPTIMIZATION_FLAGS})

target_include_directories(two_marker_whycode_triangulation_node PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)

ament_target_dependencies(two_marker_whycode_triangulation_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(two_marker_whycode_triangulation_node
  "${cpp_typesupport_target}"
)

# Four marker WhyCode triangulation
add_executable(four_marker_whycode_triangulation_node
  four_marker_whycode_triangulation.cpp
  four_marker_whycode_triangulation_node.cpp
)

target_compile_options(four_marker_whycode_triangulation_node PRIVATE ${OPTIMIZATION_FLAGS})

target_include_directories(four_marker_whycode_triangulation_node PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)

ament_target_dependencies(four_marker_whycode_triangulation_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(four_marker_whycode_triangulation_node
  "${cpp_typesupport_target}"
)

# Install executables
install(TARGETS
  two_marker_whycode_triangulation_node
  four_marker_whycode_triangulation_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
