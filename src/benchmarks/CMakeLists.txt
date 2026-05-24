# Contains performance benchmarks and unit tests
message(STATUS "Building benchmarks and tests...")

# Set required CMake policies
cmake_policy(SET CMP0079 NEW)  # Allow target_link_libraries() for targets in other directories

# Check for additional benchmark dependencies
find_package(Threads REQUIRED)

# Add XSIMD include fix - ensure proper header ordering
add_definitions(-DXSIMD_ENABLE_FALLBACK)

# Common benchmark libraries and flags
set(BENCHMARK_LIBRARIES
  whycon_core
  ${OpenCV_LIBRARIES}
  ${SIMD_LIBRARY}
  Threads::Threads
)

# Fix XSIMD header ordering issues
set(BENCHMARK_COMPILE_OPTIONS 
  ${OPTIMIZATION_FLAGS} 
  -DNDEBUG
  # -include xsimd/config/xsimd_inline.hpp  # Force include inline definitions first
)

#=============================================================================
# SIMD and Performance Benchmarks
#=============================================================================

# Core algorithms benchmark
add_executable(benchmark
  benchmark.cpp
)

target_compile_options(benchmark PRIVATE ${BENCHMARK_COMPILE_OPTIONS})
target_link_libraries(benchmark ${BENCHMARK_LIBRARIES})

# Segment computation benchmark
add_executable(segment_bench
  segment_bench.cpp
)

target_compile_options(segment_bench PRIVATE ${BENCHMARK_COMPILE_OPTIONS})
target_link_libraries(segment_bench ${BENCHMARK_LIBRARIES})

# Ellipse computation benchmark
add_executable(ellipse_bench
  ellipse_bench.cpp
)

target_compile_options(ellipse_bench PRIVATE ${BENCHMARK_COMPILE_OPTIONS})
target_link_libraries(ellipse_bench ${BENCHMARK_LIBRARIES})

# Binarization loop benchmark
add_executable(binarize_loop_bench
  binarize_loop_bench.cpp
)

target_compile_options(binarize_loop_bench PRIVATE ${BENCHMARK_COMPILE_OPTIONS})
target_link_libraries(binarize_loop_bench ${BENCHMARK_LIBRARIES})

# Bilinear interpolation benchmark
add_executable(bilinear_bench
  bilinear_bench.cpp
)

target_compile_options(bilinear_bench PRIVATE ${BENCHMARK_COMPILE_OPTIONS})
target_link_libraries(bilinear_bench ${BENCHMARK_LIBRARIES})

# Binarization benchmark
add_executable(benchmark_binarization
  benchmark_binarization.cpp
)

target_compile_options(benchmark_binarization PRIVATE ${BENCHMARK_COMPILE_OPTIONS})
target_link_libraries(benchmark_binarization ${BENCHMARK_LIBRARIES})

#=============================================================================
# Unit Tests and Algorithm Tests
#=============================================================================

# Basic SIMD test
add_executable(cv_simd
  cv_simd.cpp
)

target_compile_options(cv_simd PRIVATE ${BENCHMARK_COMPILE_OPTIONS})
target_link_libraries(cv_simd ${BENCHMARK_LIBRARIES})

# Branch prediction test
add_executable(branch_pred
  branch_pred.cpp
)

target_compile_options(branch_pred PRIVATE ${BENCHMARK_COMPILE_OPTIONS})
target_link_libraries(branch_pred ${BENCHMARK_LIBRARIES})

# Class overhead test
add_executable(class_test
  class_test.cpp
)

target_compile_options(class_test PRIVATE ${BENCHMARK_COMPILE_OPTIONS})
target_link_libraries(class_test ${BENCHMARK_LIBRARIES})

#=============================================================================
# Optional Installation
#=============================================================================
# Uncomment if you want benchmarks installed
# install(TARGETS 
#   benchmark segment_bench ellipse_bench binarize_loop_bench 
#   bilinear_bench benchmark_binarization cv_simd branch_pred class_test
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )