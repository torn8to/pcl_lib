cmake_minimum_required(VERSION 3.15)
project(pcl_lib VERSION 1.0 LANGUAGES CXX)

option(USE_CUDA "Build with CUDA support" ON)

if(USE_CUDA)
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
    else()
        message(STATUS "CUDA compiler not found. Disabling CUDA support.")
        set(USE_CUDA OFF)
    endif()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)   

# Build all libraries (including dependencies like stdgpu) with -fPIC so
# they can be linked into shared objects.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(CheckLanguage)
include(FetchContent)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
if(rclcpp_FOUND)
    set(ROS2_FOUND TRUE)
    message(STATUS "ROS 2 (rclcpp) found")
endif()

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Iridescence REQUIRED)

include(third_party/sophus.cmake)
include(third_party/yaml.cmake)
include(third_party/bonxai.cmake)

if(USE_CUDA)
    # Example: for NVIDIA Ampere GPUs (like RTX 30xx, A100)
    set(CMAKE_CUDA_ARCHITECTURES 86)

    find_package(CUDAToolkit REQUIRED)
    find_package(CUDA REQUIRED)
    include(third_party/stdgpu.cmake)
endif()

include(third_party/gtest.cmake)
include_directories(${OpenCV_INCLUDE_DIRS})

if(USE_CUDA)
    add_library(${PROJECT_NAME}_lib_gpu SHARED
      src/cuda/GPUVoxelHashMap.cu
      #src/cuda/GPURegistration.cu
      src/cuda/MotionCompensation.cu
    )
    target_include_directories(${PROJECT_NAME}_lib_gpu PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(${PROJECT_NAME}_lib_gpu PUBLIC
      Eigen3::Eigen
      Sophus::Sophus
      stdgpu::stdgpu
      CUDA::cublas
    )
    target_compile_features(${PROJECT_NAME}_lib_gpu PUBLIC cuda_std_17 cxx_std_17)
    target_compile_options(${PROJECT_NAME}_lib_gpu PRIVATE
      $<$<COMPILE_LANGUAGE:CUDA>:
        --expt-relaxed-constexpr
        --expt-extended-lambda
        -lineinfo
      >
    )
endif()


add_library(${PROJECT_NAME}_lib SHARED
  src/Pipeline.cpp
  src/Registration.cpp
  src/VoxelMap.cpp
  src/ekf/Esekf.cpp
  src/ekf/IterEsekf.cpp 
  src/probabilistic_map.cpp
)

# scripts section
add_executable(OfflineBagProcessor scripts/SingleLidarBagReader.cpp)
target_link_libraries(OfflineBagProcessor PRIVATE
  ${PROJECT_NAME}_lib
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  bonxai
)

#add_executable(cpu_pipeline_timings benchmarks/cpu_pipeline_timings.cpp)
#target_include_directories(cpu_pipeline_timings PRIVATE
#${CMAKE_CURRENT_SOURCE_DIR}
#)
#target_link_libraries(cpu_pipeline_timings PRIVATE ${PROJECT_NAME}_lib)

#add_executable(gpu_pipeline_timings benchmarks/gpu_pipeline_timings.cpp)
#target_link_libraries(gpu_pipeline_timings PRIVATE ${PROJECT_NAME}_lib)

#add_executable(compensation_benchmark benchmarks/MotionCompensation.cpp)
#target_include_directories(compensation_benchmark PRIVATE
#  ${CMAKE_CURRENT_SOURCE_DIR}
#  ${CMAKE_CURRENT_SOURCE_DIR}/benchmarks
#)
#target_link_libraries(compensation_benchmark PRIVATE ${PROJECT_NAME}_lib)
#add_executable(odom_bin_generator scripts/odom_read_into_cloud_save.cpp)
#target_link_libraries(odom_bin_generator PRIVATE ${PROJECT_NAME}_lib)

#scripts section end

# Propagate include dirs and CUDA/TPLs via the shared lib
#target_link_libraries(bench_icp PRIVATE ${PROJECT_NAME}_lib)

if(USE_CUDA)
    target_compile_features(${PROJECT_NAME}_lib PUBLIC cuda_std_17 cxx_std_17)

    target_compile_options(${PROJECT_NAME}_lib PRIVATE
        # These flags apply only when compiling CUDA source files
        $<$<COMPILE_LANGUAGE:CUDA>:
            --expt-relaxed-constexpr
            --expt-extended-lambda
            -lineinfo
        >
    )
else()
    target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
endif()

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link required libraries
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  Eigen3::Eigen
  Sophus::Sophus
  TBB::tbb
  ${OpenCV_LIBS}
  Iridescence::Iridescence
  yaml-cpp::yaml-cpp
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_generator_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  nav_msgs::nav_msgs__rosidl_generator_cpp
  nav_msgs::nav_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_generator_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  std_msgs::std_msgs__rosidl_generator_cpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
  rosidl_runtime_cpp::rosidl_runtime_cpp
)

if(USE_CUDA)
    target_include_directories(${PROJECT_NAME}_lib
        PRIVATE ${CUDAToolkit_INCLUDE_DIRS}
    )
endif()

# testing stuff
enable_testing()

#add_executable(MotionCompensationTest
#  test/TestMotionCompensation.cpp
#)

#target_include_directories(MotionCompensationTest PRIVATE
#  ${CMAKE_CURRENT_SOURCE_DIR}/src
#  ${CMAKE_CURRENT_SOURCE_DIR}/include
#)

#target_link_libraries(
#  MotionCompensationTest
#  GTest::gtest_main
#  ${PROJECT_NAME}_lib
#)

if(USE_CUDA)
    add_executable(GpuVoxelHashMapTest
      test/TestGpuVoxelHashMap.cpp
    )

    # Ensure the test is compiled with the CUDA device compiler so that
    # CUDA-only headers such as stdgpu's <stdgpu/...> can be used safely.
    set_source_files_properties(test/TestGpuVoxelHashMap.cpp
      PROPERTIES LANGUAGE CUDA)

    target_include_directories(GpuVoxelHashMapTest PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(
      GpuVoxelHashMapTest
      GTest::gtest_main
      ${PROJECT_NAME}_lib
      ${PROJECT_NAME}_lib_gpu
    )

    target_compile_features(GpuVoxelHashMapTest PUBLIC cuda_std_17 cxx_std_17)

    target_compile_options(GpuVoxelHashMapTest PRIVATE
      $<$<COMPILE_LANGUAGE:CUDA>:
        --expt-relaxed-constexpr
        --expt-extended-lambda
        -lineinfo
      >
    )
    include(GoogleTest)
    gtest_discover_tests(GpuVoxelHashMapTest)
    #gtest_discover_tests(MotionCompensationTest)

    # Set CUDA properties
    set_target_properties(${PROJECT_NAME}_lib PROPERTIES
      CUDA_SEPARABLE_COMPILATION ON
      CUDA_RESOLVE_DEVICE_SYMBOLS ON
    )
endif()



