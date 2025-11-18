cmake_minimum_required(VERSION 3.15)
project(pcl_lib VERSION 1.0
                LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)   

#include(cmake/CPM.cmake)

include(CheckLanguage)
include(FetchContent)


find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)
find_package(Thrust REQUIRED CONFIG)

thrust_create_target(Thrust)


# Example: for NVIDIA Ampere GPUs (like RTX 30xx, A100)
set(CMAKE_CUDA_ARCHITECTURES 86)

find_package(CUDAToolkit REQUIRED)
find_package(CUDA REQUIRED)

check_language(CUDA)


# Example: for NVIDIA Ampere GPUs (like RTX 30xx, A100)
set(CMAKE_CUDA_ARCHITECTURES 86)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#disable installation of gtest
set(INSTALL_GTEST OFF)
set(INSTALL_GMOCK OFF)
FetchContent_MakeAvailable(googletest)



FetchContent_Declare(
    stdgpu
    GIT_REPOSITORY https://github.com/stotko/stdgpu.git
    GIT_TAG        master
)

# Exclude unneeded parts from the build
set(STDGPU_BUILD_EXAMPLES OFF CACHE INTERNAL "")
set(STDGPU_BUILD_BENCHMARKS OFF CACHE INTERNAL "")
set(STDGPU_BUILD_TESTS OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(stdgpu)

#)


# Enable relaxed constexpr for all CUDA compilations
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")

add_library(${PROJECT_NAME}_lib SHARED
  src/Pipeline.cpp
  src/Registration.cpp
  src/VoxelMap.cpp
  #src/cuda/GPUVoxelHashMap.cu
  #src/cuda/GPURegistration.cu
  src/cuda/MotionCompensation.cu
)

add_executable(odom_bin_generator scripts/odom_read_into_cloud_save.cpp)
target_link_libraries(odom_bin_generator PRIVATE ${PROJECT_NAME}_lib)

# Propagate include dirs and CUDA/TPLs via the shared lib
#target_link_libraries(bench_icp PRIVATE ${PROJECT_NAME}_lib)

target_compile_features(${PROJECT_NAME}_lib PUBLIC cuda_std_17 cxx_std_17)

target_compile_options(${PROJECT_NAME}_lib PRIVATE
    # These flags apply only when compiling CUDA source files
    $<$<COMPILE_LANGUAGE:CUDA>:
        --expt-relaxed-constexpr
        --expt-extended-lambda
        -lineinfo
    >
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link required libraries
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  Eigen3::Eigen
  Sophus::Sophus
  CUDA::cudart
  CUDA::cublas
  CUDA::curand
  TBB::tbb
)

target_include_directories(${PROJECT_NAME}_lib
    PRIVATE ${CUDAToolkit_INCLUDE_DIRS}
)

# testing stuff
enable_testing()

add_executable(MotionCompensationTest
  test/MotionCompensation.cpp
)

target_link_libraries(
  MotionCompensationTest
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(MotionCompensationTest)

# Set CUDA properties
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  CUDA_RESOLVE_DEVICE_SYMBOLS ON
)



