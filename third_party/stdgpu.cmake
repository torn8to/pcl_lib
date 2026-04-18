include(FetchContent)

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

# stdgpu (when built with CMake 3.23+ FILE_SET) does not set BUILD_INTERFACE include
# dirs, so we must add them so CUDA files that #include <stdgpu/...> can find headers.
target_include_directories(stdgpu PUBLIC
  $<BUILD_INTERFACE:${stdgpu_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${stdgpu_BINARY_DIR}/include>
)

# Enable relaxed constexpr for all CUDA compilations
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --expt-relaxed-constexpr")
