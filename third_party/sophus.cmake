include(FetchContent)

FetchContent_Declare(
  Sophus
  GIT_REPOSITORY https://github.com/strasdat/Sophus.git
  GIT_TAG        main
)

set(BUILD_SOPHUS_TESTS OFF CACHE INTERNAL "")
set(BUILD_SOPHUS_EXAMPLES OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(Sophus)
