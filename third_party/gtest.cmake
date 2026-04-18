include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#disable installation of gtest
set(INSTALL_GTEST OFF)
set(INSTALL_GMOCK OFF)
FetchContent_MakeAvailable(googletest)
