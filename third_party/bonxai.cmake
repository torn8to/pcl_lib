FetchContentDeclare(
  bonxai
  GIT_REPOSITORY https://github.com/facontidavide/Bonxai.git
  GIT_TAG v0.6.0 # release 0.6.0
)

set(BONXAI_BUILD_TESTING  OFF CACHE BOOL "" FORCEE)
set(BONXAI_BUILD_EXANPLES OFF CACHE BOOL "" FORCEE)
set(BONXAI_BUILD_ROS OFF CACHE BOOL "" FORCEE)

FetchContent_MakeAvailable(
  bonxai
)
