FetchContentDeclare(
  bonxai
  GIT_REPOSITORY https://github.com/facontidavide/Bonxai.git
  GIT_TAG 77863653f9202184418dd6352fe173f75f85b634 # release 0.6.0
)

set(BONXAI_BUILD_TESTING  OFF CACHE BOOL "" FORCEE)
set(BONXAI_BUILD_EXANPLES OFF CACHE BOOL "" FORCEE)
set(BONXAI_BUILD_ROS OFF CACHE BOOL "" FORCEE)

FetchContent_MakeAvailable(
  bonxai
)
