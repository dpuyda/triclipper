add_executable(example
  ${CMAKE_SOURCE_DIR}/include/triclipper/triclipper.h
  main.cpp
)

target_include_directories(example
  PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
