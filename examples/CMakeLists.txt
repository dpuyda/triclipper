add_executable(example main.cpp ${CMAKE_SOURCE_DIR}/include/triclipper/triclipper.hpp)

target_include_directories(example PRIVATE ${CMAKE_SOURCE_DIR}/include)
