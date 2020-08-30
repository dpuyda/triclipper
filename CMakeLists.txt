cmake_minimum_required(VERSION 3.9)
project(triclipper)

option(BUILD_EXAMPLES "Build examples" OFF)
option(BUILD_TESTS "Build tests" OFF)
option(DEBUG_OUTPUT "Enable debug output" OFF)

# Enable debug output
if (DEBUG_OUTPUT)
  add_definitions(-DTRICLIPPER_DEBUG)
endif()

# Build examples
if (BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# Build tests
if (BUILD_TESTS)
  add_subdirectory(tests)
endif()
