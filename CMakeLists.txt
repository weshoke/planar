cmake_minimum_required(VERSION 3.1.0)

# Enable C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED on)

# Build debug by default
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE Debug)
endif()


file(GLOB_RECURSE planar_sources "src/*.hpp" "src/*.cpp")
file(GLOB_RECURSE planar_test_sources "test/*.hpp" "test/*.cpp")

# Defining Planar Tests

add_executable(planar-test-all
  ${planar_sources} ${planar_test_sources})

set_target_properties(planar-test-all PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(planar-test-all PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third-party/lest/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third-party/versor/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third-party/versor/include/vsr>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third-party/variant/include>)


get_property(dirs TARGET planar-test-all PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()
