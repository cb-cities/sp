project(sp LANGUAGES CXX)

# Require C++14-compliant compiler; only available for CMake v. 3.1 and up
set(CMAKE_CXX_STANDARD 14)

cmake_minimum_required(VERSION 3.1)

SET(CMAKE_COLOR_MAKEFILE ON)
SET(CMAKE_VERBOSE_MAKEFILE OFF)

# General compile settings
IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "Debug")
 #SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)

# GNU specific settings
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
endif()

# Intel specific settings
if (CMAKE_CXX_COMPILER_ID MATCHES "Intel")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
endif()

# Clang specific settings
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-undefined-var-template")
endif()

# CMake seems to have no way to enable/disable testing per subproject,
# so we provide an option similar to BUILD_TESTING, but just for SP.
option(SP_BUILD_TESTING "enable testing for sp" ON)

# CMake Modules
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Include directories
include_directories(BEFORE
  ${sp_SOURCE_DIR}/include/
  ${sp_SOURCE_DIR}/external/
)

# sp executable
SET(sp_src
  ${sp_SOURCE_DIR}/src/graph.cc
)

add_executable(sp ${sp_src} ${sp_SOURCE_DIR}/src/main.cc)

add_library(lsp SHARED ${sp_src} ${sp_SOURCE_DIR}/src/py.cc)

# Unit test
if(SP_BUILD_TESTING)
  SET(test_src
    ${sp_SOURCE_DIR}/tests/test_main.cc
    ${sp_SOURCE_DIR}/tests/graph_test.cc
    )
  add_executable(sptest ${sp_src} ${test_src})
  add_test(NAME sptest COMMAND $<TARGET_FILE:sptest>)
  enable_testing()
endif()

# Coverage
find_package(codecov)
if(ENABLE_COVERAGE)
  #add_executable(mpmtest_coverage ${mpm_src} ${test_src})
  add_coverage(sptest)
endif()
