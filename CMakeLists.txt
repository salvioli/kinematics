cmake_minimum_required(VERSION 3.23)

project(kinematics)

# Project options
option(KINEMATICS_BUILD_TESTS "Build tests" OFF)

# Global settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(cmake/compiler_warnings.cmake)

# Find dependencies
find_package(Eigen3 REQUIRED)

# Main library
add_library(kinematics)
add_library(kinematics::kinematics ALIAS kinematics)

target_sources(
  kinematics
  PRIVATE src/kinematics.cpp
  PUBLIC FILE_SET
         HEADERS
         BASE_DIRS
         include
         FILES
         include/kinematics.h)

target_default_compiler_warnings(kinematics)

target_link_libraries(kinematics PUBLIC Eigen3::Eigen)

# Tests configuration
if(KINEMATICS_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.16.0)
  # Prevent overriding parent project's compiler/linker settings on Windows
  set(gtest_force_shared_crt
      ON
      CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()

  add_executable(kinematics_test)
  target_sources(kinematics_test PRIVATE test/kinematics_test.cpp)
  target_link_libraries(kinematics_test PRIVATE kinematics GTest::gtest_main)

  include(GoogleTest)
  gtest_discover_tests(kinematics_test)
endif()

# Installation rules
include(GNUInstallDirs)
install(
  TARGETS kinematics
  EXPORT kinematicsTargets
  FILE_SET HEADERS
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
  EXPORT kinematicsTargets
  FILE kinematicsTargets.cmake
  NAMESPACE kinematics::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kinematics)
