cmake_minimum_required(VERSION 3.5.0)
project(rct_common VERSION 0.1.0)

include(cmake/rct_macros.cmake)

if(RCT_BUILD_TESTS)
  find_package(GTest QUIET)
  if ( NOT GTest_FOUND )

    include(ExternalProject)

    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(GTEST_CXX_FLAGS "-w -std=c++14")
      set(GTEST_C_FLAGS "-w")

      ExternalProject_Add(GTest
        GIT_REPOSITORY    https://github.com/google/googletest.git
        GIT_TAG           release-1.8.1
        SOURCE_DIR        ${CMAKE_BINARY_DIR}/../rct_common/googletest-src
        BINARY_DIR        ${CMAKE_BINARY_DIR}/../rct_common/googletest-build
        CMAKE_CACHE_ARGS
                -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
                -DCMAKE_BUILD_TYPE:STRING=Release
                -DCMAKE_CXX_FLAGS:STRING=${GTEST_CXX_FLAGS}
                -DCMAKE_C_FLAGS:STRING=${GTEST_C_FLAGS}
                -DBUILD_GMOCK:BOOL=OFF
                -DBUILD_GTEST:BOOL=ON
                -DBUILD_SHARED_LIBS:BOOL=ON
      )
    elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      ExternalProject_Add(GTest
        GIT_REPOSITORY    https://github.com/google/googletest.git
        GIT_TAG           release-1.8.1
        SOURCE_DIR        ${CMAKE_BINARY_DIR}/../rct_common/googletest-src
        BINARY_DIR        ${CMAKE_BINARY_DIR}/../rct_common/googletest-build
        CMAKE_CACHE_ARGS
                -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
                -DCMAKE_BUILD_TYPE:STRING=Release
                -DBUILD_GMOCK:BOOL=OFF
                -DBUILD_GTEST:BOOL=ON
                -DBUILD_SHARED_LIBS:BOOL=ON
      )
    endif()
  endif()
endif()

add_library(${PROJECT_NAME} INTERFACE)

rct_configure_package(${PROJECT_NAME})

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/rct_macros.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})