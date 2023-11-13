# Performs multiple operation so other packages may find a package Adapted from the Tesseract CMake macros Usage:
# rct_configure_package(targetA targetB ...) * It installs the provided targets * It exports the provided targets under
# the namespace rct:: * It installs the package.xml file * It create and install the ${PROJECT_NAME}-config.cmake and
# ${PROJECT_NAME}-config-version.cmake
macro(rct_configure_package)
  install(
    TARGETS ${ARGV}
    EXPORT ${PROJECT_NAME}-targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
  install(EXPORT ${PROJECT_NAME}-targets NAMESPACE rct:: DESTINATION lib/cmake/${PROJECT_NAME})

  install(FILES package.xml DESTINATION share/${PROJECT_NAME})

  # Create cmake config files
  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    ${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

  write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
                                   VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
          DESTINATION lib/cmake/${PROJECT_NAME})

  export(EXPORT ${PROJECT_NAME}-targets NAMESPACE rct:: FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
endmacro()

# This macro call the appropriate gtest function to add a test based on the cmake version Usage:
# rct_gtest_discover_tests(target)
macro(rct_gtest_discover_tests target)
  if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
    gtest_add_tests(${target} "" AUTO)
  else()
    gtest_discover_tests(${target})
  endif()
endmacro()

# This macro add a custom target that will run the tests after they are finished building when RCT_RUN_TESTS is enabled
# This is added to allow ability do disable the running of tests as part of the build for CI which calls make test
macro(rct_add_run_tests_target)
  if(RCT_RUN_TESTS)
    add_custom_target(run_tests ALL WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                      COMMAND ${CMAKE_CTEST_COMMAND} -V -O "/tmp/${PROJECT_NAME}_ctest.log" -C $<CONFIGURATION>)
  else()
    add_custom_target(run_tests)
  endif()
endmacro()
