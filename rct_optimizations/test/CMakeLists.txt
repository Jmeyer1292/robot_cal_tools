find_package(GTest REQUIRED)

# Build the test support library
add_library(${PROJECT_NAME}_test_support
  src/pose_generator.cpp
  src/observation_creator.cpp
  src/utilities.cpp)
target_link_libraries(${PROJECT_NAME}_test_support ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_test_support PUBLIC include)

# The actual tests...
add_executable(${PROJECT_NAME}_conversion_tests conversion_utest.cpp)
target_link_libraries(${PROJECT_NAME}_conversion_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest GTest::Main)
rct_gtest_discover_tests(${PROJECT_NAME}_conversion_tests)
add_dependencies(${PROJECT_NAME}_conversion_tests ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_conversion_tests)

add_executable(${PROJECT_NAME}_extrinsic_multi_static_camera_tests extrinsic_multi_static_camera_utest.cpp)
target_link_libraries(${PROJECT_NAME}_extrinsic_multi_static_camera_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest GTest::Main)
rct_gtest_discover_tests(${PROJECT_NAME}_extrinsic_multi_static_camera_tests)
add_dependencies(${PROJECT_NAME}_extrinsic_multi_static_camera_tests ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_extrinsic_multi_static_camera_tests)

add_executable(${PROJECT_NAME}_extrinsic_camera_on_wrist_tests extrinsic_camera_on_wrist_utest.cpp)
target_link_libraries(${PROJECT_NAME}_extrinsic_camera_on_wrist_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest GTest::Main)
rct_gtest_discover_tests(${PROJECT_NAME}_extrinsic_camera_on_wrist_tests)
add_dependencies(${PROJECT_NAME}_extrinsic_camera_on_wrist_tests ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_extrinsic_camera_on_wrist_tests)

add_executable(${PROJECT_NAME}_extrinsic_hand_eye_tests extrinsic_hand_eye_utest.cpp)
target_link_libraries(${PROJECT_NAME}_extrinsic_hand_eye_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest GTest::Main)
rct_gtest_discover_tests(${PROJECT_NAME}_extrinsic_hand_eye_tests)
add_dependencies(${PROJECT_NAME}_extrinsic_hand_eye_tests ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_extrinsic_hand_eye_tests)

add_executable(${PROJECT_NAME}_dh_parameter_tests dh_parameter_utest.cpp)
target_link_libraries(${PROJECT_NAME}_dh_parameter_tests PRIVATE GTest::GTest GTest::Main)
rct_gtest_discover_tests(${PROJECT_NAME}_dh_parameter_tests)
add_dependencies(${PROJECT_NAME}_dh_parameter_tests ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}_dh_parameter_tests)

# Install the test executables so they can be run independently later if needed
install(
  TARGETS
    ${PROJECT_NAME}_conversion_tests
    ${PROJECT_NAME}_extrinsic_multi_static_camera_tests
    ${PROJECT_NAME}_extrinsic_camera_on_wrist_tests
    ${PROJECT_NAME}_extrinsic_hand_eye_tests
    ${PROJECT_NAME}_dh_parameter_tests
  RUNTIME DESTINATION bin/tests
  LIBRARY DESTINATION lib/tests
  ARCHIVE DESTINATION lib/tests
)
