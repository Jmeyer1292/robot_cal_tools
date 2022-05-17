^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rct_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2022-05-17)
------------------
* remove changelogs
* Add changelogs
* Move utilities from ros pkg to common pkg and add missing dependencies cmake config
* Convert rct_optimizations to be a ROS-generic CMake package (`#42 <https://github.com/Jmeyer1292/robot_cal_tools/issues/42>`_)
  * make rct_optimizations a pure CMake package
  change other packages so they treat rct_optimizations as a pure CMake package
  Fix erroneously commented-out Eigen3 dependency
  remove pattern matching filter from include install
  Add rct_common package, move macros and GTest infrastructure to it
  rename RCT_ENABLE_RUN_TESTING to RCT_RUN_TESTS
  Add flags to build and run tests in CI
  install git in CI env
  add git to ADDITIONAL_DEBS for industrial_ci docker image
  Remove RCT_RUN_TESTS flag from CI config
  link rct_examples test against GTest
  * set RCT_RUN_TESTS=True
* Contributors: Joseph Schornak, Levi Armstrong

0.1.0 (2020-03-27)
------------------
