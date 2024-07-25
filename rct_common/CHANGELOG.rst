^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rct_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2024-07-25)
------------------
* fix cxx target version
* Update to use RICB (`#109 <https://github.com/Jmeyer1292/robot_cal_tools/issues/109>`_)
  * Updated to use RICB
  * Updated CI
  * Dropped Bionic build
* Linters (`#108 <https://github.com/Jmeyer1292/robot_cal_tools/issues/108>`_)
  * Add clang format linter and CI build
  * Add cmake format linter and CI build
  * Add CI badges to the readme
  * Replace symlinks with symlink to CI directory
  * Updated workflows
  * Change clang format version
  * Ran clang format
  * Ran CMake format
  * Added git to ADDITIONAL_DEBS field
  ---------
  Co-authored-by: Levi Armstrong <levi.armstrong@gmail.com>
* Contributors: Levi Armstrong, Michael Ripperger

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
