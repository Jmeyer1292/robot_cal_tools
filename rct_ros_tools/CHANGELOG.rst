^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rct_ros_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* remove changelogs
* Add changelogs
* Fix issue `#102 <https://github.com/Jmeyer1292/robot_cal_tools/issues/102>`_ using private yamlcpp header
* Move utilities from ros pkg to common pkg and add missing dependencies cmake config
* Improvement Updates (`#101 <https://github.com/Jmeyer1292/robot_cal_tools/issues/101>`_)
  * Added random seed to homography validation for consistent behavior
  * Skip failed PnP optimizations in camera intrinsic calibration
  * Improved intrinsic calibration tool
  * Handled bad images and poses in data set
* Target Finder Plugins (`#96 <https://github.com/Jmeyer1292/robot_cal_tools/issues/96>`_)
  * Removed unused utility
  * Revised data set to take a target finder base class
  * Replaced target loaders with target finder plugins
  * Consolidated target finder plugin implementations into a single file
  * Revised command line cal node to utilize target finder plugin
  * Removed unused target configuration files
  * Revised static camera extrinsic calibration node to utilize target finder plugin; removed unnecessary ChAruCo variant
  * Added reference link to find ArUco dictionary enumerations
  * Revised print statement
  * Revised data set to check for images in which the target features could not be found
  * Removed default ChaRuCo dictionary from target finder plugin for clarity
  * Updated examples to use target loader plugin
  * Updated target finder configuration files
  * Minor updates and formatting changes to example launch files
  * Combined analysis utilites for hand eye calibration examples; added homography check to static camera extrinsic calibration
  * Replace eigen_conversions dependency with tf2_eigen
  * Add comment about ChArUco dictionary
  * Updated target finder plugin unit test
  * Added drawing of identified circles to modified circle grid target finder
  * Print only covariance above 0.5 in camera intrinsic calibration tool
  * Removed data loader directory
  * Added utility to convert between YAML and XmlRpcValue
  * Updated target loader plugins; moved circle detector parameter loader into modified circle grid target finder plugin loader function
  * Minor changes to parameter loaders
  * Updated examples and command line cal to use XmlRpc to YAML conversion
  * Added circle detector parameters to example target definition
  * Updated unit test
  * Changed XmlRpc to YAML function signature for Xenial build
* Update Serialization (`#92 <https://github.com/Jmeyer1292/robot_cal_tools/issues/92>`_)
  * Added serialization of rct_optimizations types
  * Replaced serialization in rct_ros_tools with definitions in rct_optimizations
  * Added yaml-cpp to package.xml
  * Fixed bugs in serialization
  * Added equals and assignment operators for unit test
  * Added serialization unit test for hand-eye calbration problems
  * Update test conditional for rct_ros_tools
  * Added decode for extrinsic hand eye result
  * Changed template name
* Update Target Finder Interface (`#87 <https://github.com/Jmeyer1292/robot_cal_tools/issues/87>`_)
  * Added base class for target finders
  * Updated modified circle grid target finder to new interface
  * Updated ChArUco grid target finder to new interface
  * Added new ArUco grid target definition
  * Updated ArUco grid target finder to new interface
  * Updated target finder demo
  * Updated CMakeLists
  * Updated the homography correspondence sampler to be a generic grid sampler
  * Added random correspondence sampler for homography checking
  * Updated target finder unit tests
  * Removed obsolete utility function
  * Updated README
  * Propagated updates to rct_ros_tools
  * Propagated updates to rct_examples
  * Removed default constructors from target classes
  * Removed target loaders with output parameters
  * Updated examples to utilize throwing target loader functions
  * Added base class for calibration targets
  * Remove template from target finder interface; added method to return target base class
  * Added Eigen aligned allocators to target features typedef
  * Updated target definitions to inherit from target base class
  * Updated target finder classes
  * Updated unit test
  * Added dictionary parameter to ChArUco loader
  * Added sample parameter to random homography correspondence sampler
  * Move homography correspondence sampler implementation to source file
  * Fixed spelling mistakes
  * Handled error when no ChArUco targets are seen
  * Replaced asserts with exceptions in homography sampler
* Fixes to support Ubuntu 20.04 LTS and OpenCV 4 (`#85 <https://github.com/Jmeyer1292/robot_cal_tools/issues/85>`_)
  * add CI job for ROS Noetic on Focal
  * use different types, enums, and headers to avoid deprecated C API
  * fix missing std_srvs dependency
  * Delete old comment
* ChArUco Grid Target Capability (`#77 <https://github.com/Jmeyer1292/robot_cal_tools/issues/77>`_)
  * Added charuco_finder
  * Fixed observation finding issues so that ChAruco corners will be detected rather than Aruco corners. Add unit test suite which runs 3 tests on 3 different image files: one unobscured (all 24 corners visible), one partially obscured (17 corners visible), and one almost entirely obscured (only 1 corner visible). If the ChAruco corner IDs match the ones found, the test passes.
  * Added ros::package::getPath in test suites for finding image files, altered CMakeLists.txt and package.xml to incorporate ros/package.h
  * Added a struct for Charuco targets to create the board and access features of it
  * Made changes to CharucoFinder to use the new CharucoGridTarget
  * Changed test suite to use the new CharucoGridTarget and removed ROS dependency
  * Attempting to fix the issue of failing to find GTest
  * Updated CMakeLists for clarity in rct_image_tools
  * Update main.yml
  Updated to build and test rct_image_tools
  * Added ChArUco target loading specialization
  * Added an example of calibrating with a ChAruco target
  * Updated CMakeList to include the ChAruco example
  * Added directory of images and poses for ChAruco example
  * Minor updates to the ChArUco grid target definition
  * Minor changes to the ChArUco target finder
  * Revised ChArUco detection unit test
  * Revised CMakeLists
  * Renamed example files
  * Updated charuco target config file with all values
  * Revised charuco extrinsic calibration example
  * Updated charuco grid target class
  * Updated ChArUco loader for changes in grid target class
  * Added ChArUco target loader to unit test
  * Added unit tests for rct_image_tools on Xenial
  * Added charuco grid target source file to CMakeLists
  * Added numeric header for Bionic
  Co-authored-by: John <john.berkebile@swri.org>
  Co-authored-by: John Berkebile <52934312+jaberkebile@users.noreply.github.com>
* Target Loading Update (`#76 <https://github.com/Jmeyer1292/robot_cal_tools/issues/76>`_)
  * Moved the custom loading exceptions to its own file
  * Added new file for loader utils
  * Added specializable struct for creating a target object from ROS param
  * Added target loader specialization for modified circle grid target
  * Integrated target loader struct in rct_ros_tools
  * Integrated target loader struct in rct_examples
  * Revised modified circle grid target struct
  * Added unit test for target loaders
* Update/sensor noise qual (`#52 <https://github.com/Jmeyer1292/robot_cal_tools/issues/52>`_)
  * preliminary library WIP commit
  * executable outline
  * pnp3d builds
  * changed derpicated struct member
  * pnp unit test passes
  * fixed issues form stash
  * moving from image_tools to optimizations
  * Builds with main file in rct_examples, and lib in rct_optimization.
  * trying to test unit test
  * Minimal viable for 2d, but needs significant code cleaning, 3d implemented, and some improved methods
  * preliminary PR build
  * PR revision pt.1. Missing gaussian noise, stat struct rework, and further documentation
  * Changed test pose
  * more documentation
  * debugging pnp
  * angle-axis representation; still fails. Switching to quaternions
  * quaternion tests still fail
  * Quaternion Method Functioning
  * squash when things work. Temp commit: 3d pnp is very inaccurate
  * 3d noise qualification fails. Occasional innacuracy, with occasional NaN returns or 60 deg oritentation shits
  * squash me; commiting for rebase
  * local paramterization may have solved 3d accuracy
  * Removed debug prints, added pnp 3d noise test
  * raised angular tolerance to 8 degrees, for xenial compatability
  * removed commented code
  * Revised noise qualification code
  * Updated noise qualification unit test
  * Renamed to noise qualification
  * Revised noise qualification example
  * Moved and renamed noise qualification tool
  * Added example launch file for noise qualification
  * raised quaternion sampling
  Co-authored-by: ctlewis <colin.lewis@swri.org>
  Co-authored-by: mripperger <michael.ripperger@swri.org>
* Make rct_image_tools a pure CMake package (`#45 <https://github.com/Jmeyer1292/robot_cal_tools/issues/45>`_)
  * make rct_image_tools a pure cmake package
  * fix bad export of Eigen3::Eigen target
  * add Eigen3 CMake target workaround to other RCT packages
  * fix for missing EIGEN3_INCLUDE_DIRS in older versions of Eigen
  * use set_property instead of set_target_properties
* replace instances of deprecated CorrespondenceSet with Correspondence2D3D::Set (`#50 <https://github.com/Jmeyer1292/robot_cal_tools/issues/50>`_)
* Remove obsolete extrinsic hand eye optimizations (`#48 <https://github.com/Jmeyer1292/robot_cal_tools/issues/48>`_)
  * Removed extrinsic hand eye optimizations that were replace by new implementation
  * Updated RCT examples to use hand-eye optimization
  * Corrected residual error print out
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
* Add throwing versions of ROS parameter loading functions (`#21 <https://github.com/Jmeyer1292/robot_cal_tools/issues/21>`_)
* Contributors: Colin Lewis, Joseph Schornak, Josh Langsfeld, Levi Armstrong, Michael Ripperger

0.1.0 (2020-03-27)
------------------
* Update library to use Isometry3d instead of Affine3d (`#31 <https://github.com/Jmeyer1292/robot_cal_tools/issues/31>`_)
* Add github actions CI (`#33 <https://github.com/Jmeyer1292/robot_cal_tools/issues/33>`_)
  * Add github actions CI
  * Add yaml-cpp depends
* Added header directory install to rct_ros_tools package
* Merge pull request `#22 <https://github.com/Jmeyer1292/robot_cal_tools/issues/22>`_ from schornakj/fix/capture-mono16
  Special handling for mono16 images in command_line_cal.cpp
* Special handling for mono16 images
* Merge pull request `#16 <https://github.com/Jmeyer1292/robot_cal_tools/issues/16>`_ from Levi-Armstrong/feature/cameraOnly
  Add  ability for target on wrist and multiple static camera calibration in two steps
* Add utility functions and classes for getting Correspondence Sets
* Add print utilities and update examples
* Add solve mult static camera pnp example tool
* Merge pull request `#15 <https://github.com/Jmeyer1292/robot_cal_tools/issues/15>`_ from Jmeyer1292/docs/yet_more_fixups
  Fixups
* Cleaning up some of the examples
* Made the save directory configurable
* Further expanding docs
* Re-worked the command line cal tool a bit and added a launch file
* Set missing licenses in packages
* Added skeleton README to rct_ros_tools
* Merge pull request `#9 <https://github.com/Jmeyer1292/robot_cal_tools/issues/9>`_ from Jmeyer1292/feature/ros_tools
  rct_rols_tools package
* Added a rct_ros_tools package that contains the parameter loaders and the command line data collection stuff
* Contributors: Jonathan Meyer, Levi, Levi Armstrong, mripperger, schornakj
