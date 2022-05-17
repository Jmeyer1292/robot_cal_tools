^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rct_image_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Improve finding the origin of a modified circle grid (`#104 <https://github.com/Jmeyer1292/robot_cal_tools/issues/104>`_)
  * Compare corner relative sizes rather than absolute sizes
  * Code cleanup based on PR feedback
  * Give tolerance parameter for when assessing equality
* Move utilities from ros pkg to common pkg and add missing dependencies cmake config
* Fix dh parameter labels and add method for calculating covariance in tagent space
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
* add a constructor for CharucoGridTarget that takes a cv::aruco::CharucoBoard object (`#93 <https://github.com/Jmeyer1292/robot_cal_tools/issues/93>`_)
* Remove contours with less than 5 points (`#91 <https://github.com/Jmeyer1292/robot_cal_tools/issues/91>`_)
* Add explicit Boost dependency (`#86 <https://github.com/Jmeyer1292/robot_cal_tools/issues/86>`_)
  * fixes for compatibility with ROS Noetic and Foxy
  * add explicit ^Cost dependency to rct_optimizations
  * restore deleted included header
* Add alternative constructor for ArUcoGridTarget (`#90 <https://github.com/Jmeyer1292/robot_cal_tools/issues/90>`_)
  * add aruco_grid_target constructor that takes ArUco GridBoard
  * move mapArucoIdsToObjPts function to anonymous namespace
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
* Unit Test Determinism (`#88 <https://github.com/Jmeyer1292/robot_cal_tools/issues/88>`_)
  * Added optional random seed to DHTransform struct
  * Use defined seed for random generator in unit tests
  * Moved image directory target compile definition to test target
* Fixes to support Ubuntu 20.04 LTS and OpenCV 4 (`#85 <https://github.com/Jmeyer1292/robot_cal_tools/issues/85>`_)
  * add CI job for ROS Noetic on Focal
  * use different types, enums, and headers to avoid deprecated C API
  * fix missing std_srvs dependency
  * Delete old comment
* Circle Detector Update (`#78 <https://github.com/Jmeyer1292/robot_cal_tools/issues/78>`_)
  * Added documentation to circle detector
  * General clean up of circle detection code
  * Changed 2 detector parameters to improve usability
  * Added check to see if detected blob matches the ellipse model within a specifiable tolerance
  * Added debug capability to circle detector
  * Added optional debug flag to image observation finder
  * Updated image observation finder test node
  * Updated default circle detector parameters to work for example
  * Changed thresholding step parameter to be more intuitive
  * Revised structure of circle detector
  * Removed debug parameter from image observation class
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
* Make rct_image_tools a pure CMake package (`#45 <https://github.com/Jmeyer1292/robot_cal_tools/issues/45>`_)
  * make rct_image_tools a pure cmake package
  * fix bad export of Eigen3::Eigen target
  * add Eigen3 CMake target workaround to other RCT packages
  * fix for missing EIGEN3_INCLUDE_DIRS in older versions of Eigen
  * use set_property instead of set_target_properties
* replace instances of deprecated CorrespondenceSet with Correspondence2D3D::Set (`#50 <https://github.com/Jmeyer1292/robot_cal_tools/issues/50>`_)
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
* Explicitly includes Eigen into the CMakeList files (`#39 <https://github.com/Jmeyer1292/robot_cal_tools/issues/39>`_)
* Contributors: Jorge Nicho, Joseph Schornak, Levi Armstrong, Michael Ripperger, marrts

0.1.0 (2020-03-27)
------------------
* Update library to use Isometry3d instead of Affine3d (`#31 <https://github.com/Jmeyer1292/robot_cal_tools/issues/31>`_)
* Reject when findCirclesGrid creates duplicates (`#30 <https://github.com/Jmeyer1292/robot_cal_tools/issues/30>`_)
  * Reject when opencv finds duplicate circles per https://github.com/opencv/opencv/issues/4775
* Allow customizing circle detector parameters (`#29 <https://github.com/Jmeyer1292/robot_cal_tools/issues/29>`_)
  * Move circle_detector.h to include
  * Allow specifying params for circle detection
  * Load circle params from a yaml file
  * Make sure circleColor loads as an int
  The yaml library loads it as an ascii character
  * Throw exception with nicer message in optionalLoad
  * Remove accidentally added clang-format file
* Merge pull request `#27 <https://github.com/Jmeyer1292/robot_cal_tools/issues/27>`_ from schornakj/feature/create-inverted-circle-grids
  Add option to draw CircleGrid calibration target as white dots on a black background
* Merge pull request `#26 <https://github.com/Jmeyer1292/robot_cal_tools/issues/26>`_ from schornakj/feature/generic-aruco-cal
  Add new observation finder to detect ArUco gridboards
* Added #includes to fix building in melodic
  Author:    Colin Lewis <colin.lewis@utexas.edu>
* Allow drawing dot target as white dots on black background
* use cv::Ptr to ArUco gridboard objects
* Add ArUco GridBoard detector tool for finding 2D-to-3D correspondences in images of ArUco GridBoards
  Follows the same pattern as the circle grid finder tool, but returns a map matching the integer IDs of detected ArUco markers to a vector of four corner positions.
* Merge pull request `#16 <https://github.com/Jmeyer1292/robot_cal_tools/issues/16>`_ from Levi-Armstrong/feature/cameraOnly
  Add  ability for target on wrist and multiple static camera calibration in two steps
* Add utility functions and classes for getting Correspondence Sets
* Add a set of image utilities and update examples
* Add solve mult static camera pnp example tool
* Merge pull request `#15 <https://github.com/Jmeyer1292/robot_cal_tools/issues/15>`_ from Jmeyer1292/docs/yet_more_fixups
  Fixups
* Set missing licenses in packages
* Updating rct_image_tools docs
* Merge pull request `#10 <https://github.com/Jmeyer1292/robot_cal_tools/issues/10>`_ from Levi-Armstrong/feature/addMultiPnP
  Add multi pnp to the multi static camera example
* Add multi pnp to the multi static camera example
* Refacted obs finder to explicitly mention the kind of circle finder it works with
* Moved the pnp example into the rct_examples directory
* Moves the PnP solver to its own header
* Cleaning out package xml files
* Added a home-grown version of the solvePnP problem in OpenCV
* Added an experimental function for computing the pose of a target using the PnP method
* Added Jeremy Zoss' calibration target generation script.
* Clang formatted everything
* Fully seperated the drawing and point finding functions
* Continued wip on obs finder
* Disabed drawing code - in process of moving it to unique function
* Removed the output image from the detect observation class
* More cleanup and file re-arrangement
* Renamed target definition to modified circle grid target
* Changed public API to use Eigen instead of custom types.
* Okay, so the example appears to work
* Hacking together a demo
* Plumbed the target definition into the circle finder
* Plumbed observations from CV circle finder to observer interface
* Image recongition appears to work
* Added observation test node
* Copying over and slightly modifying the custom circle finder code from IC2
* Contributors: Doug Smith, Jonathan Meyer, Joseph Schornak, Levi, Levi Armstrong, Michael Ripperger, Reid Christopher, schornakj
