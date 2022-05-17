^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rct_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Move utilities from ros pkg to common pkg and add missing dependencies cmake config
* Improvement Updates (`#101 <https://github.com/Jmeyer1292/robot_cal_tools/issues/101>`_)
  * Added random seed to homography validation for consistent behavior
  * Skip failed PnP optimizations in camera intrinsic calibration
  * Improved intrinsic calibration tool
  * Handled bad images and poses in data set
* DH Object Serialization (`#100 <https://github.com/Jmeyer1292/robot_cal_tools/issues/100>`_)
  * Added packages to meta package package.xml
  * Updated DH chain to be YAML serializable
  * Updated DH chain related code for addition of private default constructor
  * Added serialization for DH types
  * Removed restriction on size of inputs for YAML decoding
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
* Fixed bug in serialization of kinematic calibration data (`#84 <https://github.com/Jmeyer1292/robot_cal_tools/issues/84>`_)
* Updated pose file to use quaternions (`#81 <https://github.com/Jmeyer1292/robot_cal_tools/issues/81>`_)
* Kinematic Optimization with 6D Pose Measurements (`#79 <https://github.com/Jmeyer1292/robot_cal_tools/issues/79>`_)
  * initial optimization using kinematic_measurement observations
  * revise orientation residual calculation
  * update to use masking; fix incorrect pose residual calculation
  * fix duplicate names for new tests
  * delete duplicate function def; adjust residual threshold in test
  * fix naming and use of kinematic truths
  * revise residual calculation to use 3 offset pts
  * Added new constructor and method to DHChain class
  * Updated create chain test method
  * WIP kinematic calibration test
  * Updated kinematic measurement cost function constructor
  * Added parameter for expected DH chain offset standard deviations
  * Added data file
  * Changed optimization residual calculation
  * Updated kinematic calibration tool with better testing
  * Renamed kinematic pose calibration; added solver parameters
  * Updated kinematic calibration example with solver parameters
  * Removed residual expectation from unit test
  * Removed unused file; loaded file name from ROS parameter
  * Moved kinematic calibration node to examples directory
  * Updated kinematic calibration node to load parameters from launch file; added launch file
  * Removed TODO comment
  Co-authored-by: Joe Schornak <joe.schornak@gmail.com>
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
* Return CovarianceResult struct instead of raw Eigen matrix (`#62 <https://github.com/Jmeyer1292/robot_cal_tools/issues/62>`_)
  * add new types to describe covariance results in a more detailed way
  add covariance functions which output params with names
  add more functions returning CovarianceResult
  switch optimization results to use new CovarianceResult
  change covariance results names to be more accurate
  change stored format of covariance results
  add new functions to directly calculate covariance and correlation coefficients for whole problem
  fix segfault in toString for NamedPairs with long names
  modify extrinsic_hand_eye problem to use updated covariance functions
  update unit tests
  add separate covariance function for subset of parameter blocks
  better descriptions for new types
  Deprecate old covariance functions, add better comments for new functions
  fix warning about variable-length arrays
  fix implicit conversion warnings
  delete old covariance fns; improve labeling
  add more flexible labeling for PNP covariance
  improve unit testing for covariance results
  Move CircleFit cost fn to header, to facilitate tests
  fix generic parameter name composition
  Add tests for the different computeCovariance functions
  test covariance output in MultiStaticCamera 2-camera problem
  add separate header for covariance-specific types
  make isometry3d label string arrays const
  * fix correlation tests in PnP utests
  * update comments and docs in covariance_analysis
  * initial implementation of covariance calculation for DH chain
  * drop ill-conditioned terms when calculating covariance for DH optimization
  * fix missing linebreak on correlation result printout
  * update covariance calculations for camera_intrinsic problem
  * add extra braces for array initializers
  * change from abs to fabs
  * sort correlation coeffs above threshold in descending order
  * use stringstream instead of stringbuf and ostream
  * add double braces to initializer lists
  * fix covariance printout
  * compare abs value for NamedParams when sorting covariance coeff list
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
* Camera intrinsic calibration validation (`#51 <https://github.com/Jmeyer1292/robot_cal_tools/issues/51>`_)
  * Created function for finding transform between two virtual targets created from a single target
  * Updated documentation about test Target class
  * Added unit test for the virtual target transformation finder
  * Unit test fixup
  * WIP add observation capability
  * unit test doc fixup
  * Added camera intrinsic validation function
  * Added unit test for camera intrinsic calibration validation
  * Refactored calibration validation functions
  * Added executable for performing camera intrinsic calibration validation
  * Print fixup
  * Updated header location for PnP
  * Formatting fixup
  * Reduced error threshold slightly
  * Updated to use covariance exception
  * Updates for newly merged PRs
* Make rct_image_tools a pure CMake package (`#45 <https://github.com/Jmeyer1292/robot_cal_tools/issues/45>`_)
  * make rct_image_tools a pure cmake package
  * fix bad export of Eigen3::Eigen target
  * add Eigen3 CMake target workaround to other RCT packages
  * fix for missing EIGEN3_INCLUDE_DIRS in older versions of Eigen
  * use set_property instead of set_target_properties
* Eigen-based PnP Cost Function and Unit Test (`#54 <https://github.com/Jmeyer1292/robot_cal_tools/issues/54>`_)
  * Added Eigen-based camera point projection method
  * Updated PNP optimization to use Eigen objects
  * Added unit test for 2D PnP optimization
  * Moved PnP optimization out of experimental folder
  * Improved clarity of camera projection function
  * Fixed bug in transformation math
  * Centered camera over target
  * Updated to use an auto-diff local parameterization
* Add functions to evaluate covariance of optimization results (`#46 <https://github.com/Jmeyer1292/robot_cal_tools/issues/46>`_)
  * Add functions to compute and print covariance
  * Fixes for files deleted after rebase
  * add covariance results to new extrinsic_hand_eye optimization
  * Fix covariance output in extrinsic hand eye problem
  * remove catch for covariance exception in circle fit optimization
  * use nullptr instead of NULL constant
  * Remove FitCircleToParallelLines unit test
  * improve documentation of covariance functions
  * reduce threshold for covariance similarity check in unit test
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
* Explicitly includes Eigen into the CMakeList files (`#39 <https://github.com/Jmeyer1292/robot_cal_tools/issues/39>`_)
* Contributors: Colin Lewis, Jorge Nicho, Joseph Schornak, Levi Armstrong, Michael Ripperger

0.1.0 (2020-03-27)
------------------
* Update library to use Isometry3d instead of Affine3d (`#31 <https://github.com/Jmeyer1292/robot_cal_tools/issues/31>`_)
* Merge pull request `#16 <https://github.com/Jmeyer1292/robot_cal_tools/issues/16>`_ from Levi-Armstrong/feature/cameraOnly
  Add  ability for target on wrist and multiple static camera calibration in two steps
* Add utility functions and classes for getting Correspondence Sets
* Remove sleeps
* Add a set of image utilities and update examples
* Add print utilities and update examples
* Add solve mult static camera pnp example tool
* Add mult camera fixed relationship and wrist calibration
* Add ability to calibrate multiple static cameras to each other only
* Merge pull request `#15 <https://github.com/Jmeyer1292/robot_cal_tools/issues/15>`_ from Jmeyer1292/docs/yet_more_fixups
  Fixups
* Cleaning up some of the examples
* Set missing licenses in packages
* Added note about examples versus tools in rct_examples
* Merge pull request `#11 <https://github.com/Jmeyer1292/robot_cal_tools/issues/11>`_ from Jmeyer1292/maintain/move_pnp_default
  Replaced my PnP Solver with Levi's
* Renamed Levi's alternate interface PnP problem solver to be the default. Added documentation to match. Adjusted use cases here and there.
* Merge pull request `#10 <https://github.com/Jmeyer1292/robot_cal_tools/issues/10>`_ from Levi-Armstrong/feature/addMultiPnP
  Add multi pnp to the multi static camera example
* Show before and after reprojection images
* Add a alternative multi static camera pnp solver
* Fixup
* Add multi pnp to the multi static camera example
* Merge pull request `#7 <https://github.com/Jmeyer1292/robot_cal_tools/issues/7>`_ from Levi-Armstrong/feature/multiStaticTest
  Add extrinsic multi static camera with target on wrist utest
* Add extrinsic multi static camera with target on wrist utest
* Merge pull request `#9 <https://github.com/Jmeyer1292/robot_cal_tools/issues/9>`_ from Jmeyer1292/feature/ros_tools
  rct_rols_tools package
* Updated README to reflect file moves
* Moved all of the offline calibration tools into the tools directory under src/. I want users to be drawn to the examples as the simplest possible indication of how to use the library.
* Restructured test and a couple of other nodes
* Moved rct_examples to use the rct_ros_tools package. Removed redundant stuff.
* Merge pull request `#6 <https://github.com/Jmeyer1292/robot_cal_tools/issues/6>`_ from Jmeyer1292/feature/multi_camera_pnp
  Multi-Camera PnP
* Confirmed that the single camera PnP problem and the multi camera variety converges to the same answer
* Merge pull request `#5 <https://github.com/Jmeyer1292/robot_cal_tools/issues/5>`_ from Jmeyer1292/feature/docs_on_multi_camera
  Add Docs to for Multi Static Camera
* Docs
* Merge pull request `#4 <https://github.com/Jmeyer1292/robot_cal_tools/issues/4>`_ from Levi-Armstrong/feature/multiStaticCamera
  Add multi static camera with target on wrist calibration
* Merge branch 'master' into feature/multiStaticCamera
* Add multi static camera with target on wrist calibration
* Merge pull request `#3 <https://github.com/Jmeyer1292/robot_cal_tools/issues/3>`_ from Jmeyer1292/experiment/test
  Basic Tests Prior to Revamp
* Copied one of the stand-alone tests into a rostest for the learning experience. I intend to refactor these packages so that the rct_examples is truly only simple examples. The offline batch processing tools will move to rct_offline_tools.
* Merge pull request `#2 <https://github.com/Jmeyer1292/robot_cal_tools/issues/2>`_ from Levi-Armstrong/master
  Expose initial guess data via yaml file
* Expose initial guess data via yaml file
* Readme fixup
* Fixed RPY conventions
* Fixup
* Added some tests to show reprojection error and accuracy delta. Will refine this interface, but these are eye-opening tests.
* Updated the rct_examples for static camera
* Cleaning up the camera on wrist example
* Readme fixup
* Renamed observationset to correspondenceset to better reflect how its used
* Refacted obs finder to explicitly mention the kind of circle finder it works with
* More documentation
* Added parameter loaders and launch file for each of the other calibration examples
* Added a launch file that starts the camera on wrist calibration with some default values
* Added parameter loaders to the extrinsic camera on wrist example
* Added a couple of helper functions for loading intrinsics and target definitions from yaml files
* Added annotated image for the moving camera cell
* Reconstructed the cmakelists for the rct_examples package
* Moved the pnp example into the rct_examples directory
* Expanding on documentation
* Added image of static workcell
* Intrinsic calibration comparison with the OpenCV equivalent.
* The plumb bomb intrinsic cal is not working great. Z and focal length vary together. Do I need more/better data? Do I need to compare to OpenCV? Using the robot tool pose would constrain the solution too.
* It compiles, so it must work
* More and more documentation
* Cleaning out package xml files
* Small update
* Added a test example for static calibration
* Merge branch 'master' of https://github.com/Jmeyer1292/robot_cal_tools into static_camera
* Fixup
* Fixups
* Added a SUPER hacky data collection tool to rct examples. This will get purged shortly, but I want to test some stuff real quick
* Added test set of images / poses
* More example cleanup
* Removed debug statements
* Basic file-based image set loading works
* Started a simple little library for parsing data set from file
* Moved eigen -> pose6d functions into their own header
* Clang formatted everything
* Fully seperated the drawing and point finding functions
* Removed the output image from the detect observation class
* Renamed target definition to modified circle grid target
* Continued clean up
* Changed public API to use Eigen instead of custom types.
* Renamed Params -> Problem
* Okay, so the example appears to work
* Hacking together a demo
* Added rct_examples package
* Contributors: Jonathan Meyer, Levi, Levi Armstrong
