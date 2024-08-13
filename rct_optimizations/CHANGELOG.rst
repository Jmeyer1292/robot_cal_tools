^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rct_optimizations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2024-08-13)
------------------
* Add missing build depend ros_industrial_cmake_boilerplate
* Contributors: Levi Armstrong

0.2.0 (2024-07-25)
------------------
* fix cxx target version
* 3D reprojection analysis (`#107 <https://github.com/Jmeyer1292/robot_cal_tools/issues/107>`_)
  * Added functions for projecting 2D image features onto 3D plane
  * Added functions for computing 3D reprojection error statistics for hand-eye calibration tools
  * Updated CI
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
* Use mask only to determine tangent space in covariance calculation
* Move utilities from ros pkg to common pkg and add missing dependencies cmake config
* Fix dh parameter labels and add method for calculating covariance in tagent space
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
* Merge pull request `#94 <https://github.com/Jmeyer1292/robot_cal_tools/issues/94>`_ from marip8/fix/random-homography-correspondence
  Bug fix: random correspondence generator
* Fix bug in generation of random correspondences for homography testing
* Add explicit Boost dependency (`#86 <https://github.com/Jmeyer1292/robot_cal_tools/issues/86>`_)
  * fixes for compatibility with ROS Noetic and Foxy
  * add explicit ^Cost dependency to rct_optimizations
  * restore deleted included header
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
* Added homography check capability for 3D correspondences (`#82 <https://github.com/Jmeyer1292/robot_cal_tools/issues/82>`_)
* Temporary fix for bad homography unit test (`#80 <https://github.com/Jmeyer1292/robot_cal_tools/issues/80>`_)
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
* DH Chain Base Offset (`#71 <https://github.com/Jmeyer1292/robot_cal_tools/issues/71>`_)
  * Added fixed base offset transform to DH chain; removed fixed DH joint type
  * Added two-axis positioner DH chain
  * Revised error checking in FK function
  * Removed const specifiers from DH chain transforms and base transform to support move assignment
* Dual DH Chain Kinematic Calibration Update (`#72 <https://github.com/Jmeyer1292/robot_cal_tools/issues/72>`_)
  * Added function to create new chain from modified DH parameters
  * Add likelihood cost to DH chain kinematic calibration problem
  * Update DH chain kinematic calibration unit test
  * Added subset parameterization
  * Added local parameterization unit test
  * Fixed typo in utility function name
  * Added mask variable to kinematic calibration problem struct
  * Added subset parameterization to kinematic calibration
  * Updated unit test for DH chain calibration
* Kinematic Measurement Observation (`#70 <https://github.com/Jmeyer1292/robot_cal_tools/issues/70>`_)
  * Added kinematic measurement type for use in kinematic calibration algorithms
  * Added a function for generating a set of 6-DoF poses for kinematic calibration testing
  * Added unit tests for kinematic calibration observation generators
  * Changed variable name; added documentation
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
* Initial DH Chain Kinematic Calibration functionality (`#60 <https://github.com/Jmeyer1292/robot_cal_tools/issues/60>`_)
  * Templated DH chain class for integration with Ceres
  * Updates to DH chain unit tests
  * Changed DH offset specification to use Eigen vector
  * Created kinematic calibration cost function
  * Added unit test for kinematic calibration
  * Restructured DH chain class to remove .hpp
  * Moved chain creation utility to source file
  * Store DH transforms as objects instead of unique pointers
  * Added random header for bionic build
  * Updated to use angle axis representation rather than locally parameterized quaternion
  * Added more documentation for parameters
  * Made random number generators static
* Maximum Likelihood (`#69 <https://github.com/Jmeyer1292/robot_cal_tools/issues/69>`_)
  * Added a cost function for maximum likelihood
  * Added naive unit test for maximum likelihood
  * Improved unit test
* PnP update (`#64 <https://github.com/Jmeyer1292/robot_cal_tools/issues/64>`_)
  * Made covariance functions take const inputs
  * Added function for getting full covariance matrix between two parameters
  * Added covariance to PnP calibration
  * Added print of covariance to unit test
  * Updated printing of covariance matrices
  * Updated cost function to use unnormalized axis angle
  * Updated unit test to check covariance
  * Reduced residual expectation for perturbed case
  * Revised PnP unit test with test fixture
  * Updated CI YAML to print CTest details on test failure
  * Corrected 3D PnP optimization; added covariance calculation
  * Updated PnP 3D unit tests
  * Updated CI config
  * CI fixup
  * Modified expectation for final cost per observation for perturbed initial condition
  * Updated names of variables
  * Updated unit test to have expectations on mean and variance of optimization results for perturbed tests
* Homography Validation Update (`#66 <https://github.com/Jmeyer1292/robot_cal_tools/issues/66>`_)
  * 2d capability. Need to template for 3d. Optional outlier detection partially implemented
  * added outlier detection option
  * fixed python syntax
  * First round pr updates; still in image tools. Squash after move
  * added sampling assertion, moved to optimizations
  * linking problem in unit test
  * Updates to homography error calculation
  * Updates to homography error unit tests
  * Renamed files
  * Tests CMakeLists fixup
  * Reduced to minimum number of homography samples
  * Use matrix instead of vector when calculating k
  Co-authored-by: ctlewis <colin.lewis@swri.org>
* Switched to SVD; updated variance calculation (`#65 <https://github.com/Jmeyer1292/robot_cal_tools/issues/65>`_)
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
* Allow PoseGenerators to generate a greater variety of poses (`#58 <https://github.com/Jmeyer1292/robot_cal_tools/issues/58>`_)
  * allow PoseGenerators to generate a greater variety of poses
  * allow more orientations of observation patterns created by PoseGenerator
  * Add pose samplers to randomly vary camera Z+ rotation
  * use random rotation pose sampler in utest
  * add convenience constructors for conical and grid pose samplers
  * add RandomZRotPoseGenerator, which is a modifier for other pose generators
  * remove randomization functionality from existing PoseGenerators
  * remove unneeded public keywords
* PnP Optimization Fix (`#63 <https://github.com/Jmeyer1292/robot_cal_tools/issues/63>`_)
  * Fixed incorrect transformation math
  * Fixed incorrect semantics of target to camera transform
  * Reduced residual error expectation slightly for perturbed case
* Eigen-based PnP Cost Function and Unit Test (`#54 <https://github.com/Jmeyer1292/robot_cal_tools/issues/54>`_)
  * Added Eigen-based camera point projection method
  * Updated PNP optimization to use Eigen objects
  * Added unit test for 2D PnP optimization
  * Moved PnP optimization out of experimental folder
  * Improved clarity of camera projection function
  * Fixed bug in transformation math
  * Centered camera over target
  * Updated to use an auto-diff local parameterization
* Disabled covariance unit tests (`#61 <https://github.com/Jmeyer1292/robot_cal_tools/issues/61>`_)
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
* Pose Generation Update (`#49 <https://github.com/Jmeyer1292/robot_cal_tools/issues/49>`_)
  * Updated pose generator to be more modular
  * Added observation creators that utilize the pose generator
  * Updated hand eye unit test to use new observation creator
* Remove obsolete extrinsic hand eye optimizations (`#48 <https://github.com/Jmeyer1292/robot_cal_tools/issues/48>`_)
  * Removed extrinsic hand eye optimizations that were replace by new implementation
  * Updated RCT examples to use hand-eye optimization
  * Corrected residual error print out
* DH Parameter Kinematic Chain Representation (`#44 <https://github.com/Jmeyer1292/robot_cal_tools/issues/44>`_)
  * Created DH robot implementation
  * Updated DH robot methods to be const
  * Consolidated DH parameters into array for better future integration with Ceres
  * Moved and renamed DH robot file
  * Added helper functions for generating test DH robot
  * Added robot observation creator utility
  * Updated robot observation creator for changes to observation
  * Updated robot observation creator to use const references to DH robot
  * Updated DH unit test to use test robot functions
  * Added unit test for DH chain generating observations for extrinsic calibration
  * Added vector header to DH chain
  * Updated DH chain observation creator
  * Minor update to observation creators
  * Updated unit test to better handle bad initial random guesses
  * Added additional checks to unit test; reduced random noise level
  * Renamed DH chain observation creation functions
  * Updated documentation and random generation
* Use mt19937 algorithm to seed pose perturbation (`#47 <https://github.com/Jmeyer1292/robot_cal_tools/issues/47>`_)
  * Use Mersenne Twister algorithm for random number generation
  * Create fewer mt19937 objects
* Observation Refactor (`#38 <https://github.com/Jmeyer1292/robot_cal_tools/issues/38>`_)
  * Refactored correspondence and observation structures
  * Updated test utilities for correspondence struct change
  * Added revised hand-eye calibration method
  * Updated naming and documentation of poses in observation
  * Added template parameter for both correspondence types
  * Improved unit test implementation
  * Removed commented code
  * Added bug fix to catch bad initial guesses
  * Updated unit test to account for potential exception throw
  * Updated variable names in hand eye problem
  * Added deprecation attributes
  * Updated tests and optimization to match previous implementation
  * Updated unit test to handle bad random initial guesses for camera/target
  * Added additional checks to unit test
  * Added hand eye unit test to updated CMakeLists
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
* Update pose generation (`#43 <https://github.com/Jmeyer1292/robot_cal_tools/issues/43>`_)
  * lookat isometry
  * unit testing
  * removed commented code
  * re-implemented grid, successful unit test with new poses
  * clang-formatted
  * PR requested changes
  * refactored grid point generation, clarified unit test parameters
  * preallocate vectors, fixed integer math
  Co-authored-by: ctlewis <colin.lewis@swri.org>
* Optimization testing utilities update (`#37 <https://github.com/Jmeyer1292/robot_cal_tools/issues/37>`_)
  * Updated initialization of correspondence types
  * Refactored optimization test utilities
  * Updated extrinsic camera on wrist unit test
  * Updated extrinsic multi-static camera unit test
  * Added correspondence constructors
* Contributors: Chris Lewis, Colin Lewis, Joseph Schornak, Levi Armstrong, Michael Ripperger

0.1.0 (2020-03-27)
------------------
* Update library to use Isometry3d instead of Affine3d (`#31 <https://github.com/Jmeyer1292/robot_cal_tools/issues/31>`_)
* Added #includes to fix building in melodic
  Author:    Colin Lewis <colin.lewis@utexas.edu>
* Merge pull request `#23 <https://github.com/Jmeyer1292/robot_cal_tools/issues/23>`_ from Jmeyer1292/fixCeresDepend
  Update package.xml to include depend on libceres-dev.
* Update package.xml to include depend on libceres-dev.
* Merge pull request `#16 <https://github.com/Jmeyer1292/robot_cal_tools/issues/16>`_ from Levi-Armstrong/feature/cameraOnly
  Add  ability for target on wrist and multiple static camera calibration in two steps
* Add solve mult static camera pnp example tool
* Add mult camera fixed relationship and wrist calibration
* Add ability to calibrate multiple static cameras to each other only
* Merge pull request `#15 <https://github.com/Jmeyer1292/robot_cal_tools/issues/15>`_ from Jmeyer1292/docs/yet_more_fixups
  Fixups
* Added a test for the extrinsic camera on wrist alongside a library of tools for generating fake data
* Merge pull request `#11 <https://github.com/Jmeyer1292/robot_cal_tools/issues/11>`_ from Jmeyer1292/maintain/move_pnp_default
  Replaced my PnP Solver with Levi's
* Renamed Levi's alternate interface PnP problem solver to be the default. Added documentation to match. Adjusted use cases here and there.
* Merge pull request `#10 <https://github.com/Jmeyer1292/robot_cal_tools/issues/10>`_ from Levi-Armstrong/feature/addMultiPnP
  Add multi pnp to the multi static camera example
* Add a alternative multi static camera pnp solver
* Merge pull request `#7 <https://github.com/Jmeyer1292/robot_cal_tools/issues/7>`_ from Levi-Armstrong/feature/multiStaticTest
  Add extrinsic multi static camera with target on wrist utest
* Add extrinsic multi static camera with target on wrist utest
* Merge pull request `#6 <https://github.com/Jmeyer1292/robot_cal_tools/issues/6>`_ from Jmeyer1292/feature/multi_camera_pnp
  Multi-Camera PnP
* Added a PnP solver for multiple cameras observing the same target.
* Merge pull request `#5 <https://github.com/Jmeyer1292/robot_cal_tools/issues/5>`_ from Jmeyer1292/feature/docs_on_multi_camera
  Add Docs to for Multi Static Camera
* Docs
* Merge pull request `#4 <https://github.com/Jmeyer1292/robot_cal_tools/issues/4>`_ from Levi-Armstrong/feature/multiStaticCamera
  Add multi static camera with target on wrist calibration
* Merge branch 'master' into feature/multiStaticCamera
* Add multi static camera with target on wrist calibration
* Merge pull request `#3 <https://github.com/Jmeyer1292/robot_cal_tools/issues/3>`_ from Jmeyer1292/experiment/test
  Basic Tests Prior to Revamp
* Added a gtest and immediately found a bug. I feel like the programming Gods are telling me something
* Renamed observationset to correspondenceset to better reflect how its used
* Moves the PnP solver to its own header
* Intrinsic calibration comparison with the OpenCV equivalent.
* The plumb bomb intrinsic cal is not working great. Z and focal length vary together. Do I need more/better data? Do I need to compare to OpenCV? Using the robot tool pose would constrain the solution too.
* Working on an intrinsic calibration func
* Added stubs for intrinsic calibration functions
* Cleaning out package xml files
* Added more documentation!
* Added documentation
* WIP - more documentation
* Removed a copy paste error which inverses the pose passed to the cost
* Swapped around a few transforms in static camera, moving target
* The code exists, but does it work? I need to work out the transforms.
* Added stub for calibration function
* Moved observation pair into the types header.
* Cloned the camera on wrist func and modified it to work with 3D points. There's probably a better way to do the calibration AND a better way to share the API features but I'm still learning.
* Moved eigen -> pose6d functions into their own header
* Moved ceres math functions into their own header file.
* Clang formatted everything
* Renamed target definition to modified circle grid target
* Continued clean up
* Removed un-used functions
* Changed public API to use Eigen instead of custom types.
* Renamed Params -> Problem
* Removed original cost function test.
* Okay, so the example appears to work
* Optimization implemented
* Copying over and slightly modifying the custom circle finder code from IC2
* wip
* Initial commit
* Contributors: Jonathan Meyer, Levi, Levi Armstrong, Reid Christopher
