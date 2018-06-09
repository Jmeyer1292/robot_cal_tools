# Robot Calibration Tools
A loosely connected bundle of tools for calibrating cameras to (industrial) robots and vice-versa with a focus on being easy to *integrate*  into more complex applications.

This work is based on the work of Dr. Chris Lewis @drchrislewis (see [industrial_calibration](https://github.com/ros-industrial/industrial_calibration)) and modifications made by Geoffrey Chiou in [IC2](https://github.com/geoffreychiou/IC2/). 

The focus and philosophy of this library is provide to provide a suite of stand-alone "optimization functions" that take as input structure full of well-documented arguments and produce an answer that is your calibration. How you collect data and what you do with the answer is left to you.

## Installation
This library is meant to be used within the ROS ecosystem, but the core optimizations depends on only [Ceres-Solver](http://ceres-solver.org/installation.html) and its dependencies. Follow the link for instructions to install Ceres. Afterwards, clone this package into your workspace and build with catkin.

The built-in "target finder" uses OpenCV under the hood. 

## Quick-Start
Please read the [calibration primer](./cal_primer.md) for a description of terminology and conventions used in this library. Afterwards, please visit the documentation for the calibration you wish to solve.

If you need to do intrinsic calibration of your sensor, then please use ROS' built in method from [here](http://wiki.ros.org/camera_calibration). Alternatively, use OpenCV's [calibrateCamera](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibratecamera) function on which the ROS utility is built.

Currently supported calibrations
 - Extrinsic calibration of a 2D camera on robot wrist
 - Extrinsic calibration of a 3D camera on robot wrist
 - Extrinsic calibration of a static 2D camera in a robot work cell
 
 See the [readme of `rct_examples`](rct_examples/README.md) for more information.

## Development Plan
 1. Replicate the OpenCV intrinsic calibration
 2. Implement Qt GUI based on [IC2](https://github.com/geoffreychiou/IC2/) to speed calibration process in simple scenarios.
 3. Provide examples of qualifiying the accuracy of a calibrated system
 4. Investigate robot kinematic calibration (ala [Mike Ferguson' package](https://github.com/mikeferguson/robot_calibration))
 5. Provide tools explicitly for robot tool calibration (for the moment, please see my super simple one at [tool_point_calibration](https://github.com/Jmeyer1292/tool_point_calibration)
