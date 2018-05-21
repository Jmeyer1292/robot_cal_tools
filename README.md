# Robot Calibration Tools
A loosely connected bundle of tools for calibrating cameras to (industrial) robots and vice-versa.

This is based on the work of Dr. Chris Lewis @drchrislewis (see [industrial_calibration](https://github.com/ros-industrial/industrial_calibration)) and modifications made by Geoffrey Chiou in [IC2](https://github.com/geoffreychiou/IC2/). 

The focus and philosophy of this library is provide to provide a suite of stand-alone "optimization functions" that take as input structure full of well-documented arguments and produce an answer that is your calibration. How you collect data and what you do with the answer is left to you.

## Installation
This library is meant to be used within the ROS ecosystem, but the core optimizations depends on only [Ceres-Solver](http://ceres-solver.org/installation.html) and its dependencies. Follow the link for instructions to install Ceres. Afterwards, clone this package into your workspace and build with catkin.

## Quick-Start
Please read the [calibration primer](./cal_primer.md) for a description of terminology and conventions used in this library. Afterwards, please visit the documentation for the calibration you wish to solve.

Table of supported calibrations:
 - Extrinsic Calibration of 2D Camera on Robot Wrist

## Development Plan
TODO

