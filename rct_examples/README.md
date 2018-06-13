# Robot Calibration Tools Examples
## Introduction
Calibrate your camera intrinsically first. See the front page readme and calibration primer for more information.

This document describes, with pictures, the "experimental setup" for common calibrations. 

***

## Extrinsic Camera on Wrist
#### Goal
Find the transform from the robot wrist, *tool0*, to the camera.
 
#### Workcell Setup
![Moving Camera, Static Target Workcell Setup](docs/moving_camera_cell.png)

#### Software Setup
TODO

#### Validation
TODO

***

## Extrinsic Static Camera
#### Goal
Find the transform from the robot base frame, *base_link*, to the *camera optical frame*.
 
#### Workcell Setup
![Static Camera Workcell Setup](docs/static_camera_cell.png)

Sorry for the paint drawings, but I hope it gets the idea across. Note the orientation of the coordinate for the target and camera. Using these, reasonable initial guesses for the calibration parameters would be:

**base to camera**
```
translation = [1.5, 1.5, 0.5];
rotation = 
	[-1,  0,  0,
	  0,  0, -1,
	  0, -1,  0];
``` 

Remember that you can think of the columns of the rotation matrix as the axes of the new coordinate frame in the old system. So in the above the first column `[-1, 0, 0]` is the new X in the old frame, which is -X so -1, 0, 0. The second column is the new +Y in the old frame and the third column is the new +Z in the old frame.

A guess at **wrist to target** would thus be:
```
translation = [0.25, 0.25, 0];
rotation = 
	[ 1,  0,  0
	  0   0,  1
	  0, -1,  0];
```

#### Software Setup
TODO

#### Validation

TODO
