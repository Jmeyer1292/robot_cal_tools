# RCT Image Tools
## Description
This package provides some support for extracting "target features" from images of targets. Nominally, we use the "modified circle grid" target. See the discussion of target
conventions in [the calibration primer](../cal_primer.md).

Jeremy Zoss wrote an awesome target creation script that can be found [here](script/calibration_target.py). See the file's docs for more information.

## Examples
Given this input image:

![Input-Image](./docs/input.png)

When running your calibration you should see this:

![Output-Image](./docs/output.png)

Note that the large dot is labeled as the origin and is in the bottom left corner if you were to rotate the first point to the top left.
***It is possible for this to screw up*** when points are observed from very skew angles. Make sure it looks right or you will have bad
convergence.

