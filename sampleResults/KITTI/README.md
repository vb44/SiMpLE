# Sample KITTI Results
This folder contains sample results for the KITTI dataset and the corresponding configuration files.\
***Note that the kitti argument is required to correct the scans for the KITTI dataset\*.*** \
The parameters were set as shown below for the real-time tests:

```cpp
// Configuration parameters.
sigma                  = 0.3;   // [m]
rMap                   = 2;     // [m]
rNew                   = 0.5;   // [m]
convergenceTolerance   = 1e-3;
minSensorRange         = 10;    // [m]

// Hardware dependent.
maxSensorRange         = 80;    // [m]
```
* The ***groundTruth*** folder includes the ground truth for each sequence provided by the KITTI dataset (https://www.cvlibs.net/datasets/kitti/eval_odometry.php).
* The ***simpleOutputs*** folder includes the trajectory estimates in the sensor frame. This is the output from the SiMpLE algorithm.
* The ***estimatesInGroundTruthFrame*** folder includes the trajectory estimates in the camera (ground truth) frame - these are the sensor frame estimates with the registration applied. This is to allow for a direct comparison with the provided ground truth.

The trajectory error values were calculated using KITTI's error metric.
Please see the ***devkit*** folder for evaluating the results. 

\* The KITTI scans have an intrinsic calibration error that needs to be corrected, which otherwise leads to a greater translational error. This is noted in IMLS-SLAM (https://arxiv.org/pdf/1802.08633.pdf).  