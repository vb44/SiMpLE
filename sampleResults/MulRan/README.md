# Sample MulRan Results
This folder contains sample results for the MulRan dataset.\
The parameters were set as shown below:

```cpp
// Configuration parameters.
sigma                  = 0.35;  // [m]
rMap                   = 2;     // [m]
rNew                   = 0.5;   // [m]
convergenceTolerance   = 1e-3;
minSensorRange         = 10;    // [m]

// Hardware dependent.
maxSensorRange         = 120;   // [m]
```
* The ***groundTruthInSensorFrame*** folder includes the ground truth for each sequence transformed into the sensor frame to allow for comparison with the trajectory estimate provided by SiMpLE.
* The ***simpleOutputs*** folder includes the trajectory estimates in the sensor frame. This is the output from the SiMpLE algorithm.

The trajectory error values were calculated using KITTI's error metric.
Please see the ***devkit*** folder for evaluating the results. 