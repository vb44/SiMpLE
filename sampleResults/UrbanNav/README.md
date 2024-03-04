# Sample UrbanNav Results
## Algorithm Configuration
This folder contains sample results for the UrbanNav dataset.\
The parameters were set as shown below:

```cpp
// Configuration parameters.
sigma                  = 0.35;  // [m]
rMap                   = 2;     // [m]
rNew                   = 0.5;   // [m]
convergenceTolerance   = 1e-3;
minSensorRange         = 5;     // [m]

// Hardware dependent.
maxSensorRange         = 120;   // [m]
```
* The ***groundTruth*** folder includes the ground truth at 1Hz for each sequence in the TUM format as originally provided by the dataset.
* The ***tumFormat*** folder includes the trajectory estimates at 1Hz in the sensor frame in the TUM format for direct comparison with the ground truth.
* The ***simpleOutputs*** folder includes the trajectory estimates at 10Hz in the sensor frame. This is the output from the SiMpLE algorithm.

The trajectory error values were calculated using EVO tools for comparison with the published results at https://www.doi.org/10.1109/MITS.2021.3092731 (*Point Wise or Feature Wise? A Benchmark Comparison of Publicly Available Lidar Odometry Algorithms in Urban Canyons*).

## Evaluating Results
The results can be evaluated by using the open-source EVO tools package found at: https://github.com/MichaelGrupp/evo and installed via running,
```bash
pip install evo --upgrade --no-binary evo
```
The sample results can be evaluated using the sample code provided at devkit/testScripts/evalUrbanNav.sh