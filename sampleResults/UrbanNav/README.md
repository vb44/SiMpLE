# Sample UrbanNav Results
## Algorithm Configuration
This folder contains sample results for the UrbanNav dataset.\
The parameters were set as shown below in the command line arguments:

```cpp
// Configuration parameters.
sigma                  = 0.35;  // [m]
rMap                   = 2;     // [m]
rNew                   = 0.35;  // [m]
convergenceTolerance   = 1e-4;

// Hardware dependent.
maxSensorRange         = 120;   // [m]
minSensorRange         = 5;     // [m]
```
* The ***groundTruth*** folder includes the ground truth at 1Hz for each sequence in the TUM format as originally provided by the dataset.
* The ***TUM_format*** folder includes the trajectory estimates at 1Hz in the sensor frame in the TUM format for direct comparison with the ground truth.
* The ***KITTI_format*** folder includes the trajectory estimates at 10Hz in the sensor frame. This is the output from the SiMpLE algorithm.

The trajectory error values were calculated using EVO tools for comparison with the published results at https://www.doi.org/10.1109/MITS.2021.3092731 (*Point Wise or Feature Wise? A Benchmark Comparison of Publicly Available Lidar Odometry Algorithms in Urban Canyons*).

## Evaluating Results
The results can be evaluated by using the open-source EVO tools package found at: https://github.com/MichaelGrupp/evo and installed via running,
```bash
pip install evo --upgrade --no-binary evo
```
The sample results can be evaluated by running,
```bash
## Data1
# Relative translational error.
evo_rpe tum groundTruth/novatel_data1_0314.txt TUM_format/TUM_2020_03_14_16_45_35.txt

# Relative rotational error.
evo_rpe tum groundTruth/novatel_data1_0314.txt TUM_format/TUM_2020_03_14_16_45_35.txt --pose_relation angle_deg

## Data2
# Relative translational error.
evo_rpe tum groundTruth/novatel_data2_0428.txt TUM_format/TUM_2019_04_28_20_58_02.txt

# Relative rotational error.
evo_rpe tum groundTruth/novatel_data2_0428.txt TUM_format/TUM_2019_04_28_20_58_02.txt --pose_relation angle_deg
```

The expected output is shown below:
```bash
## Data1
# Relative translational error.
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max	0.649478
      mean	0.237553
    median	0.227928
       min	0.004009
      rmse	0.260640
       sse	20.176138
       std	0.107245

# Relative rotational error.
RPE w.r.t. rotation angle in degrees (deg)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max	5.909907
      mean	0.864131
    median	0.630888
       min	0.004534
      rmse	1.180547
       sse	413.926014
       std	0.804344

## Data2
# Relative translational error.
RPE w.r.t. translation part (m)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max	1.457344
      mean	0.174186
    median	0.162476
       min	0.002255
      rmse	0.243579
       sse	28.834750
       std	0.170265

# Relative rotational error.
RPE w.r.t. rotation angle in degrees (deg)
for delta = 1 (frames) using consecutive pairs
(not aligned)

       max	3.267425
      mean	0.296394
    median	0.145826
       min	0.002035
      rmse	0.502792
       sse	122.860557
       std	0.406141
```