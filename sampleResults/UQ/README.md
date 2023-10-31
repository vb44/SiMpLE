# Sample UQ Results
This folder contains sample results for the UQ dataset.\
The parameters were set as shown below in the command line arguments:

VLP-16
```cpp
// Configuration parameters.
sigma                  = 12;    // [m]
rMap                   = 2;     // [m]
rNew                   = 0.35;  // [m]
convergenceTolerance   = 1e-4;

// Hardware dependent.
maxSensorRange         = 80;    // [m]
minSensorRange         = 0;     // [m]
```

Livox
```cpp
// Configuration parameters.
sigma                  = 0.3;   // [m]
rMap                   = 2;     // [m]
rNew                   = 0.35;  // [m]
convergenceTolerance   = 1e-4;

// Hardware dependent.
maxSensorRange         = 120;   // [m]
minSensorRange         = 0;     // [m]
```

* The ***sensorFrame*** folder includes the SiMpLE estimates in the sensor frame for both sensors.

The UQ dataset scans can be downloaded from https://drive.google.com/drive/folders/1PgECQIySs72Qz-CXhAsAKSq_fRi5S3oA?usp=sharing.
The folder contains the raw VLP-16 and Livox scans in the KITTI format for comparison with the sensor trajectory results shown in the paper.