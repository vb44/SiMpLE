# Sample UQ Results
This folder contains sample results for the UQ dataset.\
The parameters were set as shown below:

VLP-16
```cpp
// Configuration parameters.
sigma                  = 12;    // [m]
rMap                   = 2;     // [m]
rNew                   = 0.5;   // [m] the halfRnew test uses rNew = 0.25 m
convergenceTolerance   = 1e-3;
minSensorRange         = 10;     // [m]

// Hardware dependent.
maxSensorRange         = 120;    // [m]
```

Livox
```cpp
// Configuration parameters.
sigma                  = 0.3;   // [m]
rMap                   = 2;     // [m]
rNew                   = 0.5;   // [m]
convergenceTolerance   = 1e-3;
minSensorRange         = 0;     // [m]

// Hardware dependent.
maxSensorRange         = 120;   // [m]
```

* The ***simpleOutputs*** folder includes the SiMpLE estimates in the sensor frame for both sensors.

The UQ dataset scans can be downloaded from [here](https://drive.google.com/drive/folders/1PgECQIySs72Qz-CXhAsAKSq_fRi5S3oA?usp=sharing).
The folder contains the raw VLP-16 and Livox scans in the KITTI format for comparison with the sensor trajectory results shown in the paper.